// Latest ElegantOTA requires manual configuration to work in AsyncWebServer mode.
//
// 1. In ElegantOTA.h, set:
//        #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
//
// 2. In ElegantOTA.cpp, uncomment the `/update` GUI route inside the
//    `#if ELEGANTOTA_USE_ASYNC_WEBSERVER == 1` block (remove the `/* ... */`
//    wrapping `_server->on("/update", HTTP_GET, ...)`). Without this, the
//    /ota/start and /ota/upload endpoints work but the browser-friendly
//    upload page at http://<device>/update returns 404.


#include "wireless.h"
#include "rx.h"
#include "tx.h"
#include "helpers.h"
#include "config.h"
#include "reset.h"
#include "serial_intercept.h"

#define VERSION "1.1.18"

const char* FW_VERSION_STR = VERSION;
bool single_shot = true;
mqtt_data_t mqtt_data[NUM_CHANNELS];

void setup() {
    Serial.begin(115200);
    delay(500);
    
    led_task_start();
    led_set_blink(250);

    config_begin();
    config_load();
    
    if(device_config.debug_verbose)
      Serial.println("Debug verbosity enabled");
    if(device_config.debug_verbose && device_config.timing_diagnostics_en)
      Serial.println("Timing diagnostics enabled");

    reset_task_start();

    Serial.println();
    Serial.print("Current FW version: ");
    Serial.println(VERSION);
    Serial.println("Starting WiFi...");
        
    wireless_setup();

    Serial.println("Starting RMT RX/TX multi channel...");
    // gpio_install_isr_service() must be called before gpio_isr_handler_add,
    // which happens inside ack_gpio_init(). Moved before the loop for that
    // reason (previously it ran after the loop, which was fine only because
    // ack_gpio_init was dynamic — now that it's permanent, order matters).
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        rmt_rx_channel_config(ch, CHANNEL_GPIOS[ch]);
        ack_irq_init(ch);
        ack_gpio_init(ch);   // Permanent ISR install — see rx.cpp:ack_gpio_isr
        rx_last_call[ch] = micros();
    }
    
    Serial.print("Device serving channels: 0-");
    Serial.println(NUM_CHANNELS - 1);

    rx_set_callback(on_rx_frame); // Debug data callback

    led_set_on();
}

void loop() {
    wireless_loop();

    // Handle RX -> MQTT transaction
    uint32_t parse_start_us = micros();
    bool parsed = parseRMTData(device_config.ac_model);
    uint32_t parse_us = micros() - parse_start_us;
    if (parsed) {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
            if (rx_new_data[ch]) {
                rx_new_data[ch] = false;
                if (device_config.rx_ack_en) {
                    ack_irq_start(ch);
                    // The ISR was installed once at setup and is always
                    // running; no per-op ack_gpio_init call needed. If the
                    // ACK already fired while parseRMTData was running,
                    // ack_irq_start() catches it retrospectively and sets
                    // ack_active immediately (no timer started).
                    if (device_config.debug_verbose && device_config.timing_diagnostics_en) {
                        uint32_t frame_to_start = ack_dbg_start_time[ch] - ack_dbg_frame_end_time[ch];
                        const char* how = ack_active[ch] ? "retrospective" : "prospective";
                        public_debug_message(String("Ch ") + ch + " RX ACK arm mode=" + how +
                                             " frame_to_start=" + String(frame_to_start) + "us"
                                             " parse_us=" + String(parse_us) + "us" +
                                             (ack_active[ch]
                                                 ? String(" retro_width=") + String(ack_width[ch]) + "us"
                                                 : String("")));
                    }
                }

                mqtt_data[ch].ch = ch;
                mqtt_data[ch].temp = rx_data[ch].temp;
                mqtt_data[ch].state = rx_data[ch].state;
                mqtt_data[ch].fan = rx_data[ch].fan;
                mqtt_data[ch].mode = rx_data[ch].mode;
                mqtt_data[ch].pending = true;
            }
        }
    }

    // Handle MQTT -> TX transaction
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (tx_requests[ch].pending && !mqtt_data[ch].pending) {
            uint32_t tx_start_us = micros();
            send_tx_payload(ch, CHANNEL_GPIOS[ch], tx_requests[ch].temp, tx_requests[ch].state == "on", tx_requests[ch].fan, tx_requests[ch].mode);
            uint32_t tx_end_us = micros();

            reconfig_rmt_rx_channel(ch, CHANNEL_GPIOS[ch]);

            if (device_config.tx_ack_en) {
                ack_irq_start(ch);
                if (device_config.debug_verbose && device_config.timing_diagnostics_en) {
                    uint32_t tx_to_start = ack_dbg_start_time[ch] - tx_end_us;
                    const char* how = ack_active[ch] ? "retrospective" : "prospective";
                    public_debug_message(String("Ch ") + ch + " TX ACK arm mode=" + how +
                                         " tx_start=" + String(tx_start_us) +
                                         " tx_end=" + String(tx_end_us) +
                                         " tx_to_start=" + String(tx_to_start) + "us" +
                                         (ack_active[ch]
                                             ? String(" retro_width=") + String(ack_width[ch]) + "us"
                                             : String("")));
                }
            }

            mqtt_data[ch].ch = ch;
            mqtt_data[ch].temp = tx_requests[ch].temp;
            mqtt_data[ch].state = tx_requests[ch].state;
            mqtt_data[ch].fan = tx_requests[ch].fan;
            mqtt_data[ch].mode = ac_mode_to_string(tx_requests[ch].mode);
            mqtt_data[ch].pending = true;
        }
    }

    // Handle ACK signal and send MQTT message.
    // Distinguish RX vs TX by tx_requests[ch].pending: TX sets it before we
    // reach here; RX never touches it. That determines which ack_en flag
    // gates the publish (rx_ack_en for RX, tx_ack_en for TX).
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (mqtt_data[ch].pending) {
            bool is_tx = tx_requests[ch].pending;
            bool ack_gated = is_tx ? device_config.tx_ack_en : device_config.rx_ack_en;
            if (ack_active[ch] || !ack_gated) {
                public_message(mqtt_data[ch]);
                tx_requests[ch].pending = false;
                ack_active[ch] = false;
                mqtt_data[ch].pending = false;
                // Cancel the 2.2 s NACK timer and suppress any phantom
                // timeout that fired in the same loop iteration as the
                // ISR set ack_active. Prevents the "command failed"
                // message from following a successful publish.
                if (ack_gated) {
                    ack_irq_stop(ch);
                    ack_timeout[ch] = false;
                }
            }
        }
    }

    // Handle NACK signal
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (ack_timeout[ch]) {
            // Full diagnostic dump — gated on timing_diagnostics_en so it
            // doesn't spam the log in normal operation. The "command failed"
            // line below stays unconditional; HA/users should always see
            // the actual failure message.
            if (device_config.debug_verbose && device_config.timing_diagnostics_en) {
                uint32_t now_us     = micros();
                uint32_t elapsed_us = now_us - ack_dbg_start_time[ch];
                uint32_t first_fall_delta = ack_dbg_first_fall_time[ch]
                    ? (int32_t)(ack_dbg_first_fall_time[ch] - ack_dbg_start_time[ch])
                    : 0;
                uint32_t last_rise_delta = ack_dbg_last_rise_time[ch]
                    ? (int32_t)(ack_dbg_last_rise_time[ch] - ack_dbg_start_time[ch])
                    : 0;

                // Dump the pulse history ring buffer (last N completed
                // pulses, most recent last).
                String history;
                for (uint8_t i = 0; i < ACK_DBG_HISTORY; i++) {
                    uint8_t idx = (ack_dbg_pulse_history_pos[ch] + i) % ACK_DBG_HISTORY;
                    if (i) history += ",";
                    history += String(ack_dbg_pulse_history[ch][idx]);
                }

                public_debug_message(String("Ch ") + ch + " NACK diag:"
                    " elapsed=" + String(elapsed_us) + "us"
                    " first_fall_delta=" + String(first_fall_delta) + "us"
                    " last_rise_delta=" + String(last_rise_delta) + "us"
                    " fall_cnt=" + String(ack_dbg_falling_count[ch]) +
                    " rise_cnt=" + String(ack_dbg_rising_count[ch]) +
                    " max_width=" + String(ack_dbg_max_width[ch]) + "us"
                    " isr_attached=" + String(ack_dbg_isr_attached[ch] ? 1 : 0) +
                    " history=[" + history + "]");
            }
            public_debug_message("Ch " + String(ch) + " command failed");

            mqtt_data[ch].pending = false;
            tx_requests[ch].pending = false;
            ack_timeout[ch] = false;
        }
    }

    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (ack_width[ch] > 0) {
            if (device_config.debug_verbose && device_config.timing_diagnostics_en) {
                // Successful ACK. Two cases:
                //   Retrospective: the ISR captured the ACK's rising edge
                //   BEFORE ack_irq_start() was called (typical for RX,
                //   where parseRMTData took ~200 ms). detect_delta is
                //   negative — the pulse ended before we armed.
                //   Prospective: the ISR fired during the 2.2 s watch
                //   window and set ack_active. detect_delta is positive.
                int32_t detect_delta = 0;
                bool retro = false;
                if (ack_dbg_last_rise_time[ch] != 0) {
                    detect_delta = (int32_t)(ack_dbg_last_rise_time[ch] - ack_dbg_start_time[ch]);
                    retro = (detect_delta < 0);
                }
                int32_t first_fall_delta = ack_dbg_first_fall_time[ch]
                    ? (int32_t)(ack_dbg_first_fall_time[ch] - ack_dbg_start_time[ch])
                    : 0;
                public_debug_message(String("Ch ") + ch + " ACK width=" + String(ack_width[ch]) + "us"
                    " mode=" + (retro ? "retrospective" : "prospective") +
                    " first_fall_delta=" + String(first_fall_delta) + "us"
                    " detect_delta=" + String(detect_delta) + "us"
                    " fall_cnt=" + String(ack_dbg_falling_count[ch]) +
                    " rise_cnt=" + String(ack_dbg_rising_count[ch]));
            }
            ack_width[ch] = 0;
        }
    }

    // Log per-loop-tick pulses > 100 ms so ordinary frame bits (~600 us)
    // don't spam the pane. Anything close to the ACK's ~1.1 s or shorter
    // near-ACK candidates still show up.
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (ack_width_dbg[ch] > 0) {
            if (device_config.debug_verbose && device_config.timing_diagnostics_en && ack_width_dbg[ch] > 100000) {
                Serial.println("Ch " + String(ch) + " PULSE width is " + String(ack_width_dbg[ch]));
            }
            ack_width_dbg[ch] = 0;
        }
    }
    
    // Send MQTT message when ESP has booted
    if (single_shot){
        public_debug_message("ESP boot done, version " + String(VERSION));
        single_shot = false;
    }
}

void on_rx_frame(uint8_t ch, const rmt_symbol_word_t* symbols, size_t num_symbols, const String& bits) {
  if (!device_config.debug_verbose) return;
  StaticJsonDocument<2048> doc;

  doc["type"] = "raw_data";
  doc["ch"]   = ch;
  doc["hex"]  = binaryToHexGroups(bits);

  JsonArray raw = doc.createNestedArray("raw");

  for (size_t i = 0; i < num_symbols; i++) {
    raw.add(symbols[i].duration0);
    raw.add(symbols[i].duration1);
  }

  doc["len"] = num_symbols;

  public_raw_message(doc);
}
