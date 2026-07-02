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

    reset_task_start();

    Serial.println();
    Serial.print("Current FW version: ");
    Serial.println(VERSION);
    Serial.println("Starting WiFi...");
        
    wireless_setup();

    Serial.println("Starting RMT RX/TX multi channel...");
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        rmt_rx_channel_config(ch, CHANNEL_GPIOS[ch]);
        ack_irq_init(ch);
        rx_last_call[ch] = micros();
    }
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    Serial.print("Device serving channels: 0-");
    Serial.println(NUM_CHANNELS - 1);

    rx_set_callback(on_rx_frame); // Debug data callback

    led_set_on();
}

void loop() {
    wireless_loop();

    // Handle RX -> MQTT transaction 
    if (parseRMTData(device_config.ac_model)) {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
            if (rx_new_data[ch]) {
                rx_new_data[ch] = false;
                if (device_config.ack_en) {
                    ack_irq_start(ch);
                    ack_gpio_init(ch);
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
            send_tx_payload(ch, CHANNEL_GPIOS[ch], tx_requests[ch].temp, tx_requests[ch].state == "on", tx_requests[ch].fan, tx_requests[ch].mode);

            reconfig_rmt_rx_channel(ch, CHANNEL_GPIOS[ch]);
            if (device_config.ack_en) {
                ack_irq_start(ch);
                ack_gpio_init(ch);
            }
            
            mqtt_data[ch].ch = ch;
            mqtt_data[ch].temp = tx_requests[ch].temp;
            mqtt_data[ch].state = tx_requests[ch].state;
            mqtt_data[ch].fan = tx_requests[ch].fan;
            mqtt_data[ch].mode = ac_mode_to_string(tx_requests[ch].mode);
            mqtt_data[ch].pending = true;
        }
    }

    // Handle ACK signal and send MQTT message
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (mqtt_data[ch].pending && (ack_active[ch] || !device_config.ack_en)){
            public_message(mqtt_data[ch]);
            tx_requests[ch].pending = false;
            ack_active[ch] = false;
            mqtt_data[ch].pending = false;
        }
    }

    // Handle NACK signal
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (ack_timeout[ch]) {
            public_debug_message("Ch " + String(ch) + " command failed");
            mqtt_data[ch].pending = false;
            tx_requests[ch].pending = false;
            ack_timeout[ch] = false;
        }
    }

    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (ack_width[ch] > 0) {
            if(device_config.debug_verbose)
                public_debug_message("Ch " + String(ch) + " ACK width is " + String(ack_width[ch]));
            ack_width[ch] = 0;
        }
    }

    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (ack_width_dbg[ch] > 0) {
            if(device_config.debug_verbose)
                Serial.println("Ch " + String(ch) + " PULSE width is " + String(ack_width_dbg[ch]));
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
