#include "rx.h"
#include "helpers.h"
#include "driver/gpio.h"
#include "debug_log.h"
#include "serial_intercept.h"

static rx_frame_cb_t rx_cb = nullptr;

void rx_set_callback(rx_frame_cb_t cb) {
  rx_cb = cb;
}

rmt_symbol_word_t rx_buffer[NUM_CHANNELS][RX_BUFFER_SIZE];
rmt_symbol_word_t rx_data_copy[NUM_CHANNELS][RX_FRAMES][RX_BUFFER_SIZE];
size_t rx_data_len[NUM_CHANNELS] = {0};
volatile bool data_ready[NUM_CHANNELS] = {false};
size_t rx_data_chunk[NUM_CHANNELS] = {0};
size_t rx_last_call[NUM_CHANNELS] = {0};
rmt_channel_handle_t rx_channel[NUM_CHANNELS] = {NULL};
rx_data_t rx_data[NUM_CHANNELS] = {};
volatile bool rx_new_data[NUM_CHANNELS] = {};
bool ack_active[NUM_CHANNELS] = {false};
bool ack_timeout[NUM_CHANNELS] = {false};
size_t ack_width[NUM_CHANNELS] = {0};
size_t ack_width_dbg[NUM_CHANNELS] = {0};

// ---- Permanent-ISR ACK detection state ----
// The GPIO ISR is installed once at boot (ack_gpio_init) and runs
// continuously, tracking every falling and rising edge on the channel
// GPIO. These variables are the always-on rolling state that lets
// ack_irq_start() detect an ACK retrospectively — if the ACK's edges
// arrived while parseRMTData or send_tx_payload were still running.
static volatile uint32_t last_falling_us[NUM_CHANNELS]  = {0};  // most recent falling edge
static volatile uint32_t last_rising_us[NUM_CHANNELS]   = {0};  // most recent rising edge (ended a pulse)
static volatile uint32_t last_pulse_width[NUM_CHANNELS] = {0};  // width of the most recent completed low pulse
static volatile bool     ack_watch[NUM_CHANNELS]        = {false}; // true while actively looking for an ACK
#define ACK_RETRO_WINDOW_US 500000  // accept a completed pulse ending within the last 500 ms

// ---- ACK-window diagnostics (see globals.h for meaning) ----
volatile uint32_t ack_dbg_frame_end_time[NUM_CHANNELS]  = {0};
volatile uint32_t ack_dbg_start_time[NUM_CHANNELS]      = {0};
volatile uint32_t ack_dbg_gpio_init_time[NUM_CHANNELS]  = {0};
volatile uint32_t ack_dbg_first_fall_time[NUM_CHANNELS] = {0};
volatile uint32_t ack_dbg_last_rise_time[NUM_CHANNELS]  = {0};
volatile uint32_t ack_dbg_falling_count[NUM_CHANNELS]   = {0};
volatile uint32_t ack_dbg_rising_count[NUM_CHANNELS]    = {0};
volatile uint32_t ack_dbg_max_width[NUM_CHANNELS]       = {0};
volatile bool     ack_dbg_isr_attached[NUM_CHANNELS]    = {false};
volatile uint32_t ack_dbg_pulse_history[NUM_CHANNELS][ACK_DBG_HISTORY] = {{0}};
volatile uint8_t  ack_dbg_pulse_history_pos[NUM_CHANNELS] = {0};



esp_timer_handle_t oneshot_timer[NUM_CHANNELS] = {NULL};

rmt_receive_config_t rx_config = {
    .signal_range_min_ns = 2000,
    .signal_range_max_ns = 2500000,
};

void ack_irq_init(uint8_t ch) {
    esp_timer_create_args_t timer_args = {
        .callback = &oneshot_timer_callback,
        .arg = (void*)ch,     // pass channel index here
        .dispatch_method = ESP_TIMER_TASK,
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &oneshot_timer[ch]));
}

void ack_irq_start(uint8_t ch) {
    ack_timeout[ch] = false;
    ack_active[ch]  = false;

    // Reset per-window diagnostic counters and timing markers so we can
    // reconstruct exactly what happened during this ACK window.
    ack_dbg_start_time[ch]      = micros();
    ack_dbg_first_fall_time[ch] = 0;
    ack_dbg_last_rise_time[ch]  = 0;
    ack_dbg_falling_count[ch]   = 0;
    ack_dbg_rising_count[ch]    = 0;
    ack_dbg_max_width[ch]       = 0;
    ack_dbg_pulse_history_pos[ch] = 0;
    for (uint8_t i = 0; i < ACK_DBG_HISTORY; i++) {
        ack_dbg_pulse_history[ch][i] = 0;
    }
    // Note: ack_dbg_gpio_init_time is NOT reset — it holds the boot-time
    // install timestamp for the (now permanent) ISR.

    // Retrospective: did a valid ACK complete within the last 500 ms?
    // Covers the case where parseRMTData / send_tx_payload were still
    // running when the ACK's rising edge fired. The permanent ISR has
    // already recorded it into last_pulse_width / last_rising_us.
    uint32_t now = micros();
    if (last_pulse_width[ch] > 500000 &&
        last_rising_us[ch] != 0 &&
        (now - last_rising_us[ch]) < ACK_RETRO_WINDOW_US) {
        ack_active[ch] = true;
        ack_width[ch]  = last_pulse_width[ch];
        // Mirror into diagnostics so the ACK-success log has a consistent
        // "when did we detect it" view.
        ack_dbg_last_rise_time[ch] = last_rising_us[ch];
        // Consume so a stale pulse isn't reused on the next call.
        last_pulse_width[ch] = 0;
        return;
    }

    // Prospective: arm the watch flag and start the 2.2 s NACK timer.
    // The ISR is already listening; it will latch ack_active when it
    // sees a rising edge with width > 500 ms.
    ack_watch[ch] = true;
    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer[ch], 2200000));
}

void ack_irq_stop(uint8_t ch) {
    ack_timeout[ch] = false;
    ack_watch[ch]   = false;
    // Not wrapped in ESP_ERROR_CHECK: stopping a not-running timer is a
    // benign error, and this is called both when we caught an ACK
    // (timer might be running) and defensively from paths where it
    // isn't (retrospective branch didn't start it).
    esp_timer_stop(oneshot_timer[ch]);
}

void ack_gpio_init(uint8_t ch) {
    // Install the ACK GPIO ISR PERMANENTLY. Called once per channel at
    // boot from setup(). Because the ISR is always listening, ACK
    // detection can't miss the falling edge just because parseRMTData /
    // send_tx_payload are still running — the ISR captures the edge
    // into last_falling_us / last_pulse_width, and ack_irq_start() picks
    // it up retrospectively.
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CHANNEL_GPIOS[ch],
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CHANNEL_GPIOS[ch], ack_gpio_isr, (void *)(intptr_t)ch));
    ack_dbg_gpio_init_time[ch] = micros();
    ack_dbg_isr_attached[ch] = true;
}

void ack_gpio_remove(uint8_t ch) {
    // No-op: the ISR is permanently attached now. Kept as a stub so any
    // remaining callers don't break.
    (void)ch;
}

void IRAM_ATTR oneshot_timer_callback(void *arg) {
    int ch = (int)(intptr_t)arg;
    ack_timeout[ch] = true;
    ack_width[ch]   = 0;
    ack_watch[ch]   = false;
    // ISR stays attached — do NOT call ack_gpio_remove here.
}

void IRAM_ATTR ack_gpio_isr(void *arg) {
    int ch = (int)(intptr_t)arg;
    bool state = digitalRead(CHANNEL_GPIOS[ch]);
    uint32_t now = micros();

    if (state) {
        // Rising edge — the low pulse just ended.
        ack_dbg_rising_count[ch]++;
        ack_dbg_last_rise_time[ch] = now;

        // Compute width only if we have a captured falling-edge timestamp.
        if (last_falling_us[ch] != 0) {
            uint32_t width = now - last_falling_us[ch];

            // Rolling always-on state (used by retrospective check).
            last_pulse_width[ch] = width;
            last_rising_us[ch]   = now;

            // Diagnostics (per-window).
            if (width > ack_dbg_max_width[ch]) ack_dbg_max_width[ch] = width;
            uint8_t p = ack_dbg_pulse_history_pos[ch];
            ack_dbg_pulse_history[ch][p] = width;
            ack_dbg_pulse_history_pos[ch] = (p + 1) % ACK_DBG_HISTORY;
            ack_width_dbg[ch] = width;

            // Prospective ACK detection (standard path): a single clean
            // >500 ms low pulse between the most recent falling edge and
            // this rising edge. Works for the typical case.
            if (ack_watch[ch] && width > 500000) {
                ack_active[ch] = true;
                ack_watch[ch]  = false;
                ack_width[ch]  = width;
            }
        }

        // Prospective ACK detection (robust fallback): the ISR can miss
        // an intermediate rising edge under load — WiFi/MQTT preemption,
        // motor-noise bounce, or two edges arriving faster than the GPIO
        // interrupt can be serviced. When that happens, `last_falling_us`
        // gets clobbered by a spurious later falling edge (e.g. motor
        // engagement noise) and the standard width above comes out too
        // small to detect. Recover by measuring from the FIRST captured
        // falling in this watch window to now — if we're still on the
        // first captured rising, no intermediate rising was properly
        // seen, and the elapsed span exceeds 500 ms, then the bus was
        // effectively LOW for the ACK's full duration and we accept it.
        if (ack_watch[ch] &&
            ack_dbg_rising_count[ch] == 1 &&
            ack_dbg_first_fall_time[ch] != 0) {
            uint32_t alt_width = now - ack_dbg_first_fall_time[ch];
            if (alt_width > 500000) {
                ack_active[ch] = true;
                ack_watch[ch]  = false;
                ack_width[ch]  = alt_width;
            }
        }
    } else {
        // Falling edge — a new low pulse just started. Always update
        // the rolling last-falling timestamp so subsequent rising edges
        // measure width from the LATEST falling edge, not the first-ever
        // one since boot.
        last_falling_us[ch] = now;

        // Diagnostics (per-window).
        ack_dbg_falling_count[ch]++;
        if (ack_dbg_first_fall_time[ch] == 0) {
            ack_dbg_first_fall_time[ch] = now;
        }
    }
}

bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    int ch = (int)(intptr_t)user_data;
    if (ch < 0 || ch >= NUM_CHANNELS) return false;

    size_t count = edata->num_symbols;
    raw_rmt_log_add(ch, edata->received_symbols, count, count == 57);

    if (micros() - rx_last_call[ch] > FRAME_TIMEOUT_US) clear_rx_buffers(ch);
    if (count > RX_BUFFER_SIZE) count = RX_BUFFER_SIZE;
    if (count == 57) {
        memcpy(rx_data_copy[ch][rx_data_chunk[ch]], edata->received_symbols, count * sizeof(rmt_symbol_word_t));
        rx_data_len[ch] = count;
        rx_data_chunk[ch]++;
        if (rx_data_chunk[ch] == RX_FRAMES) {
            rx_data_chunk[ch] = 0;
            data_ready[ch] = true;
            // Latest RMT callback for the 3rd frame — the RMT peripheral
            // is now idle for this channel; the next thing on the bus is
            // whatever the TAC-910 is about to send.
            ack_dbg_frame_end_time[ch] = micros();
        }
    }



    rmt_receive(rx_channel[ch], rx_buffer[ch], sizeof(rx_buffer[ch]), &rx_config);
    rx_last_call[ch] = micros();
    return true;
}

void rmt_rx_channel_config(uint8_t ch, gpio_num_t gpio) {
    if (tx_channel[ch]) {
        rmt_disable(tx_channel[ch]);
        rmt_del_channel(tx_channel[ch]);
        tx_channel[ch] = NULL;
    }

    pinMode(gpio, INPUT);

    rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = RX_BUFFER_SIZE,
    };

    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_callback };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel[ch]));
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel[ch], &cbs, (void*)(intptr_t)ch));
    ESP_ERROR_CHECK(rmt_enable(rx_channel[ch]));
    ESP_ERROR_CHECK(rmt_receive(rx_channel[ch], rx_buffer[ch], sizeof(rx_buffer[ch]), &rx_config));

    Serial.printf("Configured RX channel %u: handle=%d gpio=%d\n", (unsigned)ch, int(rx_channel[ch]), int(gpio));
}

void clear_rx_buffers(uint8_t ch) {
    if (ch >= NUM_CHANNELS) return;
    rx_data_chunk[ch] = 0;
    memset(rx_data_copy[ch], 0, sizeof(rx_data_copy[ch]));
}

void reconfig_rmt_rx_channel(uint8_t ch, gpio_num_t gpio) {
    // Tear down existing and re-create
    if (ch >= NUM_CHANNELS) return;
    if (rx_channel[ch]) {
        ESP_ERROR_CHECK(rmt_disable(rx_channel[ch]));
        ESP_ERROR_CHECK(rmt_del_channel(rx_channel[ch]));
        rx_channel[ch] = NULL;
    }
    rmt_rx_channel_config(ch, gpio);
}

bool parseRMTData(const char* model) {
    bool data_received = false;

    // Check each channel for ready data
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        if (!data_ready[ch]) continue;

        size_t len = rx_data_len[ch];
        rmt_symbol_word_t symbols[RX_BUFFER_SIZE];
        std::vector<String> frames;
        for(int chunk=0; chunk<RX_FRAMES; chunk++) {
            noInterrupts();
            memcpy(symbols, rx_data_copy[ch][chunk], len * sizeof(rmt_symbol_word_t));
            data_ready[ch] = false;
            interrupts();

            int i = 0;
            String bits = "";
            int level0 = symbols[i].level1;
            int level1 = symbols[i].level0;
            int duration0 = symbols[i].duration1;
            int duration1 = symbols[i].duration0;

            if (level0 == 0 && duration0 >= HEADER_LOW_MIN && duration0 <= HEADER_LOW_MAX &&
                level1 == 1 && duration1 >= HEADER_HIGH_MIN && duration1 <= HEADER_HIGH_MAX) {
                i++;
                bits = "";
                Serial.printf("Ch %d chunk %d\n", ch, chunk);
                while (i < (int)len) {
                    level0 = symbols[i].level1;
                    level1 = symbols[i].level0;
                    duration0 = symbols[i].duration1;
                    duration1 = symbols[i].duration0;
                    if (duration1 >= BIT_HIGH_0_MIN && duration1 <= BIT_HIGH_0_MAX) {
                        bits += "0";
                    } else if (duration1 >= BIT_HIGH_1_MIN && duration1 <= BIT_HIGH_1_MAX) {
                        bits += "1";
                    } else {
                        Serial.printf("Framing parsing error at location: %d\n", i);
                        Serial.printf("Low: %d\n", duration0);
                        Serial.printf("High: %d\n", duration1);
                        i++;
                        continue;
                    }
                    i++;
                }
            }

            if (bits.length() > 0) {
                frames.push_back(bits);
            }
            else {
                frames.push_back(DUMMY_FRAME);
                Serial.printf("Ch %d broken chunk diagnostics: level0=%d level1=%d dur0=%d dur1=%d\n", ch, level0, level1, duration0, duration1);
            }
            
            if(rx_cb) rx_cb(ch, symbols, len, bits); // Backup data for debug
        }

        if (frames.size() == RX_FRAMES) {
            Serial.printf("Complete frame decoded (ch %u):\n", (unsigned)ch);
            Serial.println("Frame 1: " + frames[0] + " (" + binaryToHexGroups(frames[0]) + ")");
            Serial.println("Frame 2: " + frames[1] + " (" + binaryToHexGroups(frames[1]) + ")");
            Serial.println("Frame 3: " + frames[2] + " (" + binaryToHexGroups(frames[2]) + ")");
            bool res = validateAndParseFrames(frames, rx_data[ch], model);
            if (res) {
              rx_new_data[ch] = true;
              data_received = true;
            }
            frames.clear();
        } else {
            Serial.printf("Invalid frames count found on ch %u!\n", (unsigned)ch);
            frames.clear();
        }
        clear_rx_buffers(ch);
    } // end channels loop
    return data_received;
}
