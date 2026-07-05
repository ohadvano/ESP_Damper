#pragma once
#include "defaults.h"

typedef struct {
    uint8_t temp;
    String state;
    uint8_t fan;
    String mode;
} rx_data_t;

typedef struct {
    uint8_t ch;
    uint8_t temp;
    String state;
    uint8_t fan;
    String mode;
    bool pending = false;
} mqtt_data_t;

// Main INO buffers
extern mqtt_data_t mqtt_data[NUM_CHANNELS];

// RX Global buffers and flags
extern rmt_symbol_word_t rx_buffer[NUM_CHANNELS][RX_BUFFER_SIZE];
extern rmt_symbol_word_t rx_data_copy[NUM_CHANNELS][RX_FRAMES][RX_BUFFER_SIZE];
extern size_t rx_data_len[NUM_CHANNELS];
extern volatile bool data_ready[NUM_CHANNELS];
extern size_t rx_data_chunk[NUM_CHANNELS];
extern size_t rx_last_call[NUM_CHANNELS];
extern rmt_channel_handle_t rx_channel[NUM_CHANNELS];
extern bool ack_active[NUM_CHANNELS];
extern bool ack_timeout[NUM_CHANNELS];
extern size_t ack_width[NUM_CHANNELS];
extern size_t ack_width_dbg[NUM_CHANNELS];

// ---- ACK-window diagnostics ----
// All timestamps are absolute micros() since boot; deltas are computed at
// log time so we can tell if the ISR was armed before or after the ACK edge.
#define ACK_DBG_HISTORY 8

extern volatile uint32_t ack_dbg_frame_end_time[NUM_CHANNELS];      // RMT saw 3rd frame complete
extern volatile uint32_t ack_dbg_start_time[NUM_CHANNELS];          // ack_irq_start() was called
extern volatile uint32_t ack_dbg_gpio_init_time[NUM_CHANNELS];      // ack_gpio_init() returned
extern volatile uint32_t ack_dbg_first_fall_time[NUM_CHANNELS];     // first falling edge in this window
extern volatile uint32_t ack_dbg_last_rise_time[NUM_CHANNELS];      // last rising edge in this window
extern volatile uint32_t ack_dbg_falling_count[NUM_CHANNELS];       // falling edges captured in window
extern volatile uint32_t ack_dbg_rising_count[NUM_CHANNELS];        // rising edges captured in window
extern volatile uint32_t ack_dbg_max_width[NUM_CHANNELS];           // max completed pulse width in window
extern volatile bool     ack_dbg_isr_attached[NUM_CHANNELS];        // is the GPIO ISR currently attached?
extern volatile uint32_t ack_dbg_pulse_history[NUM_CHANNELS][ACK_DBG_HISTORY]; // last N completed pulse widths
extern volatile uint8_t  ack_dbg_pulse_history_pos[NUM_CHANNELS];   // ring index (0..ACK_DBG_HISTORY-1)

// RX config
extern rmt_receive_config_t rx_config;

// TX Global buffers and flags
extern rmt_channel_handle_t tx_channel[NUM_CHANNELS];
extern rmt_encoder_handle_t copy_encoder[NUM_CHANNELS];
