#pragma once
#include <Arduino.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

// =========================
// Pin / Frame Configuration
// =========================
#define CH1_GPIO           ((gpio_num_t)13)  // ESP32 GPIO13 (DOIT DEVKIT V1)

#define NUM_CHANNELS        8

// DOIT ESP32 DEVKIT V1 pin map. Avoids SPI-flash pins (6-11),
// boot-strapping pins (0, 2, 5, 12, 15), and input-only pins (34-39).
static const gpio_num_t CHANNEL_GPIOS[NUM_CHANNELS] = {
    GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_17, GPIO_NUM_18,
    GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23
};

#define RX_FRAMES          3
#define RX_BUFFER_SIZE     64
#define DUMMY_FRAME        "11000000111111111110111000000000110000001111111111101110"
#define FRAME_TIMEOUT_US   400000    // 400 ms

// =========================
// TX timing (us)
// =========================
#define HEADER_LOW_US      5900
#define HEADER_HIGH_US     1800
#define BIT0_LOW_US         580
#define BIT0_HIGH_US        500
#define BIT1_LOW_US         580
#define BIT1_HIGH_US       1600
#define FOOTER_LOW_US      7800

// =========================
// RX Timing (us)
// =========================
#define HEADER_LOW_MIN     100
#define HEADER_LOW_MAX     700
#define HEADER_HIGH_MIN   1600
#define HEADER_HIGH_MAX   2200
#define FOOTER_LOW_MIN    7700
#define FOOTER_LOW_MAX    8000

#define BIT_LOW_MIN        250
#define BIT_LOW_MAX        700
#define BIT_HIGH_0_MIN     400
#define BIT_HIGH_0_MAX     900
#define BIT_HIGH_1_MIN    1450
#define BIT_HIGH_1_MAX    2200
