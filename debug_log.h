#pragma once

#include <Arduino.h>
#include "globals.h"

#define WEB_LOG_LINES        200
#define WEB_LOG_LINE_MAX     256
#define WEB_LOG_UI_ACTIVE_MS 3000

// Temporary boot/pre-UI buffer behavior.
#define WEB_BOOT_LOG_LINES          32
#define WEB_BOOT_LOG_GRACE_MS       30000UL
#define WEB_BOOT_LOG_MAX_AGE_MS     120000UL

// Main web debug log API.
void web_log_write(const uint8_t* data, size_t len);
void web_log_write_char(char c);
void web_log_add_line(const String& line);

String web_log_json();
void web_log_clear();

// Live streaming (SSE): invoked once per stored line, after the line is in
// the ring buffer. Called outside the ring-buffer mutex so callers may push
// to network handlers freely.
typedef void (*web_log_stream_cb_t)(const String& line);
void web_log_set_stream_callback(web_log_stream_cb_t cb);

bool web_log_ui_active();
void web_log_mark_ui_active();
void web_log_mark_ui_active_and_clear_if_needed();
void web_log_mark_network_ready();

// Serial tee object. Use through serial_intercept.h.
class WebLogSerialClass : public Print {
public:
  void begin(unsigned long baud) {
    ::Serial.begin(baud);
  }

  void begin(unsigned long baud, uint32_t config) {
    ::Serial.begin(baud, config);
  }

  operator bool() const {
    return (bool)::Serial;
  }

  int available() {
    return ::Serial.available();
  }

  int read() {
    return ::Serial.read();
  }

  int peek() {
    return ::Serial.peek();
  }

  void flush() {
    ::Serial.flush();
  }

  size_t write(uint8_t c) override {
    ::Serial.write(c);
    web_log_write_char((char)c);
    return 1;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    ::Serial.write(buffer, size);
    web_log_write(buffer, size);
    return size;
  }

  using Print::write;
};

extern WebLogSerialClass DebugSerial;

// Backward-compatible rawlog names used by web.cpp.
inline String raw_rmt_log_json() {
  return web_log_json();
}

inline void raw_rmt_log_clear() {
  web_log_clear();
}

inline bool raw_rmt_log_ui_active() {
  return web_log_ui_active();
}

inline void raw_rmt_log_mark_ui_active() {
  web_log_mark_ui_active();
}

inline void raw_rmt_log_mark_ui_active_and_clear_if_needed() {
  web_log_mark_ui_active_and_clear_if_needed();
}

// Kept for later RAW RMT logging. Currently implemented as a no-op.
void raw_rmt_log_add(uint8_t ch,
                     const rmt_symbol_word_t* symbols,
                     size_t count,
                     bool accepted_len);
