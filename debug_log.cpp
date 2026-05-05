#include "debug_log.h"
#include "config.h"
#include <ArduinoJson.h>

WebLogSerialClass DebugSerial;

// --------- Live web log storage ---------

static String web_lines[WEB_LOG_LINES];
static uint32_t web_seq[WEB_LOG_LINES] = {0};
static volatile uint8_t web_head = 0;
static volatile uint32_t web_next_seq = 1;

static volatile uint32_t web_ui_last_seen_ms = 0;

static String partial_line;

static portMUX_TYPE web_log_mux = portMUX_INITIALIZER_UNLOCKED;


// --------- Temporary boot/pre-UI buffer ---------

static String boot_lines[WEB_BOOT_LOG_LINES];
static uint8_t boot_head = 0;
static bool boot_wrapped = false;
static bool boot_closed = false;
static bool network_ready_seen = false;
static uint32_t boot_started_ms = 0;
static uint32_t network_ready_ms = 0;


// --------- Internal helpers ---------

static bool boot_log_hard_expired() {
  return boot_started_ms != 0 &&
         (millis() - boot_started_ms) > WEB_BOOT_LOG_MAX_AGE_MS;
}

static bool boot_log_grace_expired() {
  return network_ready_seen &&
         network_ready_ms != 0 &&
         (millis() - network_ready_ms) > WEB_BOOT_LOG_GRACE_MS;
}

static bool boot_log_should_discard() {
  return boot_log_hard_expired() || boot_log_grace_expired();
}

static void boot_log_clear() {
  for (uint8_t i = 0; i < WEB_BOOT_LOG_LINES; i++) {
    boot_lines[i] = "";
  }

  boot_head = 0;
  boot_wrapped = false;
  boot_closed = true;
}

static void web_log_store_line_raw(const String& line) {
  if (line.length() == 0) return;

  portENTER_CRITICAL(&web_log_mux);

  uint8_t idx = web_head;
  web_lines[idx] = line;
  web_seq[idx] = web_next_seq++;
  web_head = (web_head + 1) % WEB_LOG_LINES;

  portEXIT_CRITICAL(&web_log_mux);
}

static void boot_log_store_line(const String& text) {
  if (!device_config.debug_verbose) return;
  if (text.length() == 0) return;
  if (boot_closed) return;

  if (boot_started_ms == 0) {
    boot_started_ms = millis();
  }

  if (boot_log_should_discard()) {
    boot_log_clear();
    return;
  }

  String line;
  line.reserve(WEB_LOG_LINE_MAX);

  line += millis();
  line += " ms | ";
  line += text;

  if (line.length() > WEB_LOG_LINE_MAX) {
    line = line.substring(0, WEB_LOG_LINE_MAX - 3);
    line += "...";
  }

  boot_lines[boot_head] = line;
  boot_head++;

  if (boot_head >= WEB_BOOT_LOG_LINES) {
    boot_head = 0;
    boot_wrapped = true;
  }
}

static void boot_log_flush_to_web_log_once() {
  if (boot_closed) return;

  if (boot_log_should_discard()) {
    boot_log_clear();
    return;
  }

  web_log_store_line_raw("--- buffered boot log start ---");

  if (boot_wrapped) {
    for (uint8_t i = boot_head; i < WEB_BOOT_LOG_LINES; i++) {
      if (boot_lines[i].length()) {
        web_log_store_line_raw(boot_lines[i]);
      }
    }
  }

  for (uint8_t i = 0; i < boot_head; i++) {
    if (boot_lines[i].length()) {
      web_log_store_line_raw(boot_lines[i]);
    }
  }

  web_log_store_line_raw("--- buffered boot log end ---");
  boot_log_clear();
}

static void web_log_store_line(const String& text) {
  if (!device_config.debug_verbose) return;
  if (text.length() == 0) return;

  if (!web_log_ui_active()) {
    boot_log_store_line(text);
    return;
  }

  String line;
  line.reserve(WEB_LOG_LINE_MAX);

  line += millis();
  line += " ms | ";

  if (text.length() > WEB_LOG_LINE_MAX - 32) {
    line += text.substring(0, WEB_LOG_LINE_MAX - 40);
    line += "...";
  } else {
    line += text;
  }

  web_log_store_line_raw(line);
}


// --------- UI/network activity tracking ---------

void web_log_mark_network_ready() {
  if (network_ready_seen) return;

  network_ready_seen = true;
  network_ready_ms = millis();
}

void web_log_mark_ui_active() {
  web_ui_last_seen_ms = millis();
}

bool web_log_ui_active() {
  return (millis() - web_ui_last_seen_ms) < WEB_LOG_UI_ACTIVE_MS;
}

void web_log_mark_ui_active_and_clear_if_needed() {
  bool was_inactive = !web_log_ui_active();

  web_ui_last_seen_ms = millis();

  if (was_inactive) {
    web_log_clear();
    boot_log_flush_to_web_log_once();
  }
}


// --------- Public web log API ---------

void web_log_add_line(const String& line) {
  web_log_store_line(line);
}

void web_log_write_char(char c) {
  if (!device_config.debug_verbose) return;

  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    web_log_store_line(partial_line);
    partial_line = "";
    return;
  }

  partial_line += c;

  if (partial_line.length() >= WEB_LOG_LINE_MAX - 32) {
    web_log_store_line(partial_line);
    partial_line = "";
  }
}

void web_log_write(const uint8_t* data, size_t len) {
  if (!data || len == 0) return;

  for (size_t i = 0; i < len; i++) {
    web_log_write_char((char)data[i]);
  }
}

String web_log_json() {
  StaticJsonDocument<8192> doc;
  JsonArray arr = doc.createNestedArray("lines");

  portENTER_CRITICAL(&web_log_mux);

  uint8_t start = web_head;

  for (uint8_t n = 0; n < WEB_LOG_LINES; n++) {
    uint8_t idx = (start + n) % WEB_LOG_LINES;

    if (web_seq[idx] == 0) {
      continue;
    }

    JsonObject o = arr.createNestedObject();
    o["seq"] = web_seq[idx];
    o["text"] = web_lines[idx];
  }

  portEXIT_CRITICAL(&web_log_mux);

  String out;
  serializeJson(doc, out);
  return out;
}

void web_log_clear() {
  portENTER_CRITICAL(&web_log_mux);

  for (uint8_t i = 0; i < WEB_LOG_LINES; i++) {
    web_lines[i] = "";
    web_seq[i] = 0;
  }

  web_head = 0;
  web_next_seq = 1;

  portEXIT_CRITICAL(&web_log_mux);

  partial_line = "";
}


// --------- RAW RMT logging kept for later ---------

void raw_rmt_log_add(uint8_t ch,
                     const rmt_symbol_word_t* symbols,
                     size_t count,
                     bool accepted_len) {
  // Disabled for now. The function remains so rx.cpp can keep calling it.
  // Later this can be re-enabled to push RMT HEX into the same web log.
  (void)ch;
  (void)symbols;
  (void)count;
  (void)accepted_len;
}
