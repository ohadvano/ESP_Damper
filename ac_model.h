#pragma once

#include <Arduino.h>

enum AcModel : uint8_t {
  AC_MODEL_TAC910 = 0,
  AC_MODEL_VAT6   = 1
};

enum AcMode : uint8_t {
  AC_MODE_UNKNOWN = 0,
  AC_MODE_COOL    = 1,
  AC_MODE_HEAT    = 2
};

AcModel ac_model_from_string(const char* s);
const char* ac_model_to_string(AcModel model);
const char* ac_model_display_name(AcModel model);

AcMode ac_mode_from_string(const String& s);
const char* ac_mode_to_string(AcMode mode);

uint8_t ac_mqtt_temp_to_rmt(AcModel model, AcMode mode, uint8_t mqttTemp);
uint8_t ac_rmt_temp_to_mqtt(AcModel model, AcMode mode, uint8_t rmtTemp);

AcMode ac_rmt_mode_to_mqtt(AcModel model, uint8_t rmtTemp);