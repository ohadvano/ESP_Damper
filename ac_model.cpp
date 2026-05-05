#include "ac_model.h"

AcModel ac_model_from_string(const char* s) {
  if (!s) return AC_MODEL_TAC910;

  String v = String(s);
  v.trim();
  v.toLowerCase();

  if (v == "vat6" || v == "vat-6") return AC_MODEL_VAT6;

  return AC_MODEL_TAC910;
}

const char* ac_model_to_string(AcModel model) {
  switch (model) {
    case AC_MODEL_VAT6:
      return "vat6";

    case AC_MODEL_TAC910:
    default:
      return "tac910";
  }
}

const char* ac_model_display_name(AcModel model) {
  switch (model) {
    case AC_MODEL_VAT6:
      return "Twitoplast Opal VAT-6";

    case AC_MODEL_TAC910:
    default:
      return "Tadiran TAC-910";
  }
}

AcMode ac_mode_from_string(const String& s) {
  String v = s;
  v.trim();
  v.toLowerCase();

  if (v == "cool") return AC_MODE_COOL;
  if (v == "heat") return AC_MODE_HEAT;

  return AC_MODE_UNKNOWN;
}

const char* ac_mode_to_string(AcMode mode) {
  switch (mode) {
    case AC_MODE_COOL:
      return "cool";

    case AC_MODE_HEAT:
      return "heat";

    case AC_MODE_UNKNOWN:
    default:
      return "unknown";
  }
}

uint8_t ac_mqtt_temp_to_rmt(AcModel model, AcMode mode, uint8_t mqttTemp) {
  switch (model) {
    case AC_MODEL_VAT6:
      if (mode == AC_MODE_HEAT) {
        return mqttTemp + 59;
      }

      if (mode == AC_MODE_COOL) {
        return mqttTemp + 27;
      }

      return mqttTemp + 27;

    case AC_MODEL_TAC910:
      return mqttTemp - 5;

    default:
      return mqttTemp;
  }
}

uint8_t ac_rmt_temp_to_mqtt(AcModel model, AcMode mode, uint8_t rmtTemp) {
  switch (model) {
    case AC_MODEL_VAT6:
      if (mode == AC_MODE_HEAT) {
        return rmtTemp - 59;
      }

      if (mode == AC_MODE_COOL) {
        return rmtTemp - 27;
      }

      return 0;

    case AC_MODEL_TAC910:
      return rmtTemp + 5;

    default:
      return rmtTemp;
  }
}

AcMode ac_rmt_mode_to_mqtt(AcModel model, uint8_t rmtTemp) {
  switch (model) {
    case AC_MODEL_VAT6:
      if (rmtTemp >= 80 && rmtTemp <= 94) {
        return AC_MODE_HEAT;
      }
      else if (rmtTemp >= 48 && rmtTemp <= 62) {
        return AC_MODE_COOL;
      }
      else {
        return AC_MODE_UNKNOWN;
      }

    case AC_MODEL_TAC910:
      return AC_MODE_UNKNOWN;

    default:
      return AC_MODE_UNKNOWN;
  }
}