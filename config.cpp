#include "config.h"
#include <Preferences.h>
#include <cstring>

Preferences prefs;
device_config_t device_config;

void config_set_defaults() {
  strcpy(device_config.device_name, "ESP Damper");

  strcpy(device_config.wifi_ssid, "");
  strcpy(device_config.wifi_pass, "");

  strcpy(device_config.mqtt_server, "192.168.1.123");
  device_config.mqtt_port = 1883;

  strcpy(device_config.mqtt_topic_rx, "/home/damper_cmd");
  strcpy(device_config.mqtt_topic_tx, "/home/damper");

  device_config.dark_mode = false;

  device_config.mqtt_user[0] = '\0';
  device_config.mqtt_pass[0] = '\0';

  device_config.debug_verbose = false;
  device_config.ack_en = true;

  // AC controller model
  // Supported values:
  //   "tac910" = Tadiran TAC-910
  //   "vat6"   = Twitoplast Opal VAT-6
  strcpy(device_config.ac_model, "tac910");

  device_config.enable_discovery = true;
}

void config_begin() {
  prefs.begin("config", false);
}

void config_load() {
  if (!prefs.getBool("valid", false)) {
    config_set_defaults();
    config_save();
    return;
  }

  // Pre-populate so fields added after the stored blob was written keep
  // their defaults (NVS only overwrites as many bytes as it stored).
  config_set_defaults();

  prefs.getBytes("cfg", &device_config, sizeof(device_config));

  // Backward compatibility:
  // Older saved configs do not have ac_model initialized.
  if (device_config.ac_model[0] == '\0') {
    strcpy(device_config.ac_model, "tac910");
    config_save();
  }

  // Safety fallback:
  // If corrupted or unknown value is stored, revert to TAC-910.
  if (strcmp(device_config.ac_model, "tac910") != 0 &&
      strcmp(device_config.ac_model, "vat6") != 0) {
    strcpy(device_config.ac_model, "tac910");
    config_save();
  }

  // Safety fallback for old/corrupt port.
  if (device_config.mqtt_port == 0) {
    device_config.mqtt_port = 1883;
    config_save();
  }
}

void config_save() {
  prefs.putBytes("cfg", &device_config, sizeof(device_config));
  prefs.putBool("valid", true);
}

void config_clear() {
  // Wipe the entire "config" namespace
  prefs.clear();
  prefs.putBool("valid", false);
}