#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>
#include "defaults.h"
#include "globals.h"
#include "ac_model.h"

void clear_mqtt_data(mqtt_data_t mqtt_data);
String binaryToHexGroups(const String &binStr);
uint32_t binToDec(const String &binStr);
bool validateAndParseFrames(std::vector<String> &frames, rx_data_t &rx_data, const char* model);

// Damper-angle <-> HA fan-mode label mapping.
//   angle 0 -> "1"    (Low)
//   angle 1 -> "2"    (Moderate)
//   angle 2 -> "3"    (High / full)
//   angle 3 -> "Auto"
String damper_angle_to_fan_label(uint8_t angle);
uint8_t fan_label_to_damper_angle(const String& label);
