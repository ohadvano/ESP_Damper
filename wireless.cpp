#include "wireless.h"
#include "tx.h"
#include "rx.h"
#include "web.h"
#include "debug_log.h"
#include "helpers.h"
#include "serial_intercept.h"

#define AP_IP       IPAddress(192, 168, 50, 1)
#define AP_GATEWAY  IPAddress(192, 168, 50, 1)
#define AP_SUBNET   IPAddress(255, 255, 255, 0)

WiFiClient espClient;
PubSubClient client(espClient);
AsyncWebServer server(80);

tx_request_t tx_requests[NUM_CHANNELS] = {};
static void reconnect();
static void mqtt_callback(char* topic, byte* payload, unsigned int length);
static void publish_discovery();

extern const char* FW_VERSION_STR;

void wireless_setup() {
    bool wifi_sta_mode = true;
    if (device_config.wifi_ssid[0] == '\0')
        wifi_sta_mode = false;
  
    if(wifi_sta_mode)
        start_sta_mode();
    else
        start_ap_mode();

    web_begin(server, client);
    server.begin();
    web_log_mark_network_ready();
    Serial.println("HTTP server started");
    ElegantOTA.begin(&server);    // Start ElegantOTA
    
    if(wifi_sta_mode) {
        led_set_blink(100);
        client.setServer(device_config.mqtt_server, device_config.mqtt_port);
        // 3072 leaves headroom for the largest HA discovery payload (~1.8 KB)
        // plus topic and MQTT framing overhead.
        client.setBufferSize(3072);
        client.setCallback(mqtt_callback);
    }
}

void wireless_loop() {
    if(WiFi.getMode() == WIFI_AP) 
        led_set_blink(1000);
    else {
        if (!client.connected()) {
            reconnect();
            if (client.connected()) Serial.println("Reconnected");
        }
        client.loop();
    }
    ElegantOTA.loop();
}

static void reconnect() {
  int counter = 0;
  bool ok = false;

  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("WIFI is not connected: ");
      Serial.println(WiFi.status());
    }

    if (device_config.mqtt_user[0] != '\0') {
      ok = client.connect(device_config.device_name,
                          device_config.mqtt_user,
                          device_config.mqtt_pass);
    } else {
      ok = client.connect(device_config.device_name);
    }

    if (ok) {
      Serial.println("MQTT connected");
      client.unsubscribe(device_config.mqtt_topic_rx);
      client.subscribe(device_config.mqtt_topic_rx);
      publish_discovery();
    } else {
      Serial.print(" Failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }

    if(counter==200){
      delay(1000);
      Serial.println("Restarting ESP...");
      ESP.restart();
    } else {
      counter++;
    }
  }
}

static void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<100> jsonDoc;
  String jsonString;

  for (unsigned int i = 0; i < length; i++) 
      jsonString += (char)tolower(payload[i]);

  deserializeJson(jsonDoc, jsonString);
  
  uint8_t ch=0;
  uint8_t temp=0;
  String state="";
  uint8_t fan=0;
  AcMode mode = AC_MODE_UNKNOWN;

  if (jsonDoc.containsKey("ch")) {
    ch = jsonDoc["ch"].as<uint8_t>();
    if (ch >= NUM_CHANNELS) return;
  }

  // Preserve last-known temp/fan from mqtt_data when the incoming command
  // omits them (HA's "off" command template only sends {ch,state}). Prevents
  // us from echoing temp=0 back and letting HA overwrite its own setpoint.
  if (jsonDoc.containsKey("temp")) {
    temp = jsonDoc["temp"].as<uint8_t>();
  } else {
    temp = mqtt_data[ch].temp;
  }

  if (jsonDoc.containsKey("state")) {
    state = jsonDoc["state"].as<String>();
  }
  if (jsonDoc.containsKey("fan")) {
    // Fan is a string label ("low"/"medium"/"high"/"auto") — convert to
    // the wire-format damper angle 0..3 that send_tx_payload expects.
    fan = fan_label_to_damper_angle(jsonDoc["fan"].as<String>());
  } else {
    fan = mqtt_data[ch].fan;
  }

  if (jsonDoc.containsKey("mode")) {
    mode = ac_mode_from_string(jsonDoc["mode"].as<String>());
  }

  AcModel model = ac_model_from_string(device_config.ac_model);

  if (model == AC_MODEL_VAT6 && mode == AC_MODE_UNKNOWN) {
    public_debug_message("Command rejected: missing HEAT/COOL mode");
    return;
  }

  if (!tx_requests[ch].pending) {
      // Just store request
      tx_requests[ch].pending = true;
      tx_requests[ch].ch = ch;
      tx_requests[ch].temp = temp;
      tx_requests[ch].state = state;
      tx_requests[ch].fan = fan;
      tx_requests[ch].mode = mode;
  }
  else
      public_debug_message("Received duplicate command on ch " + String(ch));
}

void public_message(mqtt_data_t data) {
  StaticJsonDocument<180> doc;
  char output[180];
  
  doc["ch"] = data.ch;
  doc["temp"] = data.temp;
  doc["state"] = data.state;
  doc["fan"] = damper_angle_to_fan_label(data.fan);
  doc["mode"] = data.mode;

  serializeJson(doc, output);
  Serial.println(output);
  client.publish(device_config.mqtt_topic_tx, output);
}

void public_debug_message(String msg) {
  StaticJsonDocument<1024> doc;
  char output[1024];
  
  doc["device"] = device_config.device_name;
  doc["msg"] = msg;
  
  serializeJson(doc, output);
  Serial.println(output);
  if (!client.publish(device_config.mqtt_topic_tx, output)) {
      doc["msg"] = "MQTT publish failed!";
      serializeJson(doc, output);
      client.publish(device_config.mqtt_topic_tx, output);
  }
}

void public_raw_message(StaticJsonDocument<2048> doc) {
  char output[2048];
  
  doc["device"] = device_config.device_name;
  
  serializeJson(doc, output);
  Serial.println(output);
  if (!client.publish(device_config.mqtt_topic_tx, output)) {
      doc["msg"] = "MQTT publish failed!";
      serializeJson(doc, output);
      client.publish(device_config.mqtt_topic_tx, output);
  }
}

void start_ap_mode() {
    Serial.println("No WiFi SSID configured, entering AP mode");
    WiFi.mode(WIFI_AP);
    // Set static IP for AP mode
    if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) {
        Serial.println("Failed to configure AP IP");
    }

    WiFi.softAP("ESP_Damper_Setup");

    Serial.println("AP mode started");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    led_set_blink(1000);   // Slow blink in AP mode
}

void start_sta_mode() {
    Serial.print("Connecting to: ");
    Serial.print(device_config.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(device_config.device_name);
    WiFi.setSleep(false);
    WiFi.begin(device_config.wifi_ssid, device_config.wifi_pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println(" connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

static String device_unique_base() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac.toLowerCase();
  return String("esp_damper_") + mac;
}

static void publish_discovery() {
  const String base = device_unique_base();

  // Disable path: erase entities by publishing empty retained payloads.
  if (!device_config.enable_discovery) {
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
      String t = String("homeassistant/climate/") + base + "_ch" + ch + "/config";
      client.publish(t.c_str(), (const uint8_t*)"", 0, true);
    }
    Serial.println("HA MQTT discovery: cleared");
    return;
  }

  const AcModel model = ac_model_from_string(device_config.ac_model);
  const bool is_vat6 = (model == AC_MODEL_VAT6);

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
    const String unique_id   = base + "_ch" + ch;
    const String object_id   = String("damper_ch") + ch;
    const String config_topic = String("homeassistant/climate/") + unique_id + "/config";

    DynamicJsonDocument doc(3072);

    doc["name"]      = String("Damper Ch") + ch;
    doc["unique_id"] = unique_id;
    doc["object_id"] = object_id;

    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids  = dev.createNestedArray("identifiers");
    ids.add(base);
    dev["name"]         = device_config.device_name;
    dev["manufacturer"] = "ESP_Damper";
    dev["model"]        = "TAC-910 / VAT-6 IR bridge";
    dev["sw_version"]   = FW_VERSION_STR;

    // HVAC modes depend on the AC controller:
    //   TAC-910: the wall remote is on/off only — cool vs heat is set at
    //            the AC's central unit. Expose "off" and "auto" (= on).
    //   VAT-6  : cool/heat is encoded per-frame — expose both explicitly.
    JsonArray modes = doc.createNestedArray("modes");
    modes.add("off");
    if (is_vat6) {
      modes.add("cool");
      modes.add("heat");
    } else {
      modes.add("auto");
    }

    JsonArray fan_modes = doc.createNestedArray("fan_modes");
    fan_modes.add("Low");
    fan_modes.add("Medium");
    fan_modes.add("High");
    fan_modes.add("Auto");

    doc["min_temp"]  = 16;
    doc["max_temp"]  = 30;
    doc["temp_step"] = 1;

    const char* tx_topic = device_config.mqtt_topic_tx;
    const char* rx_topic = device_config.mqtt_topic_rx;

    const String ch_filter = String("{% if value_json.ch == ") + ch + " %}";

    // Inbound (device -> HA) state templates.
    doc["current_temperature_topic"]    = tx_topic;
    doc["current_temperature_template"] =
        ch_filter + "{{ value_json.temp }}{% endif %}";

    doc["temperature_state_topic"]      = tx_topic;
    doc["temperature_state_template"]   =
        ch_filter + "{{ value_json.temp }}{% endif %}";

    doc["mode_state_topic"] = tx_topic;
    if (is_vat6) {
      // VAT-6: mode is real; map cool/heat directly, off from state.
      doc["mode_state_template"] =
          ch_filter +
          "{% if value_json.state == 'off' %}off"
          "{% elif value_json.mode in ['cool','heat'] %}{{ value_json.mode }}"
          "{% endif %}{% endif %}";
    } else {
      // TAC-910: no cool/heat on the wire — just on/off.
      doc["mode_state_template"] =
          ch_filter +
          "{% if value_json.state == 'off' %}off{% else %}auto{% endif %}"
          "{% endif %}";
    }

    doc["fan_mode_state_topic"]    = tx_topic;
    doc["fan_mode_state_template"] =
        ch_filter + "{{ value_json.fan }}{% endif %}";

    // Outbound (HA -> device) command templates. `this.state` is the
    // current HVAC mode, `this.attributes.*` are the setpoint and fan.
    // We use `or <default>` (rather than `| int(N)`) so that a stale 0
    // value in HA's state — from any prior bug — still falls back sanely.
    doc["temperature_command_topic"] = rx_topic;
    if (is_vat6) {
      doc["temperature_command_template"] =
          String("{\"ch\":") + ch +
          ",\"temp\":{{ value | int }}"
          ",\"state\":\"on\""
          ",\"mode\":\"{{ this.state if this.state in ['cool','heat'] else 'cool' }}\""
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}";
    } else {
      doc["temperature_command_template"] =
          String("{\"ch\":") + ch +
          ",\"temp\":{{ value | int }}"
          ",\"state\":\"on\""
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}";
    }

    doc["mode_command_topic"] = rx_topic;
    if (is_vat6) {
      doc["mode_command_template"] =
          String("{% if value == 'off' %}") +
          "{\"ch\":" + ch + ",\"state\":\"off\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% else %}"
          "{\"ch\":" + ch +
          ",\"state\":\"on\""
          ",\"mode\":\"{{ value }}\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% endif %}";
    } else {
      // TAC-910: value is 'off' or 'auto'; no mode field on the wire.
      doc["mode_command_template"] =
          String("{% if value == 'off' %}") +
          "{\"ch\":" + ch + ",\"state\":\"off\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% else %}"
          "{\"ch\":" + ch +
          ",\"state\":\"on\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% endif %}";
    }

    doc["fan_mode_command_topic"] = rx_topic;
    if (is_vat6) {
      doc["fan_mode_command_template"] =
          String("{\"ch\":") + ch +
          ",\"state\":\"on\""
          ",\"fan\":\"{{ value }}\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"mode\":\"{{ this.state if this.state in ['cool','heat'] else 'cool' }}\"}";
    } else {
      doc["fan_mode_command_template"] =
          String("{\"ch\":") + ch +
          ",\"state\":\"on\""
          ",\"fan\":\"{{ value }}\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}}";
    }

    static char buffer[2560];
    size_t n = serializeJson(doc, buffer, sizeof(buffer));

    if (!client.publish(config_topic.c_str(), (const uint8_t*)buffer, n, true)) {
      Serial.print("Discovery publish failed for ch ");
      Serial.println(ch);
    }
  }

  Serial.println("HA MQTT discovery: published");
}