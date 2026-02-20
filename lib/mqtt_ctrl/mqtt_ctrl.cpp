#include "mqtt_ctrl.h"
#include "../../include/version.h"

static MqttCtrl* g_mqttInstance = nullptr;

MqttCtrl::MqttCtrl() : client_(wifiClient_) {
    // Generate Device ID from MAC
    uint64_t chipid = ESP.getEfuseMac();
    char buf[13];
    snprintf(buf, sizeof(buf), "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
    deviceId_ = String(buf);
    g_mqttInstance = this;
}

void MqttCtrl::begin(const MqttConfig& config, const String& deviceName) {
    config_ = config;
    deviceName_ = deviceName;
    if (config_.enabled && config_.server.length() > 0) {
        client_.setServer(config_.server.c_str(), config_.port);
        client_.setBufferSize(4096); // Status JSON is large
        client_.setCallback([](char* topic, uint8_t* payload, unsigned int length) {
            if (g_mqttInstance) g_mqttInstance->callback_(topic, payload, length);
        });
    }
}

void MqttCtrl::callback_(char* topic, uint8_t* payload, unsigned int length) {
    String p = "";
    for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
    if (commandCb_) {
        commandCb_(String(topic), p);
    }
}

void MqttCtrl::setConfig(const MqttConfig& config) {
    bool serverChanged = (config_.server != config.server || config_.port != config.port || config_.user != config.user || config_.pass != config.pass);
    config_ = config;
    if (!config_.enabled) {
        client_.disconnect();
    } else {
        client_.setBufferSize(4096);
        if (serverChanged) {
            client_.disconnect();
            client_.setServer(config_.server.c_str(), config_.port);
            discoverySent_ = false; 
        }
    }
}

void MqttCtrl::setDeviceName(const String& name) {
    if (deviceName_ != name) {
        deviceName_ = name;
        discoverySent_ = false; // Name changed -> resend discovery
    }
}

bool MqttCtrl::isConnected() {
    return client_.connected();
}

void MqttCtrl::loop() {
    if (!config_.enabled || config_.server.length() == 0) return;

    if (!client_.connected()) {
        uint32_t now = millis();
        if (now - lastReconnectAttempt_ > 5000) {
            lastReconnectAttempt_ = now;
            reconnect_();
        }
    } else {
        client_.loop();
        if (!discoverySent_) {
            sendDiscovery();
            discoverySent_ = true;
        }
    }
}

void MqttCtrl::reconnect_() {
    if (WiFi.status() != WL_CONNECTED) return;

    Serial.print("[MQTT] Connecting to ");
    Serial.print(config_.server);
    Serial.print("...");

    String clientId = "Lumina-" + deviceId_;
    String willTopic = getBaseTopic_() + "/status";
    bool connected = false;
    
    if (config_.user.length() > 0) {
        connected = client_.connect(clientId.c_str(), config_.user.c_str(), config_.pass.c_str(), 
                                    willTopic.c_str(), 0, true, "offline");
    } else {
        connected = client_.connect(clientId.c_str(), nullptr, nullptr, 
                                    willTopic.c_str(), 0, true, "offline");
    }

    if (connected) {
        Serial.println("connected");
        discoverySent_ = false; // Trigger discovery send in loop
        client_.publish(willTopic.c_str(), "online", true); // Retained online status
        client_.subscribe((getBaseTopic_() + "/update/cmd").c_str());
    } else {
        Serial.print("failed, rc=");
        Serial.println(client_.state());
    }
}

String MqttCtrl::getBaseTopic_() {
    return "lumina/" + deviceId_;
}

void MqttCtrl::publishState(const JsonObject& state) {
    if (!client_.connected()) return;

    String topic = getBaseTopic_() + "/state";
    String payload;
    serializeJson(state, payload);
    if (!client_.publish(topic.c_str(), payload.c_str())) {
        Serial.println("[MQTT] Publish state FAILED (payload too large?)");
    }
}

void MqttCtrl::publishState(const String& payload) {
    if (!client_.connected()) return;
    String topic = getBaseTopic_() + "/state";
    if (!client_.publish(topic.c_str(), payload.c_str())) {
        Serial.println("[MQTT] Publish state FAILED (payload too large?)");
    }
}

void MqttCtrl::publishDiscovery_(const char* component, const char* objectId, const char* name, const char* unit, const char* devClass, const char* stateClass, const char* icon, const char* entCat, int precision) {
    // Topic: homeassistant/<component>/<node_id>/<object_id>/config
    // Replace dots in objectId for the topic path but keep them for val_tpl
    String sanId = String(objectId);
    sanId.replace('.', '_');
    String topic = "homeassistant/" + String(component) + "/lumina_" + deviceId_ + "/" + sanId + "/config";
    
    JsonDocument doc;
    doc["name"] = name;
    doc["has_entity_name"] = true;
    doc["uniq_id"] = "lumina_" + deviceId_ + "_" + sanId;
    doc["stat_t"] = getBaseTopic_() + "/state";
    doc["avty_t"] = getBaseTopic_() + "/status";
    
    if (strcmp(objectId, "upd_latest") == 0) {
        doc["val_tpl"] = "{{ value_json.upd_latest if value_json.update_available else 'Aktuell' }}";
    } else if (strcmp(component, "binary_sensor") == 0) {
        doc["val_tpl"] = "{{ 'ON' if value_json." + String(objectId) + " else 'OFF' }}";
    } else {
        doc["val_tpl"] = "{{ value_json." + String(objectId) + " }}";
    }
    
    if (unit) doc["unit_of_meas"] = unit;
    if (devClass) doc["dev_cla"] = devClass;
    if (stateClass) doc["stat_cla"] = stateClass;
    if (icon) doc["icon"] = icon;
    if (entCat) doc["ent_cat"] = entCat;
    if (precision >= 0) doc["sug_dsp_prc"] = precision;

    JsonObject dev = doc["dev"].to<JsonObject>();
    dev["ids"] = "lumina_" + deviceId_;
    dev["name"] = deviceName_;
    dev["mdl"] = "LuminaGrowX";
    dev["sw"] = FW_VERSION;
    dev["mf"] = "SleXx88";

    String payload;
    serializeJson(doc, payload);
    client_.publish(topic.c_str(), payload.c_str(), true); // Retain discovery
}

void MqttCtrl::publishUpdateDiscovery_(const char* objectId, const char* name) {
    String sanId = String(objectId);
    sanId.replace('.', '_');
    String topic = "homeassistant/update/lumina_" + deviceId_ + "/" + sanId + "/config";
    
    JsonDocument doc;
    doc["name"] = name;
    doc["has_entity_name"] = true;
    doc["uniq_id"] = "lumina_" + deviceId_ + "_" + sanId;
    
    String stateTopic = getBaseTopic_() + "/state";
    
    // Versions information
    doc["ins_v_t"] = stateTopic;
    doc["ins_v_tpl"] = "{{ value_json.fw }}";
    doc["lat_v_t"] = stateTopic;
    doc["lat_v_tpl"] = "{{ value_json.upd_latest }}";
    
    // Status (in_progress: true/false)
    doc["in_pr_t"] = stateTopic;
    doc["in_pr_tpl"] = "{{ 'true' if value_json.upd_status == 'installing' else 'false' }}";
    
    // Availability
    doc["avty_t"] = getBaseTopic_() + "/status";
    
    // Progress
    doc["pr_t"] = stateTopic;
    doc["pr_tpl"] = "{{ value_json.upd_progress }}";
    
    doc["dev_cla"] = "firmware";
    
    // Link to GitHub
    doc["rel_u"] = "https://github.com/SleXx88/LuminaGrowX/releases/latest";
    
    // Command
    doc["cmd_t"] = getBaseTopic_() + "/update/cmd";
    doc["payload_install"] = "install";

    JsonObject dev = doc["dev"].to<JsonObject>();
    dev["ids"] = "lumina_" + deviceId_;
    dev["name"] = deviceName_;
    dev["mdl"] = "LuminaGrowX";
    dev["sw"] = FW_VERSION;
    dev["mf"] = "SleXx88";

    String payload;
    serializeJson(doc, payload);
    client_.publish(topic.c_str(), payload.c_str(), true);
}

void MqttCtrl::publishButtonDiscovery_(const char* objectId, const char* name, const char* icon, const char* entCat, const char* payload, const char* avtyTpl) {
    String sanId = String(objectId);
    sanId.replace('.', '_');
    String topic = "homeassistant/button/lumina_" + deviceId_ + "/" + sanId + "/config";
    
    JsonDocument doc;
    doc["name"] = name;
    doc["has_entity_name"] = true;
    doc["uniq_id"] = "lumina_" + deviceId_ + "_" + sanId;
    
    // Command topic
    doc["cmd_t"] = getBaseTopic_() + "/update/cmd";
    doc["payload_press"] = payload;
    
    if (avtyTpl) {
        // Multi-Availability: Device online AND Update available
        JsonArray avty = doc["availability"].to<JsonArray>();
        
        JsonObject av1 = avty.add<JsonObject>();
        av1["t"] = getBaseTopic_() + "/status";
        
        JsonObject av2 = avty.add<JsonObject>();
        av2["t"] = getBaseTopic_() + "/state";
        av2["val_tpl"] = avtyTpl;
    } else {
        doc["avty_t"] = getBaseTopic_() + "/status";
    }
    
    if (icon) doc["icon"] = icon;
    if (entCat) doc["ent_cat"] = entCat;

    JsonObject dev = doc["dev"].to<JsonObject>();
    dev["ids"] = "lumina_" + deviceId_;
    dev["name"] = deviceName_;
    dev["mdl"] = "LuminaGrowX";
    dev["sw"] = FW_VERSION;
    dev["mf"] = "SleXx88";

    String p;
    serializeJson(doc, p);
    client_.publish(topic.c_str(), p.c_str(), true);
}

void MqttCtrl::sendDiscovery() {
    if (!client_.connected()) return;
    Serial.println("[MQTT] Sending HA Discovery...");

    // Basic Sensors
    publishDiscovery_("sensor", "temp_c", "Temperatur", "°C", "temperature", "measurement", nullptr, nullptr, 1);
    publishDiscovery_("sensor", "humi_rh", "Luftfeuchtigkeit", "%", "humidity", "measurement", nullptr, nullptr, 1);
    publishDiscovery_("sensor", "vpd_kpa", "VPD", "kPa", "pressure", "measurement", nullptr, nullptr, 2);
    publishDiscovery_("sensor", "dew_c", "Taupunkt", "°C", "temperature", "measurement", nullptr, nullptr, 1);
    
    publishDiscovery_("sensor", "temp_out_c", "Temp Außen", "°C", "temperature", "measurement", nullptr, nullptr, 1);
    publishDiscovery_("sensor", "humi_out_rh", "RH Außen", "%", "humidity", "measurement", nullptr, nullptr, 1);

    // Fans & Light
    publishDiscovery_("sensor", "fan_pct", "Lüfter", "%", nullptr, "measurement", "mdi:fan", nullptr, 0);
    publishDiscovery_("sensor", "fan_rpm", "Lüfter RPM", "rpm", nullptr, "measurement", "mdi:fan", nullptr, 0);
    publishDiscovery_("sensor", "fan2_pct", "Lüfter 2", "%", nullptr, "measurement", "mdi:fan", nullptr, 0);
    publishDiscovery_("sensor", "fan2_rpm", "Lüfter 2 RPM", "rpm", nullptr, "measurement", "mdi:fan", nullptr, 0);
    publishDiscovery_("sensor", "fan3_pct", "Lüfter 3", "%", nullptr, "measurement", "mdi:fan", nullptr, 0);
    publishDiscovery_("sensor", "light_pct", "Licht", "%", nullptr, "measurement", "mdi:led-strip", nullptr, 0);
    
    // Distance
    publishDiscovery_("sensor", "tof_mm", "Abstand", "mm", "distance", "measurement", "mdi:ruler", nullptr, 0);
    
    // Binary Status
    publishDiscovery_("binary_sensor", "door_open", "Tür", nullptr, "door", nullptr);
    publishDiscovery_("binary_sensor", "pump_on", "Pumpe (Luft)", nullptr, "running", nullptr);
    publishDiscovery_("binary_sensor", "light_on", "LED Beleuchtung", nullptr, nullptr, nullptr, "mdi:led-on");
    publishDiscovery_("binary_sensor", "fan_on", "Lüfter Status", nullptr, "running", nullptr);

    // Stepper
    publishDiscovery_("sensor", "stepper.pos_mm", "Lampen Höhe", "mm", nullptr, "measurement", "mdi:arrow-up-down", nullptr, 1);
    publishDiscovery_("binary_sensor", "stepper.moving", "Motor fährt", nullptr, "moving", nullptr);
    publishDiscovery_("binary_sensor", "stepper.homing", "Referenzfahrt", nullptr, nullptr, nullptr, "mdi:home-search");
    publishDiscovery_("binary_sensor", "stepper.uart_ok", "Motor UART OK", nullptr, "connectivity", nullptr, nullptr, "diagnostic");

    // Grow & Info
    publishDiscovery_("sensor", "grow.phase", "Phase", nullptr, nullptr, nullptr, "mdi:sprout");
    publishDiscovery_("sensor", "grow.day", "Tag", "d", nullptr, "total_increasing", "mdi:calendar-today", nullptr, 0);
    publishDiscovery_("sensor", "grow.total_days", "Gesamttage", "d", nullptr, nullptr, "mdi:calendar-range", "diagnostic", 0);
    publishDiscovery_("sensor", "seed", "Sorte", nullptr, nullptr, nullptr, "mdi:seed");

    // Update Entity & Manual Button (Button stays prominent)
    publishUpdateDiscovery_("firmware", "Firmware Update");
    publishButtonDiscovery_("start_update", "Firmware Update starten", "mdi:update", nullptr, "install", "{{ 'online' if value_json.update_available else 'offline' }}");
    
    // Additional Update Sensors (Progress in main, Latest in diagnostic)
    publishDiscovery_("binary_sensor", "update_available", "Update verfügbar", nullptr, "update", nullptr, "mdi:package-down", "diagnostic");
    publishDiscovery_("sensor", "upd_latest", "Neueste Version", nullptr, nullptr, nullptr, "mdi:github", "diagnostic");
    publishDiscovery_("sensor", "upd_progress", "Update Fortschritt", "%", nullptr, nullptr, "mdi:progress-download", nullptr, 0);

    // System Diagnostic
    publishDiscovery_("sensor", "health.state", "System Status", nullptr, nullptr, nullptr, "mdi:heart-pulse", "diagnostic");
    publishDiscovery_("binary_sensor", "health.modules.dac", "Modul: LED-Treiber", nullptr, "connectivity", nullptr, nullptr, "diagnostic");
    publishDiscovery_("binary_sensor", "health.modules.sht_in", "Modul: SHT Innen", nullptr, "connectivity", nullptr, nullptr, "diagnostic");
    publishDiscovery_("binary_sensor", "health.modules.sht_out", "Modul: SHT Außen", nullptr, "connectivity", nullptr, nullptr, "diagnostic");
    publishDiscovery_("binary_sensor", "health.modules.tof", "Modul: ToF-Sensor", nullptr, "connectivity", nullptr, nullptr, "diagnostic");
    publishDiscovery_("binary_sensor", "health.modules.rtc", "Modul: Echtzeituhr", nullptr, "connectivity", nullptr, nullptr, "diagnostic");
    
    publishDiscovery_("sensor", "esp_temp", "Chip Temperatur", "°C", "temperature", "measurement", "mdi:cpu-64-bit", "diagnostic", 1);
    publishDiscovery_("sensor", "rssi", "WLAN Signal", "dBm", "signal_strength", "measurement", nullptr, "diagnostic", 0);
    publishDiscovery_("sensor", "uptime_s", "Laufzeit", "s", "duration", "total_increasing", nullptr, "diagnostic", 0);
    publishDiscovery_("sensor", "ip", "IP Adresse", nullptr, nullptr, nullptr, "mdi:ip-network", "diagnostic");
    publishDiscovery_("sensor", "ssid", "WLAN SSID", nullptr, nullptr, nullptr, "mdi:wifi", "diagnostic");
    publishDiscovery_("binary_sensor", "wifi_connected", "WLAN verbunden", nullptr, "connectivity", nullptr, nullptr, "diagnostic");
    publishDiscovery_("binary_sensor", "internet_ok", "Internet OK", nullptr, "connectivity", nullptr, nullptr, "diagnostic");
}
