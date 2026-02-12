#include "mqtt_ctrl.h"
#include "../../include/version.h"

MqttCtrl::MqttCtrl() : client_(wifiClient_) {
    // Generate Device ID from MAC
    uint64_t chipid = ESP.getEfuseMac();
    char buf[13];
    snprintf(buf, sizeof(buf), "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
    deviceId_ = String(buf);
}

void MqttCtrl::begin(const MqttConfig& config, const String& deviceName) {
    config_ = config;
    deviceName_ = deviceName;
    if (config_.enabled && config_.server.length() > 0) {
        client_.setServer(config_.server.c_str(), config_.port);
        client_.setBufferSize(2048); // Status JSON is large
    }
}

void MqttCtrl::setConfig(const MqttConfig& config) {
    bool serverChanged = (config_.server != config.server || config_.port != config.port || config_.user != config.user || config_.pass != config.pass);
    config_ = config;
    if (!config_.enabled) {
        client_.disconnect();
    } else {
        client_.setBufferSize(2048);
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
    bool connected = false;
    
    if (config_.user.length() > 0) {
        connected = client_.connect(clientId.c_str(), config_.user.c_str(), config_.pass.c_str());
    } else {
        connected = client_.connect(clientId.c_str());
    }

    if (connected) {
        Serial.println("connected");
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

void MqttCtrl::publishDiscovery_(const char* component, const char* objectId, const char* name, const char* unit, const char* devClass, const char* stateClass, const char* icon) {
    // Topic: homeassistant/<component>/<node_id>/<object_id>/config
    // Replace dots in objectId for the topic path but keep them for val_tpl
    String sanId = String(objectId);
    sanId.replace('.', '_');
    String topic = "homeassistant/" + String(component) + "/lumina_" + deviceId_ + "/" + sanId + "/config";
    
    JsonDocument doc;
    doc["name"] = deviceName_ + " " + name;
    doc["uniq_id"] = "lumina_" + deviceId_ + "_" + sanId;
    doc["stat_t"] = getBaseTopic_() + "/state";
    
    if (strcmp(component, "binary_sensor") == 0) {
        doc["val_tpl"] = "{{ 'ON' if value_json." + String(objectId) + " == true else 'OFF' }}";
    } else {
        doc["val_tpl"] = "{{ value_json." + String(objectId) + " }}";
    }
    
    if (unit) doc["unit_of_meas"] = unit;
    if (devClass) doc["dev_cla"] = devClass;
    if (stateClass) doc["stat_cla"] = stateClass;
    if (icon) doc["icon"] = icon;

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

void MqttCtrl::sendDiscovery() {
    if (!client_.connected()) return;
    Serial.println("[MQTT] Sending HA Discovery...");

    // Basic Sensors
    publishDiscovery_("sensor", "temp_c", "Temperatur", "°C", "temperature", "measurement");
    publishDiscovery_("sensor", "humi_rh", "Luftfeuchtigkeit", "%", "humidity", "measurement");
    publishDiscovery_("sensor", "vpd_kpa", "VPD", "kPa", "pressure", "measurement");
    publishDiscovery_("sensor", "dew_c", "Taupunkt", "°C", "temperature", "measurement");
    
    publishDiscovery_("sensor", "temp_out_c", "Temp Außen", "°C", "temperature", "measurement");
    publishDiscovery_("sensor", "humi_out_rh", "RH Außen", "%", "humidity", "measurement");

    // Fans & Light
    publishDiscovery_("sensor", "fan_pct", "Lüfter", "%", nullptr, "measurement", "mdi:fan");
    publishDiscovery_("sensor", "fan_rpm", "Lüfter RPM", "rpm", nullptr, "measurement", "mdi:fan");
    publishDiscovery_("sensor", "fan2_pct", "Lüfter 2", "%", nullptr, "measurement", "mdi:fan");
    publishDiscovery_("sensor", "fan2_rpm", "Lüfter 2 RPM", "rpm", nullptr, "measurement", "mdi:fan");
    publishDiscovery_("sensor", "fan3_pct", "Lüfter 3", "%", nullptr, "measurement", "mdi:fan");
    publishDiscovery_("sensor", "light_pct", "Licht", "%", nullptr, "measurement", "mdi:led-strip");
    
    // Distance
    publishDiscovery_("sensor", "tof_mm", "Abstand", "mm", "distance", "measurement", "mdi:ruler");
    
    // Binary Status
    publishDiscovery_("binary_sensor", "door_open", "Tür", nullptr, "door", nullptr);
    publishDiscovery_("binary_sensor", "pump_on", "Pumpe", nullptr, "running", nullptr);
    publishDiscovery_("binary_sensor", "light_on", "Licht Status", nullptr, "light", nullptr);
    publishDiscovery_("binary_sensor", "fan_on", "Lüfter Status", nullptr, "running", nullptr);

    // Stepper
    publishDiscovery_("sensor", "stepper.pos_mm", "Lampen Höhe", "mm", nullptr, "measurement", "mdi:arrow-up-down");
    publishDiscovery_("binary_sensor", "stepper.moving", "Motor fährt", nullptr, "moving", nullptr);
    publishDiscovery_("binary_sensor", "stepper.homing", "Referenzfahrt", nullptr, nullptr, nullptr, "mdi:home-search");
    publishDiscovery_("binary_sensor", "stepper.uart_ok", "Motor UART OK", nullptr, "connectivity", nullptr);

    // Grow & Info
    publishDiscovery_("sensor", "grow.phase", "Phase", nullptr, nullptr, nullptr, "mdi:sprout");
    publishDiscovery_("sensor", "grow.day", "Tag", "d", nullptr, "total_increasing", "mdi:calendar-today");
    publishDiscovery_("sensor", "grow.total_days", "Gesamttage", "d", nullptr, nullptr, "mdi:calendar-range");
    publishDiscovery_("sensor", "seed", "Sorte", nullptr, nullptr, nullptr, "mdi:seed");

    // System Diagnostic
    publishDiscovery_("sensor", "health.state", "System Status", nullptr, nullptr, nullptr, "mdi:heart-pulse");
    publishDiscovery_("sensor", "esp_temp", "Chip Temperatur", "°C", "temperature", "measurement", "mdi:cpu-64-bit");
    publishDiscovery_("sensor", "rssi", "WLAN Signal", "dBm", "signal_strength", "measurement");
    publishDiscovery_("sensor", "uptime_s", "Laufzeit", "s", "duration", "total_increasing");
    publishDiscovery_("sensor", "ip", "IP Adresse", nullptr, nullptr, nullptr, "mdi:ip-network");
    publishDiscovery_("sensor", "ssid", "WLAN SSID", nullptr, nullptr, nullptr, "mdi:wifi");
    publishDiscovery_("binary_sensor", "wifi_connected", "WLAN verbunden", nullptr, "connectivity", nullptr);
    publishDiscovery_("binary_sensor", "internet_ok", "Internet OK", nullptr, "connectivity", nullptr);
    publishDiscovery_("binary_sensor", "update.has_update", "Update verfügbar", nullptr, "update", nullptr);
}
