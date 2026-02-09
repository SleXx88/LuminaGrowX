#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

struct MqttConfig {
    bool enabled = false;
    String server = "";
    uint16_t port = 1883;
    String user = "";
    String pass = "";
};

class MqttCtrl {
public:
    MqttCtrl();

    void begin(const MqttConfig& config, const String& deviceName);
    void loop();
    void setConfig(const MqttConfig& config);
    bool isConnected();
    
    // Updates device name if changed at runtime
    void setDeviceName(const String& name);

    // Publish current state (called periodically from main)
    // Takes a JSON object with all sensor data
    void publishState(const JsonObject& state);
    void publishState(const String& payload);

    // Send HA Auto Discovery Config
    void sendDiscovery();

private:
    MqttConfig config_;
    WiFiClient wifiClient_;
    PubSubClient client_;
    String deviceName_;
    String deviceId_; // derived from MAC

    uint32_t lastReconnectAttempt_ = 0;
    bool discoverySent_ = false;

    void reconnect_();
    String getBaseTopic_();
    void publishDiscovery_(const char* component, const char* objectId, const char* name, const char* unit, const char* devClass, const char* stateClass = nullptr, const char* icon = nullptr);
    void publishSwitchDiscovery_(const char* objectId, const char* name, const char* icon);
};
