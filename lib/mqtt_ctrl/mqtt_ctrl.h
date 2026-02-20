#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <functional>

struct MqttConfig {
    bool enabled = false;
    String server = "";
    uint16_t port = 1883;
    String user = "";
    String pass = "";
};

typedef std::function<void(const String& topic, const String& payload)> MqttCommandCallback;

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

    // Callback for incoming commands
    void onCommand(MqttCommandCallback cb) { commandCb_ = cb; }

private:
    MqttConfig config_;
    WiFiClient wifiClient_;
    PubSubClient client_;
    String deviceName_;
    String deviceId_; // derived from MAC
    MqttCommandCallback commandCb_ = nullptr;

    uint32_t lastReconnectAttempt_ = 0;
    bool discoverySent_ = false;

    void reconnect_();
    String getBaseTopic_();
    void publishDiscovery_(const char* component, const char* objectId, const char* name, const char* unit, const char* devClass, const char* stateClass = nullptr, const char* icon = nullptr, const char* entCat = nullptr, int precision = -1);
    void publishUpdateDiscovery_(const char* objectId, const char* name);
    void publishButtonDiscovery_(const char* objectId, const char* name, const char* icon = nullptr, const char* entCat = nullptr, const char* payload = "press", const char* avtyTpl = nullptr);
    void publishSwitchDiscovery_(const char* objectId, const char* name, const char* icon);
    
    void callback_(char* topic, uint8_t* payload, unsigned int length);
};
