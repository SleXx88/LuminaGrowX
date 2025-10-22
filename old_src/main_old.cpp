#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <time.h>

// Lisa Pro – ESP32-S3
#define FW_VERSION "1.0.0"

static constexpr int LIGHT_PIN = 5;
static constexpr int FAN_PIN = 6;
static constexpr int PUMP_PIN = 7;

struct NetCfg
{
  String ssid;
  String pass;
  bool useStatic = false;
  String ip;
  String mask = "255.255.255.0";
  String gw;
  String dns = "1.1.1.1";
};
struct AppCfg
{
  String seed = "Northern Lights";
};
struct GrowState
{
  bool started = false;
  uint32_t start_epoch = 0;
  uint16_t total_days = 90;
};
struct NotifyCfg
{
  bool enabled = false;
  String phone;
  String apikey;
};

Preferences prefs;
NetCfg net;
AppCfg app;
GrowState grow;
NotifyCfg notifyCfg;
AsyncWebServer http(80);
AsyncWebSocket ws("/ws");

bool lightOn = false, fanOn = false, pumpOn = false;
float tempC = 24.2f, humiRH = 55.4f, vpdKPa = 1.10f;
bool wifi_connected = false, internet_ok = false;
String ntp_string("—");
uint32_t nextNetProbe = 0;
const uint32_t NET_PROBE_MS = 60000;
uint32_t nextPushAt = 0;
const uint32_t PUSH_INTERVAL_MS = 2000;
uint32_t rebootAt = 0;
String apSSID;
IPAddress apIP(192, 168, 4, 1);

static String urlEncode(const String &s)
{
  String out;
  char hex[4];
  for (size_t i = 0; i < s.length(); ++i)
  {
    char c = s[i];
    if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || ('0' <= c && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~')
      out += c;
    else
    {
      snprintf(hex, sizeof(hex), "%%%02X", (unsigned char)c);
      out += hex;
    }
  }
  return out;
}

bool parseIPv4(const String &s, IPAddress &out) { return out.fromString(s); }
uint64_t uptime_s() { return millis() / 1000ULL; }
void tzInit()
{
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
}
String fmtDateTime(time_t t)
{
  if (t == 0)
    return String("—");
  struct tm tm;
  localtime_r(&t, &tm);
  char buf[32];
  strftime(buf, sizeof(buf), "%d.%m.%Y %H:%M:%S", &tm);
  return String(buf);
}
void ntpInit() { configTime(0, 0, "pool.ntp.org", "time.nist.gov"); }
bool timeIsSet()
{
  time_t now;
  time(&now);
  return now > 1700000000;
}
String currentTimeString()
{
  time_t now;
  time(&now);
  return fmtDateTime(now);
}

void loadConfig()
{
  if (!prefs.begin("cfg", false))
  {
    Serial.println("[NVS] prefs.begin failed");
    return;
  }
  net.ssid = prefs.getString("ssid", "");
  net.pass = prefs.getString("pass", "");
  net.useStatic = prefs.getBool("useStatic", false);
  net.ip = prefs.getString("ip", "");
  net.mask = prefs.getString("mask", "255.255.255.0");
  net.gw = prefs.getString("gw", "");
  net.dns = prefs.getString("dns", "1.1.1.1");
  app.seed = prefs.getString("seed", app.seed);
  grow.started = prefs.getBool("growStarted", false);
  grow.start_epoch = prefs.getUInt("growStart", 0);
  grow.total_days = prefs.getUShort("growTotal", 90);
  notifyCfg.enabled = prefs.getBool("ntf_en", false);
  notifyCfg.phone = prefs.getString("ntf_phone", "");
  notifyCfg.apikey = prefs.getString("ntf_key", "");
  prefs.end();
}

void saveNetworkConfig(const NetCfg &n)
{
  if (!prefs.begin("cfg", false))
    return;
  prefs.putString("ssid", n.ssid);
  prefs.putString("pass", n.pass);
  prefs.putBool("useStatic", n.useStatic);
  prefs.putString("ip", n.ip);
  prefs.putString("mask", n.mask);
  prefs.putString("gw", n.gw);
  prefs.putString("dns", n.dns);
  prefs.end();
}
void saveAppConfig(const AppCfg &a)
{
  if (!prefs.begin("cfg", false))
    return;
  prefs.putString("seed", a.seed);
  prefs.end();
}
void saveGrowConfig(const GrowState &g)
{
  if (!prefs.begin("cfg", false))
    return;
  prefs.putBool("growStarted", g.started);
  prefs.putUInt("growStart", g.start_epoch);
  prefs.putUShort("growTotal", g.total_days);
  prefs.end();
}
void saveNotifyConfig(const NotifyCfg &c)
{
  if (!prefs.begin("cfg", false))
    return;
  prefs.putBool("ntf_en", c.enabled);
  prefs.putString("ntf_phone", c.phone);
  prefs.putString("ntf_key", c.apikey);
  prefs.end();
}

void startAP()
{
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac = mac.substring(mac.length() - 4);
  apSSID = String("LisaPro-Setup-") + mac;
  WiFi.softAP(apSSID.c_str());
  apIP = WiFi.softAPIP();
  Serial.printf("AP gestartet: %s @ %s\n", apSSID.c_str(), apIP.toString().c_str());
}
void connectWiFi()
{
  WiFi.mode(WIFI_AP_STA);
  startAP();
  if (net.useStatic)
  {
    IPAddress lip, lgw, lms, ldn;
    parseIPv4(net.ip, lip);
    parseIPv4(net.gw, lgw);
    parseIPv4(net.mask, lms);
    parseIPv4(net.dns, ldn);
    if (lip && lgw && lms)
      WiFi.config(lip, lgw, lms, ldn);
  }
  if (net.ssid.length())
    WiFi.begin(net.ssid.c_str(), net.pass.c_str());
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000)
  {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  wifi_connected = WiFi.isConnected();
  if (wifi_connected)
  {
    Serial.printf("WiFi OK: %s RSSI=%d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (MDNS.begin("lisapro"))
    {
      MDNS.addService("http", "tcp", 80);
      Serial.println("mDNS: http://lisapro.local/");
    }
  }
  else
  {
    Serial.println("WiFi nicht verbunden – AP-Setup aktiv (http://" + apIP.toString() + ")");
  }
}

bool probeInternet()
{
  if (!wifi_connected)
    return false;
  HTTPClient httpc;
  bool ok = false;
  if (httpc.begin("http://clients3.google.com/generate_204"))
  {
    int code = httpc.GET();
    ok = (code == 204 || code == 200);
    httpc.end();
  }
  return ok;
}

uint16_t growDay()
{
  if (!grow.started || grow.start_epoch == 0)
    return 0;
  time_t now;
  time(&now);
  if (now < grow.start_epoch)
    return 0;
  uint32_t days = (uint32_t)((now - grow.start_epoch) / 86400UL) + 1;
  if (days > grow.total_days)
    days = grow.total_days;
  return (uint16_t)days;
}
String growPhase(uint16_t day)
{
  if (day == 0)
    return "—";
  if (day <= 14)
    return "Keimung/Seedling";
  if (day <= 35)
    return "Vegetationsphase";
  return "Blütephase";
}

// WhatsApp: CallMeBot
bool whatsappSend(const String &phone, const String &apikey, const String &message)
{
  if (!wifi_connected)
    return false;
  HTTPClient http;
  String url = String("https://api.callmebot.com/whatsapp.php?phone=") + urlEncode(phone) + "&text=" + urlEncode(message) + "&apikey=" + urlEncode(apikey);
  bool ok = false;
  if (http.begin(url))
  {
    int code = http.GET();
    ok = (code == 200);
    http.end();
  }
  return ok;
}

String makeStatusJson()
{
  JsonDocument doc;
  doc["ip"] = wifi_connected ? WiFi.localIP().toString() : String("0.0.0.0");
  doc["use_static"] = net.useStatic;
  doc["wifi_connected"] = wifi_connected;
  doc["internet_ok"] = internet_ok;
  doc["rssi"] = wifi_connected ? WiFi.RSSI() : -127;
  doc["uptime_s"] = uptime_s();
  doc["ntp_time"] = ntp_string;
  doc["temp_c"] = roundf(tempC * 10.f) / 10.f;
  doc["humi_rh"] = roundf(humiRH * 10.f) / 10.f;
  doc["vpd_kpa"] = roundf(vpdKPa * 10.f) / 10.f;
  doc["light_on"] = lightOn;
  doc["fan_on"] = fanOn;
  doc["pump_on"] = pumpOn;
  JsonObject health = doc["health"].to<JsonObject>();
  health["state"] = "ok";
  health["message"] = "";
  JsonObject g = doc["grow"].to<JsonObject>();
  g["started"] = grow.started;
  g["start_epoch"] = grow.start_epoch;
  g["total_days"] = grow.total_days;
  uint16_t day = growDay();
  g["day"] = day;
  g["phase"] = growPhase(day);
  JsonObject ntf = doc["notify"].to<JsonObject>();
  ntf["enabled"] = notifyCfg.enabled;
  ntf["phone"] = notifyCfg.phone;
  ntf["apikey"] = notifyCfg.apikey.length() ? "***" : ""; // API-Key nicht im Klartext
  JsonArray addons = doc["addons"].to<JsonArray>();
  JsonObject ap = doc["ap"].to<JsonObject>();
  ap["ssid"] = apSSID;
  ap["ip"] = WiFi.softAPIP().toString();
  doc["seed"] = app.seed;
  doc["fw"] = FW_VERSION;
  doc["ts"] = (uint64_t)millis();
  String out;
  serializeJson(doc, out);
  return out;
}

String makeInfoJson()
{
  JsonDocument doc;
  doc["seed"] = app.seed;
  JsonObject netw = doc["network"].to<JsonObject>();
  netw["ssid"] = net.ssid;
  netw["use_static"] = net.useStatic;
  JsonObject st = netw["static"].to<JsonObject>();
  st["ip"] = net.ip;
  st["mask"] = net.mask;
  st["gw"] = net.gw;
  st["dns"] = net.dns;
  JsonObject ntf = doc["notify"].to<JsonObject>();
  ntf["enabled"] = notifyCfg.enabled;
  ntf["phone"] = notifyCfg.phone;
  ntf["apikey"] = notifyCfg.apikey;
  String out;
  serializeJson(doc, out);
  return out;
}
void broadcastStatus()
{
  if (ws.count() > 0)
    ws.textAll(makeStatusJson());
}

void sendFile(AsyncWebServerRequest *req, const char *path, const char *mime)
{
  if (LittleFS.exists(path))
    req->send(LittleFS, path, mime);
  else
    req->send(404, "text/plain", "Not found");
}
void onApiInfo(AsyncWebServerRequest *req) { req->send(200, "application/json", makeInfoJson()); }
void onApiStatus(AsyncWebServerRequest *req) { req->send(200, "application/json", makeStatusJson()); }
void onApiSeed(AsyncWebServerRequest *req, uint8_t *data, size_t len)
{
  JsonDocument doc;
  if (deserializeJson(doc, data, len))
  {
    req->send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }
  if (!doc["seed"].isNull())
    app.seed = String((const char *)doc["seed"]);
  saveAppConfig(app);
  req->send(200, "application/json", "{\"ok\":true}");
  broadcastStatus();
}
void onApiNetwork(AsyncWebServerRequest *req, uint8_t *data, size_t len)
{
  JsonDocument doc;
  if (deserializeJson(doc, data, len))
  {
    req->send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }
  NetCfg n = net;
  if (!doc["ssid"].isNull())
    n.ssid = String((const char *)doc["ssid"]);
  if (!doc["pass"].isNull())
    n.pass = String((const char *)doc["pass"]);
  if (!doc["use_static"].isNull())
    n.useStatic = doc["use_static"].as<bool>();
  if (!doc["static"].isNull())
  {
    JsonObject st = doc["static"].as<JsonObject>();
    if (!st["ip"].isNull())
      n.ip = String((const char *)st["ip"]);
    if (!st["mask"].isNull())
      n.mask = String((const char *)st["mask"]);
    if (!st["gw"].isNull())
      n.gw = String((const char *)st["gw"]);
    if (!st["dns"].isNull())
      n.dns = String((const char *)st["dns"]);
  }
  if (n.useStatic)
  {
    IPAddress ip, mask, gw, dns;
    if (!parseIPv4(n.ip, ip) || !parseIPv4(n.mask, mask) || !parseIPv4(n.gw, gw))
    {
      req->send(400, "application/json", "{\"error\":\"invalid static ip configuration\"}");
      return;
    }
    if (n.dns.length() && !parseIPv4(n.dns, dns))
    {
      req->send(400, "application/json", "{\"error\":\"invalid dns\"}");
      return;
    }
  }
  // Notifications
  NotifyCfg nc = notifyCfg;
  if (!doc["notify"].isNull())
  {
    JsonObject ntf = doc["notify"].as<JsonObject>();
    if (!ntf["enabled"].isNull())
      nc.enabled = ntf["enabled"].as<bool>();
    if (!ntf["phone"].isNull())
      nc.phone = String((const char *)ntf["phone"]);
    if (!ntf["apikey"].isNull())
      nc.apikey = String((const char *)ntf["apikey"]);
  }
  net = n;
  notifyCfg = nc;
  saveNetworkConfig(net);
  saveNotifyConfig(notifyCfg);
  bool reboot = doc["reboot"].as<bool>();
  req->send(200, "application/json", "{\"ok\":true}");
  if (reboot)
  {
    rebootAt = millis() + 800;
  }
}

void onApiGrow(AsyncWebServerRequest *req, uint8_t *data, size_t len)
{
  JsonDocument doc;
  if (deserializeJson(doc, data, len))
  {
    req->send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }
  String action = String((const char *)doc["action"]);
  time_t now;
  time(&now);
  if (action == "start")
  {
    grow.started = true;
    grow.start_epoch = (uint32_t)now;
    if (!doc["total_days"].isNull())
      grow.total_days = (uint16_t)doc["total_days"].as<uint16_t>();
    saveGrowConfig(grow);
    req->send(200, "application/json", "{\"ok\":true}");
    broadcastStatus();
  }
  else if (action == "stop")
  {
    grow.started = false;
    grow.start_epoch = 0;
    saveGrowConfig(grow);
    req->send(200, "application/json", "{\"ok\":true}");
    broadcastStatus();
  }
  else
  {
    req->send(400, "application/json", "{\"error\":\"unknown action\"}");
  }
}

void onApiNotifyTest(AsyncWebServerRequest *req, uint8_t *data, size_t len)
{
  JsonDocument doc;
  if (deserializeJson(doc, data, len))
  {
    req->send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }
  String phone = doc["phone"].isNull() ? notifyCfg.phone : String((const char *)doc["phone"]);
  String key = doc["apikey"].isNull() ? notifyCfg.apikey : String((const char *)doc["apikey"]);
  String msg = String("Lisa Pro: Test – ") + fmtDateTime(time(nullptr)) + " | IP " + (wifi_connected ? WiFi.localIP().toString() : String("0.0.0.0"));
  bool ok = whatsappSend(phone, key, msg);
  if (ok)
    req->send(200, "application/json", "{\"ok\":true}");
  else
    req->send(500, "application/json", "{\"error\":\"send failed\"}");
}

void onWsEvent(AsyncWebSocket *wss, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    client->text(makeStatusJson());
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);
  if (!LittleFS.begin())
    Serial.println("LittleFS mount failed");
  tzInit();
  loadConfig();
  connectWiFi();
  ntpInit();

  http.on("/", HTTP_GET, [](AsyncWebServerRequest *req)
          { sendFile(req, "/index.html", "text/html"); });
  http.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *req)
          { sendFile(req, "/index.html", "text/html"); });
  http.on("/pico.min.css", HTTP_GET, [](AsyncWebServerRequest *req)
          { sendFile(req, "/pico.min.css", "text/css"); });
  http.on("/bg.jpg", HTTP_GET, [](AsyncWebServerRequest *req)
          { sendFile(req, "/bg.jpg", "image/jpeg"); });

  http.on("/api/info", HTTP_GET, onApiInfo);
  http.on("/api/status", HTTP_GET, onApiStatus);
  http.on("/api/seed", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL, [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total)
          { onApiSeed(req, data, len); });
  http.on("/api/network", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL, [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total)
          { onApiNetwork(req, data, len); });
  http.on("/api/grow", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL, [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total)
          { onApiGrow(req, data, len); });
  http.on("/api/notify/test", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL, [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total)
          { onApiNotifyTest(req, data, len); });

  ws.onEvent(onWsEvent);
  http.addHandler(&ws);
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
  http.begin();
  Serial.println("HTTP server started");
}

void loop()
{
  const uint32_t nowMs = millis();
  if (timeIsSet())
    ntp_string = currentTimeString();
  else
    ntp_string = "—";
  if ((nowMs % 5000) < 30)
  {
    tempC += 0.03f;
    if (tempC > 26.0f)
      tempC = 24.0f;
    humiRH += 0.05f;
    if (humiRH > 60.0f)
      humiRH = 55.0f;
    vpdKPa = 1.2f - (humiRH - 50.0f) * 0.01f;
  }
  if (nowMs >= nextNetProbe)
  {
    wifi_connected = WiFi.isConnected();
    bool prev = internet_ok;
    internet_ok = probeInternet();
    nextNetProbe = nowMs + NET_PROBE_MS;
    if (prev != internet_ok)
      broadcastStatus();
  }
  if (nowMs >= nextPushAt)
  {
    broadcastStatus();
    nextPushAt = nowMs + PUSH_INTERVAL_MS;
  }
  if (rebootAt && nowMs >= rebootAt)
  {
    Serial.println("Reboot…");
    delay(100);
    ESP.restart();
  }
}