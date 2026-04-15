#include "SettingsWebServer.h"

#include <Arduino.h>
#include <WebServer.h>

#include "DebugLog.h"
#include "WebDebug.h"
#include "WiFiSettings.h"

static WebServer webServer(80);
static bool webServerStarted = false;

static String htmlEscape(const String &value) {
  String escaped;
  escaped.reserve(value.length());
  for (size_t i = 0; i < value.length(); i++) {
    char c = value.charAt(i);
    switch (c) {
      case '&': escaped += F("&amp;"); break;
      case '<': escaped += F("&lt;"); break;
      case '>': escaped += F("&gt;"); break;
      case '"': escaped += F("&quot;"); break;
      case '\'': escaped += F("&#39;"); break;
      default: escaped += c; break;
    }
  }
  return escaped;
}

static String pageStart(const String &title) {
  String page;
  page.reserve(1600);
  page += F("<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">");
  page += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  page += F("<title>");
  page += htmlEscape(title);
  page += F("</title>");
  page += F("<style>");
  page += F("body{font-family:Arial,sans-serif;margin:0;background:#f6f8f9;color:#172026}");
  page += F("main{max-width:760px;margin:0 auto;padding:24px}");
  page += F("h1{font-size:28px;margin:0 0 12px}");
  page += F("p{line-height:1.45}");
  page += F("a.button,button{display:inline-block;font-size:16px;padding:11px 16px;border:0;border-radius:6px;background:#0b6b58;color:#fff;text-decoration:none;cursor:pointer}");
  page += F("button.secondary,a.secondary{background:#5d6870}");
  page += F("label{display:block;font-weight:700;margin:18px 0 6px}");
  page += F("input{box-sizing:border-box;width:100%;font-size:18px;padding:12px;border:1px solid #9aa4aa;border-radius:6px;background:#fff}");
  page += F("pre{white-space:pre-wrap;word-break:break-word;background:#172026;color:#f6f8f9;padding:14px;border-radius:6px;line-height:1.45}");
  page += F("h2{font-size:20px;margin:28px 0 8px}");
  page += F(".actions{display:flex;gap:12px;flex-wrap:wrap;margin-top:20px}");
  page += F(".notice{padding:12px;border-left:4px solid #0b6b58;background:#e9f5f2;margin:18px 0}");
  page += F(".status{padding:12px;border-left:4px solid #2f6f9f;background:#eef6fb;margin:18px 0}");
  page += F(".hint{font-size:14px;color:#4b565c}");
  page += F(".stream-status{font-size:14px;color:#4b565c;margin:0 0 8px}");
  page += F("</style></head><body><main>");
  return page;
}

static void appendPageEnd(String &page) {
  page += F("</main></body></html>");
}

static String homePage() {
  String page = pageStart(F("MSCanBus"));
  page += F("<h1>MSCanBus</h1>");
  page += F("<div class=\"status\">");
  page += htmlEscape(accessPointStatusText());
  page += F("</div>");
  page += F("<div class=\"actions\">");
  page += F("<a class=\"button\" href=\"/wifi\">Change WiFi Settings</a>");
  page += F("<a class=\"button secondary\" href=\"/debug\">Logs / Debug</a>");
  page += F("</div>");
  appendPageEnd(page);
  return page;
}

static String wifiSettingsPage(const String &notice) {
  String page = pageStart(F("MSCanBus Access Point"));
  page += F("<h1>MSCanBus Access Point</h1>");
  if (notice.length() > 0) {
    page += F("<div class=\"notice\">");
    page += htmlEscape(notice);
    page += F("</div>");
  }
  page += F("<div class=\"status\">");
  page += htmlEscape(accessPointStatusText());
  page += F("</div>");
  page += F("<h2>Current Configuration</h2>");
  page += F("<pre>");
  page += htmlEscape(accessPointDebugText());
  page += F("</pre>");
  page += F("<form method=\"post\" action=\"/wifi\">");
  page += F("<label for=\"ssid\">Access point name</label>");
  page += F("<input id=\"ssid\" name=\"ssid\" maxlength=\"32\" required value=\"");
  page += htmlEscape(accessPointSsid());
  page += F("\">");
  page += F("<label for=\"password\">Access point password</label>");
  page += F("<input id=\"password\" name=\"password\" type=\"password\" maxlength=\"63\" placeholder=\"Blank creates an open access point\">");
  page += F("<p class=\"hint\">Enter the full password every time you save. Use 8-63 printable ASCII characters, or leave blank for an open access point.</p>");
  page += F("<div class=\"actions\"><button type=\"submit\">Save Access Point</button></div>");
  page += F("</form>");
  page += F("<form method=\"post\" action=\"/wifi/forget\">");
  page += F("<div class=\"actions\"><button class=\"secondary\" type=\"submit\">Restore Default Access Point</button></div>");
  page += F("</form>");
  page += F("<div class=\"actions\"><a class=\"button secondary\" href=\"/\">Home</a><a class=\"button secondary\" href=\"/debug\">Logs / Debug</a></div>");
  appendPageEnd(page);
  return page;
}

static void appendDebugPanel(String &page, const __FlashStringHelper *title, const char *id) {
  page += F("<section><h2>");
  page += title;
  page += F("</h2><p class=\"stream-status\" id=\"status-");
  page += id;
  page += F("\">Connecting...</p><pre id=\"debug-");
  page += id;
  page += F("\">Waiting for data...</pre></section>");
}

static void appendDebugLink(String &page, const __FlashStringHelper *title, const char *path) {
  page += F("<a class=\"button secondary\" href=\"");
  page += path;
  page += F("\">");
  page += title;
  page += F("</a>");
}

static const __FlashStringHelper *debugStateText(DebugLogModule module) {
  return isDebugLogEnabled(module) ? F("enabled") : F("disabled");
}

static void appendDebugState(String &page, const __FlashStringHelper *title, DebugLogModule module) {
  page += F("<p>");
  page += title;
  page += F(": ");
  page += debugStateText(module);
  page += F("</p>");
}

static String debugPage() {
  String page = pageStart(F("MSCanBus Debug"));
  page += F("<h1>Logs / Debug</h1>");
  page += F("<p class=\"hint\">Choose one live debug stream. Serial logs remain available on the USB monitor.</p>");
  page += F("<div class=\"actions\">");
  appendDebugLink(page, F("WiFi"), "/debug/wifi");
  appendDebugLink(page, F("System"), "/debug/system");
  appendDebugLink(page, F("Startup"), "/debug/startup");
  appendDebugLink(page, F("GPS"), "/debug/gps");
  appendDebugLink(page, F("Accelerometer / IMU"), "/debug/imu");
  appendDebugLink(page, F("Temperature / EGT"), "/debug/egt");
  appendDebugLink(page, F("CAN"), "/debug/can");
  page += F("</div>");
  page += F("<div class=\"status\">");
  appendDebugState(page, F("WiFi"), DEBUG_LOG_WIFI);
  appendDebugState(page, F("System"), DEBUG_LOG_SYSTEM);
  appendDebugState(page, F("Startup"), DEBUG_LOG_STARTUP);
  appendDebugState(page, F("GPS"), DEBUG_LOG_GPS);
  appendDebugState(page, F("UBlox"), DEBUG_LOG_UBLOX);
  appendDebugState(page, F("IMU"), DEBUG_LOG_IMU);
  appendDebugState(page, F("EGT"), DEBUG_LOG_EGT);
  appendDebugState(page, F("CAN"), DEBUG_LOG_CAN);
  page += F("</div>");
  page += F("<div class=\"actions\"><a class=\"button secondary\" href=\"/\">Home</a><a class=\"button\" href=\"/wifi\">Change WiFi Settings</a></div>");
  appendPageEnd(page);
  return page;
}

static String debugModulePage(const __FlashStringHelper *title, const char *id, DebugLogModule module) {
  String page = pageStart(F("MSCanBus Debug"));
  page += F("<h1>");
  page += title;
  page += F("</h1>");
  page += F("<p class=\"hint\">Live debug stream for this module only.</p>");
  page += F("<div class=\"status\">Debug is ");
  page += debugStateText(module);
  page += F(".</div>");
  page += F("<div class=\"actions\"><form method=\"post\" action=\"/debug/");
  page += id;
  page += F("/enable\"><button type=\"submit\">Enable Debug</button></form>");
  page += F("<form method=\"post\" action=\"/debug/");
  page += id;
  page += F("/disable\"><button class=\"secondary\" type=\"submit\">Disable Debug</button></form></div>");
  appendDebugPanel(page, title, id);
  page += F("<div class=\"actions\"><a class=\"button secondary\" href=\"/debug\">All Debug Pages</a><a class=\"button secondary\" href=\"/\">Home</a></div>");
  page += F("<script>");
  page += F("const id='");
  page += id;
  page += F("';");
  page += F("const url='/debug/stream/");
  page += id;
  page += F("';");
  page += F("const out=document.getElementById('debug-'+id);");
  page += F("const statusEl=document.getElementById('status-'+id);");
  page += F("const events=new EventSource(url);");
  page += F("events.onopen=()=>{statusEl.textContent='Connected';};");
  page += F("events.onmessage=(event)=>{out.textContent=event.data;out.dataset.seen='1';statusEl.textContent='Updated '+new Date().toLocaleTimeString();};");
  page += F("events.onerror=()=>{statusEl.textContent=out.dataset.seen?'Waiting for next update...':'Connecting...';};");
  page += F("</script>");
  appendPageEnd(page);
  return page;
}

static void appendDebugToggleForms(String &page, const char *id, const __FlashStringHelper *label, DebugLogModule module, const char *enablePath, const char *disablePath) {
  page += F("<div class=\"status\">");
  page += label;
  page += F(" is ");
  page += debugStateText(module);
  page += F(".</div>");
  page += F("<div class=\"actions\"><form method=\"post\" action=\"");
  page += enablePath;
  page += F("\"><button type=\"submit\">Enable ");
  page += label;
  page += F("</button></form>");
  page += F("<form method=\"post\" action=\"");
  page += disablePath;
  page += F("\"><button class=\"secondary\" type=\"submit\">Disable ");
  page += label;
  page += F("</button></form></div>");
}

static String gpsDebugModulePage() {
  String page = pageStart(F("MSCanBus GPS Debug"));
  page += F("<h1>GPS</h1>");
  page += F("<p class=\"hint\">Regular GPS debug shows parsed GPS state. UBlox driver debug shows the driver trace from qqqlab_GPS_UBLOX.cpp.</p>");
  appendDebugToggleForms(page, "gps", F("Regular GPS Debug"), DEBUG_LOG_GPS, "/debug/gps/enable", "/debug/gps/disable");
  appendDebugToggleForms(page, "gps", F("UBlox Driver Debug"), DEBUG_LOG_UBLOX, "/debug/gps/ublox/enable", "/debug/gps/ublox/disable");
  appendDebugPanel(page, F("GPS"), "gps");
  page += F("<div class=\"actions\"><a class=\"button secondary\" href=\"/debug\">All Debug Pages</a><a class=\"button secondary\" href=\"/\">Home</a></div>");
  page += F("<script>");
  page += F("const id='gps';");
  page += F("const url='/debug/stream/gps';");
  page += F("const out=document.getElementById('debug-'+id);");
  page += F("const statusEl=document.getElementById('status-'+id);");
  page += F("const events=new EventSource(url);");
  page += F("events.onopen=()=>{statusEl.textContent='Connected';};");
  page += F("events.onmessage=(event)=>{out.textContent=event.data;out.dataset.seen='1';statusEl.textContent='Updated '+new Date().toLocaleTimeString();};");
  page += F("events.onerror=()=>{statusEl.textContent=out.dataset.seen?'Waiting for next update...':'Connecting...';};");
  page += F("</script>");
  appendPageEnd(page);
  return page;
}

static void sendWiFiSettingsPage(const String &notice = "") {
  webServer.send(200, "text/html", wifiSettingsPage(notice));
}

static void handleWiFiRoot() {
  webServer.send(200, "text/html", homePage());
}

static void handleWiFiSettings() {
  sendWiFiSettingsPage();
}

static void handleDebug() {
  webServer.send(200, "text/html", debugPage());
}

static void handleDebugModule(const __FlashStringHelper *title, const char *id, DebugLogModule module) {
  webServer.send(200, "text/html", debugModulePage(title, id, module));
}

static void handleGpsDebugModule() {
  webServer.send(200, "text/html", gpsDebugModulePage());
}

static void redirectToDebugModule(const char *id) {
  String location = F("/debug/");
  location += id;
  webServer.sendHeader("Location", location, true);
  webServer.send(303, "text/plain", "");
}

static void handleDebugToggle(DebugLogModule module, bool enabled, const char *id) {
  setDebugLogEnabled(module, enabled);
  redirectToDebugModule(id);
}

static String debugStreamPayload() {
  String payload;
  payload.reserve(5200);
  payload += F("[WiFi / Access Point]\n");
  payload += accessPointDebugText();
  payload += F("\n\n[Startup / System / GPS / IMU / EGT / CAN]\n");
  payload += webAllDebugText();
  return payload;
}

static String moduleDebugStreamPayload(const String &module) {
  if (module == F("wifi")) {
    if (!isDebugLogEnabled(DEBUG_LOG_WIFI)) return F("WiFi debug disabled\n");
    return accessPointDebugText();
  }
  if (module == F("system")) return webSystemStatusDebugText();
  if (module == F("startup")) return webStartupDebugText();
  if (module == F("gps")) return webGpsDebugText();
  if (module == F("imu")) return webImuDebugText();
  if (module == F("egt")) return webEgtDebugText();
  if (module == F("can")) return webCanDebugText();
  return debugStreamPayload();
}

static String sseMessage(const String &payload) {
  String event;
  event.reserve(payload.length() + 128);
  event += F("retry: 1000\n");
  event += F("id: ");
  event += millis();
  event += '\n';
  event += F("event: message\n");
  event += F("data: stream_event_ms=");
  event += millis();
  event += '\n';

  int start = 0;
  while (start < payload.length()) {
    int end = payload.indexOf('\n', start);
    if (end < 0) end = payload.length();
    event += F("data: ");
    event += payload.substring(start, end);
    event += '\n';
    start = end + 1;
  }

  event += '\n';
  return event;
}

static void handleDebugStream() {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.sendHeader("Connection", "close");
  webServer.send(200, "text/event-stream", sseMessage(debugStreamPayload()));
}

static void handleModuleDebugStream(const String &module) {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.sendHeader("Connection", "close");
  webServer.send(200, "text/event-stream", sseMessage(moduleDebugStreamPayload(module)));
}

static void handleWiFiSave() {
  if (!webServer.hasArg("ssid")) {
    webServer.send(400, "text/plain", "Missing ssid");
    return;
  }

  String newSsid = webServer.arg("ssid");
  String newPassword = webServer.hasArg("password") ? webServer.arg("password") : "";
  newSsid.trim();

  if (!isValidAccessPointSsid(newSsid)) {
    webServer.send(400, "text/plain", "Access point name must be 1-32 printable ASCII characters");
    return;
  }

  const char *passwordError = accessPointPasswordValidationError(newPassword);
  if (passwordError) {
    webServer.send(400, "text/plain", passwordError);
    return;
  }

  requestAccessPointSettings(newSsid, newPassword);
  sendWiFiSettingsPage(F("Access point settings accepted. Reconnect to the new access point after it restarts."));
}

static void handleWiFiForget() {
  requestDefaultAccessPointSettings();
  sendWiFiSettingsPage(F("Default access point settings will be restored."));
}

void setupSettingsWebServer() {
  if (webServerStarted) return;

  webServer.on("/", HTTP_GET, handleWiFiRoot);
  webServer.on("/wifi", HTTP_GET, handleWiFiSettings);
  webServer.on("/wifi", HTTP_POST, handleWiFiSave);
  webServer.on("/debug", HTTP_GET, handleDebug);
  webServer.on("/debug/wifi", HTTP_GET, []() { handleDebugModule(F("WiFi"), "wifi", DEBUG_LOG_WIFI); });
  webServer.on("/debug/system", HTTP_GET, []() { handleDebugModule(F("System"), "system", DEBUG_LOG_SYSTEM); });
  webServer.on("/debug/startup", HTTP_GET, []() { handleDebugModule(F("Startup"), "startup", DEBUG_LOG_STARTUP); });
  webServer.on("/debug/gps", HTTP_GET, handleGpsDebugModule);
  webServer.on("/debug/imu", HTTP_GET, []() { handleDebugModule(F("Accelerometer / IMU"), "imu", DEBUG_LOG_IMU); });
  webServer.on("/debug/egt", HTTP_GET, []() { handleDebugModule(F("Temperature / EGT"), "egt", DEBUG_LOG_EGT); });
  webServer.on("/debug/can", HTTP_GET, []() { handleDebugModule(F("CAN"), "can", DEBUG_LOG_CAN); });
  webServer.on("/debug/wifi/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_WIFI, true, "wifi"); });
  webServer.on("/debug/wifi/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_WIFI, false, "wifi"); });
  webServer.on("/debug/system/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_SYSTEM, true, "system"); });
  webServer.on("/debug/system/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_SYSTEM, false, "system"); });
  webServer.on("/debug/startup/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_STARTUP, true, "startup"); });
  webServer.on("/debug/startup/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_STARTUP, false, "startup"); });
  webServer.on("/debug/gps/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_GPS, true, "gps"); });
  webServer.on("/debug/gps/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_GPS, false, "gps"); });
  webServer.on("/debug/gps/ublox/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_UBLOX, true, "gps"); });
  webServer.on("/debug/gps/ublox/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_UBLOX, false, "gps"); });
  webServer.on("/debug/imu/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_IMU, true, "imu"); });
  webServer.on("/debug/imu/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_IMU, false, "imu"); });
  webServer.on("/debug/egt/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_EGT, true, "egt"); });
  webServer.on("/debug/egt/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_EGT, false, "egt"); });
  webServer.on("/debug/can/enable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_CAN, true, "can"); });
  webServer.on("/debug/can/disable", HTTP_POST, []() { handleDebugToggle(DEBUG_LOG_CAN, false, "can"); });
  webServer.on("/debug/stream", HTTP_GET, handleDebugStream);
  webServer.on("/debug/stream/wifi", HTTP_GET, []() { handleModuleDebugStream(F("wifi")); });
  webServer.on("/debug/stream/system", HTTP_GET, []() { handleModuleDebugStream(F("system")); });
  webServer.on("/debug/stream/startup", HTTP_GET, []() { handleModuleDebugStream(F("startup")); });
  webServer.on("/debug/stream/gps", HTTP_GET, []() { handleModuleDebugStream(F("gps")); });
  webServer.on("/debug/stream/imu", HTTP_GET, []() { handleModuleDebugStream(F("imu")); });
  webServer.on("/debug/stream/egt", HTTP_GET, []() { handleModuleDebugStream(F("egt")); });
  webServer.on("/debug/stream/can", HTTP_GET, []() { handleModuleDebugStream(F("can")); });
  webServer.on("/wifi/forget", HTTP_POST, handleWiFiForget);
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "/", true);
    webServer.send(302, "text/plain", "");
  });
  webServer.begin();
  webServerStarted = true;
  printSerialDebugLine(F("WiFi settings web server started on port 80"));
  appendStartupDebugLogLine(F("WiFi settings web server started on port 80"));
  printSerialDebugLine(F("WiFi debug stream available at /debug/stream"));
  appendStartupDebugLogLine(F("WiFi debug stream available at /debug/stream"));
}

void maintainSettingsWebServer() {
  webServer.handleClient();
}
