#include "web/WebConfigServer.h"

#include <Arduino.h>
#include <Ethernet.h>
#include <stdio.h>
#include <string.h>

#include "Log.h"
#include "debug/DebugCounters.h"
#include "gpi/GpiManager.h"
#include "network/NetworkConfig.h"

namespace dbcan {
namespace r70v2 {

namespace {

static const uint16_t kWebPort = 80;
static const size_t kMaxRequestBytes = 1536;
static const uint32_t kRestartDelayMs = 200;

EthernetServer gServer(kWebPort);
Stream* gLog = 0;
DebugCounters* gCounters = 0;
bool gStarted = false;
bool gRestartPending = false;
uint32_t gRestartAtMs = 0;

void countRequest() {
  if (gCounters != 0) {
    ++gCounters->web_requests_total;
  }
}

void countError() {
  if (gCounters != 0) {
    ++gCounters->web_error_count;
  }
}

void countSave() {
  if (gCounters != 0) {
    ++gCounters->web_save_network_count;
  }
}

void countRouteBackupRecall() {
  if (gCounters != 0) {
    ++gCounters->web_route_backup_recall_count;
  }
}

void countRouteRevert() {
  if (gCounters != 0) {
    ++gCounters->web_route_revert_count;
  }
}

void countRouteError() {
  if (gCounters != 0) {
    ++gCounters->web_route_error_count;
  }
}

void logLine(const char* text) {
  if (gLog != 0 && text != 0) {
    gLog->println(text);
  }
}

void sendHeader(EthernetClient& client,
                int code,
                const char* phrase,
                const char* contentType) {
  client.print(F("HTTP/1.1 "));
  client.print(code);
  client.print(' ');
  client.println(phrase);
  client.print(F("Content-Type: "));
  client.println(contentType);
  client.println(F("Connection: close"));
  client.println(F("Cache-Control: no-store"));
  client.println();
}

void sendEscaped(EthernetClient& client, const char* text) {
  if (text == 0) {
    return;
  }
  for (const char* cursor = text; *cursor != '\0'; ++cursor) {
    switch (*cursor) {
      case '&':
        client.print(F("&amp;"));
        break;
      case '<':
        client.print(F("&lt;"));
        break;
      case '>':
        client.print(F("&gt;"));
        break;
      case '"':
        client.print(F("&quot;"));
        break;
      default:
        client.print(*cursor);
        break;
    }
  }
}

void ipv4ToText(const uint8_t ip[4], char* out, size_t outSize) {
  if (out == 0 || outSize == 0) {
    return;
  }
  snprintf(out, outSize, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

void routeAddressToText(const GpiRouteRow& route, char* out, size_t outSize) {
  if (out == 0 || outSize == 0) {
    return;
  }
  snprintf(out,
           outSize,
           "%u.%02u",
           static_cast<unsigned>(route.targetSubnet),
           static_cast<unsigned>(route.targetDevice));
}

bool parseHexNibble(char ch, uint8_t& value) {
  if (ch >= '0' && ch <= '9') {
    value = static_cast<uint8_t>(ch - '0');
    return true;
  }
  if (ch >= 'A' && ch <= 'F') {
    value = static_cast<uint8_t>(ch - 'A' + 10);
    return true;
  }
  if (ch >= 'a' && ch <= 'f') {
    value = static_cast<uint8_t>(ch - 'a' + 10);
    return true;
  }
  return false;
}

bool urlDecode(const char* src, char* dest, size_t destSize) {
  if (src == 0 || dest == 0 || destSize == 0) {
    return false;
  }
  size_t out = 0;
  for (size_t i = 0; src[i] != '\0'; ++i) {
    char ch = src[i];
    if (ch == '+') {
      ch = ' ';
    } else if (ch == '%' && src[i + 1] != '\0' && src[i + 2] != '\0') {
      uint8_t hi = 0;
      uint8_t lo = 0;
      if (!parseHexNibble(src[i + 1], hi) || !parseHexNibble(src[i + 2], lo)) {
        return false;
      }
      ch = static_cast<char>((hi << 4) | lo);
      i += 2;
    }
    if (out + 1 >= destSize) {
      return false;
    }
    dest[out++] = ch;
  }
  dest[out] = '\0';
  return true;
}

bool extractFormValue(const char* body, const char* key, char* value, size_t valueSize) {
  if (body == 0 || key == 0 || value == 0 || valueSize == 0) {
    return false;
  }
  const size_t keyLen = strlen(key);
  const char* cursor = body;
  while (*cursor != '\0') {
    const char* entryStart = cursor;
    const char* amp = strchr(entryStart, '&');
    const char* entryEnd = amp != 0 ? amp : entryStart + strlen(entryStart);
    const char* eq = static_cast<const char*>(memchr(entryStart, '=', entryEnd - entryStart));
    if (eq != 0 && static_cast<size_t>(eq - entryStart) == keyLen &&
        strncmp(entryStart, key, keyLen) == 0) {
      char encoded[128] = {};
      const size_t encodedLen = static_cast<size_t>(entryEnd - eq - 1);
      if (encodedLen >= sizeof(encoded)) {
        return false;
      }
      memcpy(encoded, eq + 1, encodedLen);
      encoded[encodedLen] = '\0';
      return urlDecode(encoded, value, valueSize);
    }
    if (amp == 0) {
      break;
    }
    cursor = amp + 1;
  }
  value[0] = '\0';
  return false;
}

bool hasCheckbox(const char* body, const char* key) {
  char value[8] = {};
  return extractFormValue(body, key, value, sizeof(value));
}

bool parseIpv4Text(const char* text, uint8_t out[4]) {
  if (text == 0 || out == 0) {
    return false;
  }
  unsigned a = 0;
  unsigned b = 0;
  unsigned c = 0;
  unsigned d = 0;
  if (sscanf(text, "%u.%u.%u.%u", &a, &b, &c, &d) != 4 ||
      a > 255 || b > 255 || c > 255 || d > 255) {
    return false;
  }
  out[0] = static_cast<uint8_t>(a);
  out[1] = static_cast<uint8_t>(b);
  out[2] = static_cast<uint8_t>(c);
  out[3] = static_cast<uint8_t>(d);
  return true;
}

bool parseUint8Range(const char* text, uint8_t minValue, uint8_t maxValue, uint8_t& out) {
  if (text == 0 || *text == '\0') {
    return false;
  }
  char* end = 0;
  const unsigned long value = strtoul(text, &end, 10);
  if (end == text || *end != '\0' || value < minValue || value > maxValue) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

bool parseRouteAddressText(const char* text, uint8_t& subnet, uint8_t& device) {
  if (text == 0 || strlen(text) != 4 || text[1] != '.') {
    return false;
  }
  if (text[0] < '0' || text[0] > '7' ||
      text[2] < '0' || text[2] > '9' ||
      text[3] < '0' || text[3] > '9') {
    return false;
  }
  subnet = static_cast<uint8_t>(text[0] - '0');
  device = static_cast<uint8_t>((text[2] - '0') * 10 + (text[3] - '0'));
  return device <= 63;
}

void trimInPlace(char* text) {
  if (text == 0) {
    return;
  }
  size_t start = 0;
  size_t length = strlen(text);
  while (start < length && (text[start] == ' ' || text[start] == '\t')) {
    ++start;
  }
  while (length > start &&
         (text[length - 1] == ' ' || text[length - 1] == '\t' ||
          text[length - 1] == '\r' || text[length - 1] == '\n')) {
    --length;
  }
  if (start > 0) {
    memmove(text, text + start, length - start);
  }
  text[length - start] = '\0';
}

bool isSafeDeviceName(const char* text) {
  if (text == 0 || *text == '\0') {
    return false;
  }
  for (const char* cursor = text; *cursor != '\0'; ++cursor) {
    const unsigned char ch = static_cast<unsigned char>(*cursor);
    if (ch < 32 || ch > 126) {
      return false;
    }
    if (ch == '<' || ch == '>' || ch == '&' || ch == '"' || ch == '\'') {
      return false;
    }
  }
  return true;
}

void printInputRow(EthernetClient& client,
                   const char* label,
                   const char* name,
                   const char* value) {
  client.print(F("<label>"));
  client.print(label);
  client.print(F("<input form=\"cfgform\" name=\""));
  client.print(name);
  client.print(F("\" value=\""));
  sendEscaped(client, value);
  client.println(F("\"></label>"));
}

void sendText(EthernetClient& client, int code, const char* phrase, const char* text) {
  sendHeader(client, code, phrase, "text/plain; charset=utf-8");
  if (text != 0) {
    client.print(text);
  }
}

void renderRouteRow(EthernetClient& client, uint8_t rowIndex, const GpiRouteRow& route) {
  char addr[8] = {};
  char loadText[8] = {};
  char backupText[8] = {};
  routeAddressToText(route, addr, sizeof(addr));
  snprintf(loadText, sizeof(loadText), "%u", static_cast<unsigned>(route.loadPreset));
  snprintf(backupText, sizeof(backupText), "%u", static_cast<unsigned>(route.backupSlot));

  client.print(F("<div class=\"route\">"));
  client.print(F("<span class=\"rn\">R"));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.print(F("</span><label class=\"cb\"><input form=\"cfgform\" type=\"checkbox\" name=\"re"));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.print(F("\" value=\"1\""));
  if (route.enabled) {
    client.print(F(" checked"));
  }
  client.print(F(">On</label><label>Input<select form=\"cfgform\" name=\"rg"));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.print(F("\">"));
  for (uint8_t gpi = 1; gpi <= kGpiPhysicalCount; ++gpi) {
    client.print(F("<option value=\""));
    client.print(static_cast<unsigned>(gpi));
    client.print(F("\""));
    if (route.physicalGpi == gpi) {
      client.print(F(" selected"));
    }
    client.print(F(">GPI"));
    client.print(static_cast<unsigned>(gpi));
    client.print(F("</option>"));
  }
  client.print(F("</select></label><label>Address<input form=\"cfgform\" name=\"ra"));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.print(F("\" value=\""));
  client.print(addr);
  client.print(F("\" size=\"5\"></label><label>Load<input form=\"cfgform\" name=\"rl"));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.print(F("\" value=\""));
  client.print(loadText);
  client.print(F("\" size=\"3\"></label><label>Backup<input form=\"cfgform\" name=\"rb"));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.print(F("\" value=\""));
  client.print(backupText);
  client.print(F("\" size=\"3\"></label><button type=\"button\" onclick=\"routeAction("));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.print(F(",'backup-recall')\">Backup + Recall</button><button type=\"button\" onclick=\"routeAction("));
  client.print(static_cast<unsigned>(rowIndex + 1));
  client.println(F(",'revert')\">Revert to Saved</button></div>"));
}

void sendMainPage(EthernetClient& client) {
  const NetworkSettings& user = NetworkConfig_userSettings();
  const ActiveNetworkConfig& active = NetworkConfig_activeConfig();
  GpiSettings gpiSettings = {};
  GpiManager_copyActiveSettings(gpiSettings);

  char ipText[24] = {};
  char gatewayText[24] = {};
  char subnetText[24] = {};
  ipv4ToText(user.staticIp, ipText, sizeof(ipText));
  ipv4ToText(user.gateway, gatewayText, sizeof(gatewayText));
  ipv4ToText(user.subnet, subnetText, sizeof(subnetText));

  sendHeader(client, 200, "OK", "text/html; charset=utf-8");
  client.println(F("<!doctype html><html><head><meta charset=\"utf-8\"><title>CueSmart-CueCan-Config</title>"
                   "<style>body{font-family:Arial,sans-serif;background:#242424;color:#e6e6e6;margin:16px}"
                   ".box{background:#333;border:1px solid #555;padding:12px;margin:12px 0;max-width:920px}"
                   "h1,h2{margin:0 0 10px 0}label{display:block;margin:8px 0}input,select{box-sizing:border-box;background:#181818;color:#eee;border:1px solid #666;padding:6px}"
                   "button{background:#5d86ff;color:#fff;border:0;padding:8px 12px}small{color:#aaa}"
                   ".route{display:flex;gap:8px;align-items:center;flex-wrap:wrap;border-top:1px solid #4b4b4b;padding:8px 0}"
                   ".route:first-of-type{border-top:0}.route label{margin:0}.route input{width:72px}.route select{width:78px}.rn{width:28px;color:#bbb}.cb{min-width:48px}.actions{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.statusline{min-height:18px;color:#9bd29b}</style>"
                   "<script>async function routeAction(row,kind){const status=document.getElementById('route-status');try{const res=await fetch('/route/'+row+'/'+kind,{method:'POST'});const text=await res.text();status.textContent=(text||'').trim()||('Route '+row+' action done');}catch(e){status.textContent='Route '+row+' request failed';}}"
                   "async function factoryReset(){const status=document.getElementById('route-status');const form=document.getElementById('cfgform');const body=new URLSearchParams(new FormData(form)).toString();try{const res=await fetch('/factory-reset',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body});const text=await res.text();status.textContent=(text||'').trim()||'Factory reset requested';}catch(e){status.textContent='Factory reset request failed';}}</script></head><body>"));
  client.println(F("<h1>CueSmart-CueCan-Config</h1>"));
  client.println(F("<form id=\"cfgform\" method=\"POST\" action=\"/save\"></form>"));
  client.println(F("<section class=\"box\"><h2>Status</h2>"));
  client.print(F("<p>IP: "));
  client.print(NetworkConfig_localIp());
  client.println(F("</p>"));
  client.print(F("<p>Link: "));
  client.print(NetworkConfig_linkStatusName());
  client.println(F("</p>"));
  client.print(F("<p>Mode: "));
  client.print(user.dhcpEnabled ? F("DHCP") : F("Static"));
  client.println(F("</p>"));
  client.print(F("<p>Name: "));
  sendEscaped(client, active.deviceName);
  client.println(F("</p></section>"));

  client.println(F("<section class=\"box\"><h2>Network</h2>"));
  client.print(F("<label><input form=\"cfgform\" type=\"checkbox\" name=\"dhcp\" value=\"1\""));
  if (user.dhcpEnabled) {
    client.print(F(" checked"));
  }
  client.println(F("> DHCP enable</label>"));
  printInputRow(client, "Static IP", "ip", ipText);
  printInputRow(client, "Gateway", "gateway", gatewayText);
  printInputRow(client, "Subnet mask", "subnet", subnetText);
  printInputRow(client, "Device name", "name", user.deviceName);
  client.println(F("</section>"));

  client.println(F("<section class=\"box\"><h2>GPI Preset Routes</h2>"
                   "<p><small>Action buttons use saved values. Press Save + Restart before testing edited values.</small></p>"));
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    renderRouteRow(client, index, gpiSettings.route[index]);
  }
  client.println(F("<div id=\"route-status\" class=\"statusline\"></div></section>"));

  client.println(F("<section class=\"box\"><div class=\"actions\"><button form=\"cfgform\" type=\"submit\">Save + Restart</button>"
                   "<label class=\"cb\"><input form=\"cfgform\" type=\"checkbox\" name=\"confirm_factory_reset\" value=\"1\"> Confirm factory reset</label>"
                   "<button type=\"button\" onclick=\"factoryReset()\">Factory Reset + Restart</button></div>"
                   "<p><small>Network and GPI route changes apply after restart.</small></p></section></body></html>"));
}

bool parseRouteFromForm(const char* body, uint8_t rowIndex, GpiRouteRow& route, const char*& reason) {
  char key[8] = {};
  char value[32] = {};
  snprintf(key, sizeof(key), "re%u", static_cast<unsigned>(rowIndex + 1));
  route.enabled = hasCheckbox(body, key) ? 1 : 0;

  snprintf(key, sizeof(key), "rg%u", static_cast<unsigned>(rowIndex + 1));
  if (!extractFormValue(body, key, value, sizeof(value)) ||
      !parseUint8Range(value, 1, kGpiPhysicalCount, route.physicalGpi)) {
    reason = "invalid_physical_gpi";
    return false;
  }

  uint8_t subnet = 0;
  uint8_t device = 0;
  snprintf(key, sizeof(key), "ra%u", static_cast<unsigned>(rowIndex + 1));
  if (!extractFormValue(body, key, value, sizeof(value)) ||
      !parseRouteAddressText(value, subnet, device)) {
    reason = "invalid_route_address";
    return false;
  }
  route.targetSubnet = subnet;
  route.targetDevice = device;

  snprintf(key, sizeof(key), "rl%u", static_cast<unsigned>(rowIndex + 1));
  if (!extractFormValue(body, key, value, sizeof(value)) ||
      !parseUint8Range(value, 1, 14, route.loadPreset)) {
    reason = "invalid_load_preset";
    return false;
  }

  snprintf(key, sizeof(key), "rb%u", static_cast<unsigned>(rowIndex + 1));
  if (!extractFormValue(body, key, value, sizeof(value)) ||
      !parseUint8Range(value, 1, 14, route.backupSlot)) {
    reason = "invalid_backup_slot";
    return false;
  }

  route.actionMode = GpiActionMode::BackupLoadRestore;
  reason = "ok";
  return true;
}

bool saveAllFromBody(const char* body) {
  NetworkSettings nextNetwork = NetworkConfig_userSettings();
  GpiSettings nextGpi = {};
  if (!GpiManager_copyActiveSettings(nextGpi)) {
    logLine("[WEB SAVE ERROR] reason=gpi_settings_unavailable");
    return false;
  }

  char value[128] = {};
  char name[32] = {};
  char rawIp[32] = {};
  char rawGateway[32] = {};
  char rawSubnet[32] = {};
  char rawName[32] = {};
  nextNetwork.dhcpEnabled = hasCheckbox(body, "dhcp");
  extractFormValue(body, "ip", rawIp, sizeof(rawIp));
  extractFormValue(body, "gateway", rawGateway, sizeof(rawGateway));
  extractFormValue(body, "subnet", rawSubnet, sizeof(rawSubnet));
  extractFormValue(body, "name", rawName, sizeof(rawName));

  if (gLog != 0) {
    char line[224] = {};
    snprintf(line,
             sizeof(line),
             "[WEB SAVE] raw dhcp=%u ip=%s gateway=%s subnet=%s name=%s",
             nextNetwork.dhcpEnabled ? 1u : 0u,
             rawIp,
             rawGateway,
             rawSubnet,
             rawName);
    gLog->println(line);
  }

  if (!extractFormValue(body, "ip", value, sizeof(value)) || !parseIpv4Text(value, nextNetwork.staticIp)) {
    logLine("[WEB SAVE ERROR] reason=invalid_ip");
    return false;
  }
  if (!extractFormValue(body, "gateway", value, sizeof(value)) ||
      !parseIpv4Text(value, nextNetwork.gateway)) {
    logLine("[WEB SAVE ERROR] reason=invalid_gateway");
    return false;
  }
  if (!extractFormValue(body, "subnet", value, sizeof(value)) ||
      !parseIpv4Text(value, nextNetwork.subnet)) {
    logLine("[WEB SAVE ERROR] reason=invalid_subnet");
    return false;
  }
  if (!extractFormValue(body, "name", name, sizeof(name))) {
    logLine("[WEB SAVE ERROR] reason=missing_name");
    return false;
  }
  trimInPlace(name);
  if (!isSafeDeviceName(name)) {
    logLine("[WEB SAVE ERROR] reason=invalid_name");
    return false;
  }
  strncpy(nextNetwork.deviceName, name, sizeof(nextNetwork.deviceName) - 1);
  nextNetwork.deviceName[sizeof(nextNetwork.deviceName) - 1] = '\0';

  for (uint8_t rowIndex = 0; rowIndex < kGpiRouteCount; ++rowIndex) {
    const char* routeReason = 0;
    if (!parseRouteFromForm(body, rowIndex, nextGpi.route[rowIndex], routeReason)) {
      if (gLog != 0) {
        char line[96] = {};
        snprintf(line,
                 sizeof(line),
                 "[WEB SAVE ERROR] reason=%s row=%u",
                 routeReason != 0 ? routeReason : "invalid_route",
                 static_cast<unsigned>(rowIndex + 1));
        gLog->println(line);
      }
      return false;
    }
  }

  if (gLog != 0) {
    char line[224] = {};
    snprintf(line,
             sizeof(line),
             "[WEB SAVE] parsed dhcp=%u ip=%u.%u.%u.%u gateway=%u.%u.%u.%u subnet=%u.%u.%u.%u name=%s",
             nextNetwork.dhcpEnabled ? 1u : 0u,
             nextNetwork.staticIp[0],
             nextNetwork.staticIp[1],
             nextNetwork.staticIp[2],
             nextNetwork.staticIp[3],
             nextNetwork.gateway[0],
             nextNetwork.gateway[1],
             nextNetwork.gateway[2],
             nextNetwork.gateway[3],
             nextNetwork.subnet[0],
             nextNetwork.subnet[1],
             nextNetwork.subnet[2],
             nextNetwork.subnet[3],
             nextNetwork.deviceName);
    gLog->println(line);
    gLog->println("[WEB SAVE] validation=OK");
  }

  const char* reason = 0;
  if (!NetworkConfig_saveUserSettings(nextNetwork, reason)) {
    if (gLog != 0) {
      gLog->print("[WEB SAVE ERROR] reason=");
      gLog->println(reason != 0 ? reason : "network_save_failed");
    }
    return false;
  }
  if (!GpiManager_saveSettingsForRestart(nextGpi, reason)) {
    if (gLog != 0) {
      gLog->print("[WEB SAVE ERROR] reason=");
      gLog->println(reason != 0 ? reason : "gpi_save_failed");
    }
    return false;
  }
  countSave();
  return true;
}

bool factoryResetFromBody(const char* body) {
  const bool confirmed = hasCheckbox(body, "confirm_factory_reset");
  if (gLog != 0) {
    gLog->print("[WEB FACTORY RESET] confirmed=");
    gLog->println(confirmed ? 1 : 0);
  }
  if (!confirmed) {
    logLine("[WEB FACTORY RESET ERROR] reason=confirmation_missing");
    return false;
  }

  const char* reason = 0;
  if (!NetworkConfig_resetUserSettingsToFactoryDefaults(reason)) {
    if (gLog != 0) {
      gLog->print("[WEB FACTORY RESET ERROR] reason=");
      gLog->println(reason != 0 ? reason : "network_reset_failed");
    }
    return false;
  }
  if (!GpiManager_resetFactorySettings(reason)) {
    if (gLog != 0) {
      gLog->print("[WEB FACTORY RESET ERROR] reason=");
      gLog->println(reason != 0 ? reason : "gpi_reset_failed");
    }
    return false;
  }
  return true;
}

bool parseRouteActionPath(const char* path, uint8_t& rowIndex, bool& isBackupRecall, bool& isRevert) {
  rowIndex = 0xFF;
  isBackupRecall = false;
  isRevert = false;
  if (strncmp(path, "/route/", 7) != 0) {
    return false;
  }
  const char* numberStart = path + 7;
  const char* slash = strchr(numberStart, '/');
  if (slash == 0) {
    return false;
  }
  char numberText[8] = {};
  const size_t numberLen = static_cast<size_t>(slash - numberStart);
  if (numberLen == 0 || numberLen >= sizeof(numberText)) {
    return false;
  }
  memcpy(numberText, numberStart, numberLen);
  numberText[numberLen] = '\0';
  char* end = 0;
  const unsigned long value = strtoul(numberText, &end, 10);
  if (end == numberText || *end != '\0' || value < 1 || value > kGpiRouteCount) {
    return false;
  }
  rowIndex = static_cast<uint8_t>(value - 1);
  if (strcmp(slash, "/backup-recall") == 0) {
    isBackupRecall = true;
    return true;
  }
  if (strcmp(slash, "/revert") == 0) {
    isRevert = true;
    return true;
  }
  return false;
}

void closeClient(EthernetClient& client) {
  client.flush();
  client.stop();
}

}  // namespace

void WebConfigServer_begin(Stream& log, DebugCounters* counters) {
  gLog = &log;
  gCounters = counters;
  gServer.begin();
  gStarted = true;
  LOG_PRINTLN(LOG_LEVEL_WARN, log, "[WEB] enabled=1 port=80");
  LOG_PRINTLN(LOG_LEVEL_WARN, log, "[WEB] network_config_page=1");
}

void WebConfigServer_restart() {
  if (!gStarted) {
    return;
  }
  gServer.begin();
  if (gLog != 0) {
    gLog->println(F("[WEB] listener_restarted=1 port=80"));
  }
}

static void (*gFactoryResetIndicator)() = 0;
static bool gFactoryResetRestart = false;

void WebConfigServer_setFactoryResetIndicator(void (*indicator)()) {
  gFactoryResetIndicator = indicator;
}

void WebConfigServer_poll() {
  if (!gStarted) {
    return;
  }
  if (gRestartPending && static_cast<int32_t>(millis() - gRestartAtMs) >= 0) {
    if (gFactoryResetRestart && gFactoryResetIndicator != 0) {
      gFactoryResetIndicator();
    }
    ESP.restart();
  }

  EthernetClient client = gServer.accept();
  if (!client) {
    return;
  }

  char request[kMaxRequestBytes + 1] = {};
  size_t length = 0;
  while (client.available() > 0 && length < kMaxRequestBytes) {
    const int ch = client.read();
    if (ch < 0) {
      break;
    }
    request[length++] = static_cast<char>(ch);
  }
  request[length] = '\0';
  if (length == 0) {
    closeClient(client);
    return;
  }

  const char* headerEnd = strstr(request, "\r\n\r\n");
  if (headerEnd == 0) {
    countError();
    closeClient(client);
    return;
  }

  char method[8] = {};
  char path[40] = {};
  if (sscanf(request, "%7s %39s", method, path) != 2) {
    countError();
    sendText(client, 400, "Bad Request", "bad request");
    closeClient(client);
    return;
  }

  countRequest();
  LOG_BLOCK(LOG_LEVEL_DEBUG, if (gLog != 0) {
    char line[72] = {};
    snprintf(line, sizeof(line), "[WEB] %s %s", method, path);
    gLog->println(line);
  });

  if (strcmp(method, "GET") == 0 && strcmp(path, "/") == 0) {
    sendMainPage(client);
    closeClient(client);
    return;
  }

  if (strcmp(method, "POST") == 0 &&
      (strcmp(path, "/save") == 0 || strcmp(path, "/save-network") == 0)) {
    const char* contentLengthKey = strstr(request, "Content-Length:");
    size_t contentLength = 0;
    if (contentLengthKey != 0) {
      contentLength = static_cast<size_t>(strtoul(contentLengthKey + 15, 0, 10));
    }
    const char* body = headerEnd + 4;
    const size_t bodyLen = length - static_cast<size_t>(body - request);
    if (contentLength > bodyLen) {
      countError();
      sendText(client, 400, "Bad Request", "incomplete request");
      closeClient(client);
      return;
    }
    if (!saveAllFromBody(body)) {
      countError();
      sendText(client, 400, "Bad Request", "invalid settings");
      closeClient(client);
      return;
    }
    sendText(client, 200, "OK", "Saved. Restarting...\n");
    closeClient(client);
    logLine("[WEB] restarting_after_save");
    gRestartPending = true;
    gRestartAtMs = millis() + kRestartDelayMs;
    return;
  }

  if (strcmp(method, "POST") == 0 && strcmp(path, "/factory-reset") == 0) {
    const char* contentLengthKey = strstr(request, "Content-Length:");
    size_t contentLength = 0;
    if (contentLengthKey != 0) {
      contentLength = static_cast<size_t>(strtoul(contentLengthKey + 15, 0, 10));
    }
    const char* body = headerEnd + 4;
    const size_t bodyLen = length - static_cast<size_t>(body - request);
    if (contentLength > bodyLen) {
      countError();
      sendText(client, 400, "Bad Request", "incomplete request");
      closeClient(client);
      return;
    }
    if (!factoryResetFromBody(body)) {
      countError();
      sendText(client, 400, "Bad Request", "Factory reset confirmation missing\n");
      closeClient(client);
      return;
    }
    sendText(client, 200, "OK", "Factory reset saved. Restarting...\n");
    closeClient(client);
    logLine("[WEB FACTORY RESET] restarting_after_factory_reset");
    gFactoryResetRestart = true;
    gRestartPending = true;
    gRestartAtMs = millis() + kRestartDelayMs;
    return;
  }

  if (strcmp(method, "POST") == 0) {
    uint8_t rowIndex = 0xFF;
    bool isBackupRecall = false;
    bool isRevert = false;
    if (parseRouteActionPath(path, rowIndex, isBackupRecall, isRevert)) {
      const char* reason = 0;
      bool ok = false;
      if (isBackupRecall) {
        if (gLog != 0) {
          char line[64] = {};
          snprintf(line, sizeof(line), "[WEB TEST] row=%u action=backup_recall",
                   static_cast<unsigned>(rowIndex + 1));
          gLog->println(line);
        }
        ok = GpiManager_triggerRouteBackupRecall(rowIndex, reason);
        if (ok) {
          countRouteBackupRecall();
        }
      } else if (isRevert) {
        if (gLog != 0) {
          char line[64] = {};
          snprintf(line, sizeof(line), "[WEB TEST] row=%u action=revert",
                   static_cast<unsigned>(rowIndex + 1));
          gLog->println(line);
        }
        ok = GpiManager_triggerRouteRevert(rowIndex, reason);
        if (ok) {
          countRouteRevert();
        }
      }
      if (!ok) {
        countError();
        countRouteError();
        if (gLog != 0) {
          char line[96] = {};
          snprintf(line,
                   sizeof(line),
                   "[WEB TEST ERROR] route=%u reason=%s",
                   static_cast<unsigned>(rowIndex + 1),
                   reason != 0 ? reason : "route_trigger_failed");
          gLog->println(line);
        }
        char text[64] = {};
        if (reason != 0 && strcmp(reason, "disabled") == 0) {
          snprintf(text, sizeof(text), "Route %u disabled\n", static_cast<unsigned>(rowIndex + 1));
        } else {
          snprintf(text, sizeof(text), "Route %u rejected\n", static_cast<unsigned>(rowIndex + 1));
        }
        sendText(client, 400, "Bad Request", text);
      } else {
        char text[96] = {};
        snprintf(text,
                 sizeof(text),
                 "Route %u %s\n",
                 static_cast<unsigned>(rowIndex + 1),
                 isBackupRecall ? "Backup + Recall sent" : "Revert to Saved scheduled");
        sendText(client, 200, "OK", text);
      }
      closeClient(client);
      return;
    }
  }

  countError();
  sendText(client, 404, "Not Found", "not found");
  closeClient(client);
}

}  // namespace r70v2
}  // namespace dbcan
