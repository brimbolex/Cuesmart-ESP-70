#include "network/NetworkConfig.h"

#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>
#include <esp_mac.h>
#include <string.h>

namespace dbcan {
namespace r70v2 {

namespace {

static const int8_t kW5500SckPin = 13;
static const int8_t kW5500MisoPin = 12;
static const int8_t kW5500MosiPin = 11;
static const int8_t kW5500CsPin = 14;
static const int8_t kW5500ResetPin = 9;
static const uint32_t kDhcpTimeoutMs = 6000;
static const uint32_t kLinkPollIntervalMs = 100;
static const uint32_t kLinkRestoreStableMs = 750;

NetworkSettingsStore gStore;
NetworkSettings gFactorySettings = {};
NetworkSettings gUserSettings = {};
ActiveNetworkConfig gActiveConfig = {};
bool gUserSettingsFromFactory = true;
IPAddress gLocalIp(0, 0, 0, 0);
uint8_t gMac[6] = {};
Stream* gLog = 0;
EthernetLinkStatus gLastLinkStatus = Unknown;
uint32_t gLastLinkPollMs = 0;
uint32_t gLinkOnSinceMs = 0;
bool gLinkLossObserved = false;

void buildLocalMac(uint8_t mac[6]) {
  uint8_t factory[6] = {};
  esp_read_mac(factory, ESP_MAC_WIFI_STA);
  mac[0] = static_cast<uint8_t>((factory[0] | 0x02) & 0xFE);
  for (uint8_t index = 1; index < 6; ++index) {
    mac[index] = factory[index];
  }
}

void resetW5500() {
  pinMode(kW5500ResetPin, OUTPUT);
  digitalWrite(kW5500ResetPin, LOW);
  delay(50);
  digitalWrite(kW5500ResetPin, HIGH);
  delay(200);
}

IPAddress toIpAddress(const uint8_t bytes[4]) {
  return IPAddress(bytes[0], bytes[1], bytes[2], bytes[3]);
}

void copyIpv4(uint8_t dest[4], const uint8_t src[4]) {
  for (uint8_t index = 0; index < 4; ++index) {
    dest[index] = src[index];
  }
}

void copyText(char* dest, size_t capacity, const char* src) {
  if (dest == 0 || capacity == 0) {
    return;
  }
  if (src == 0) {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, src, capacity - 1);
  dest[capacity - 1] = '\0';
}

const char* activeIpSourceName(ActiveIpSource source) {
  switch (source) {
    case ActiveIpSource::UserStatic:
      return "UserStatic";
    case ActiveIpSource::DhcpLease:
      return "DhcpLease";
    case ActiveIpSource::DhcpFailedUserStaticFallback:
      return "DhcpFailedUserStaticFallback";
    case ActiveIpSource::FactoryStaticFallback:
      return "FactoryStaticFallback";
    case ActiveIpSource::DhcpFailedFactoryStaticFallback:
      return "DhcpFailedFactoryStaticFallback";
    default:
      return "Unknown";
  }
}

void printIpv4(Print& out, const uint8_t bytes[4]) {
  out.print(bytes[0]);
  out.print('.');
  out.print(bytes[1]);
  out.print('.');
  out.print(bytes[2]);
  out.print('.');
  out.print(bytes[3]);
}

void printMac(Print& out, const uint8_t mac[6]) {
  for (uint8_t index = 0; index < 6; ++index) {
    if (mac[index] < 0x10) {
      out.print('0');
    }
    out.print(mac[index], HEX);
    if (index < 5) {
      out.print(':');
    }
  }
}

void fillActiveFromUserSettings(ActiveNetworkConfig& active,
                                const NetworkSettings& settings,
                                ActiveIpSource source,
                                const IPAddress& ipOverride,
                                const IPAddress& gatewayOverride,
                                const IPAddress& subnetOverride,
                                const IPAddress& dns1Override) {
  memset(&active, 0, sizeof(active));
  active.source = source;
  if (ipOverride != IPAddress(0, 0, 0, 0)) {
    active.ip[0] = ipOverride[0];
    active.ip[1] = ipOverride[1];
    active.ip[2] = ipOverride[2];
    active.ip[3] = ipOverride[3];
  } else {
    copyIpv4(active.ip, settings.staticIp);
  }
  if (gatewayOverride != IPAddress(0, 0, 0, 0)) {
    active.gateway[0] = gatewayOverride[0];
    active.gateway[1] = gatewayOverride[1];
    active.gateway[2] = gatewayOverride[2];
    active.gateway[3] = gatewayOverride[3];
  } else {
    copyIpv4(active.gateway, settings.gateway);
  }
  if (subnetOverride != IPAddress(0, 0, 0, 0)) {
    active.subnet[0] = subnetOverride[0];
    active.subnet[1] = subnetOverride[1];
    active.subnet[2] = subnetOverride[2];
    active.subnet[3] = subnetOverride[3];
  } else {
    copyIpv4(active.subnet, settings.subnet);
  }
  if (dns1Override != IPAddress(0, 0, 0, 0)) {
    active.dns1[0] = dns1Override[0];
    active.dns1[1] = dns1Override[1];
    active.dns1[2] = dns1Override[2];
    active.dns1[3] = dns1Override[3];
  } else {
    copyIpv4(active.dns1, settings.dns1);
  }
  copyIpv4(active.dns2, settings.dns2);
  copyText(active.deviceName, sizeof(active.deviceName), settings.deviceName);
}

void printFactoryLine(Print& out, const NetworkSettings& settings) {
  out.print(F("NETWORK_FACTORY ip="));
  printIpv4(out, settings.staticIp);
  out.print(F(" gateway="));
  printIpv4(out, settings.gateway);
  out.print(F(" subnet="));
  printIpv4(out, settings.subnet);
  out.print(F(" name="));
  out.println(settings.deviceName);
}

void printSettingsLine(Print& out,
                       const NetworkSettings& settings,
                       const char* source) {
  out.print(F("[NETWORK_SETTINGS] source="));
  out.print(source != 0 ? source : "unknown");
  out.print(F(" dhcp="));
  out.print(settings.dhcpEnabled ? 1 : 0);
  out.print(F(" staticIp="));
  printIpv4(out, settings.staticIp);
  out.print(F(" gateway="));
  printIpv4(out, settings.gateway);
  out.print(F(" subnet="));
  printIpv4(out, settings.subnet);
  out.print(F(" name="));
  out.println(settings.deviceName);
}

void printActiveLine(Print& out, const ActiveNetworkConfig& active) {
  out.print(F("NETWORK_ACTIVE source="));
  out.print(activeIpSourceName(active.source));
  out.print(F(" ip="));
  printIpv4(out, active.ip);
  out.print(F(" name="));
  out.println(active.deviceName);
}

bool parseIpv4Text(const char* text, uint8_t out[4]) {
  if (text == 0 || out == 0) {
    return false;
  }
  const char* cursor = text;
  for (uint8_t index = 0; index < 4; ++index) {
    char* endPtr = 0;
    const unsigned long value = strtoul(cursor, &endPtr, 10);
    if (endPtr == cursor || value > 255) {
      return false;
    }
    out[index] = static_cast<uint8_t>(value);
    if (index < 3) {
      if (*endPtr != '.') {
        return false;
      }
      cursor = endPtr + 1;
    } else if (*endPtr != '\0') {
      return false;
    }
  }
  return true;
}

void printSettingsSummary(Print& out,
                          const char* prefix,
                          const NetworkSettings& settings) {
  out.print(prefix);
  out.print(F(" dhcp="));
  out.print(settings.dhcpEnabled ? 1 : 0);
  out.print(F(" staticIp="));
  printIpv4(out, settings.staticIp);
  out.print(F(" gateway="));
  printIpv4(out, settings.gateway);
  out.print(F(" subnet="));
  printIpv4(out, settings.subnet);
  out.print(F(" dns1="));
  printIpv4(out, settings.dns1);
  out.print(F(" dns2="));
  printIpv4(out, settings.dns2);
  out.print(F(" deviceName="));
  out.println(settings.deviceName);
}

const char* linkStatusName(EthernetLinkStatus status) {
  switch (status) {
    case Unknown:
      return "Unknown";
    case LinkON:
      return "LinkON";
    case LinkOFF:
      return "LinkOFF";
    default:
      return "Unknown";
  }
}

}  // namespace

bool NetworkConfig_begin(Stream& log) {
  gLog = &log;
  log.println(F("[BOOT_STAGE] before_network_settings"));
  gStore.begin();
  gStore.makeFactoryDefaults(gFactorySettings);
  printFactoryLine(log, gFactorySettings);

  gStore.loadOrFactory(gUserSettings, gUserSettingsFromFactory, &log);
  if (gUserSettingsFromFactory) {
    log.print(F("NETWORK_SETTINGS invalid_reason="));
    log.println(gStore.lastError());
  }
  printSettingsLine(log, gUserSettings, gUserSettingsFromFactory ? "factory" : "persistent");
  log.println(F("[BOOT_STAGE] after_network_settings"));

  SPI.begin(kW5500SckPin, kW5500MisoPin, kW5500MosiPin, kW5500CsPin);
  Ethernet.init(kW5500CsPin);
  resetW5500();
  buildLocalMac(gMac);
  log.println(F("[BOOT_STAGE] before_network_apply"));

  if (!gUserSettings.dhcpEnabled) {
    Ethernet.begin(gMac,
                   toIpAddress(gUserSettings.staticIp),
                   toIpAddress(gUserSettings.dns1),
                   toIpAddress(gUserSettings.gateway),
                   toIpAddress(gUserSettings.subnet));
    fillActiveFromUserSettings(gActiveConfig,
                               gUserSettings,
                               gUserSettingsFromFactory ? ActiveIpSource::FactoryStaticFallback
                                                        : ActiveIpSource::UserStatic,
                               IPAddress(0, 0, 0, 0),
                               IPAddress(0, 0, 0, 0),
                               IPAddress(0, 0, 0, 0),
                               IPAddress(0, 0, 0, 0));
    log.print(F("NETWORK_APPLY mode=static source="));
    log.print(gUserSettingsFromFactory ? F("factory") : F("user"));
    log.print(F(" ip="));
    printIpv4(log, gActiveConfig.ip);
    log.println();
  } else {
    log.print(F("NETWORK_APPLY mode=dhcp timeout_ms="));
    log.println(static_cast<unsigned long>(kDhcpTimeoutMs));
    if (Ethernet.begin(gMac, kDhcpTimeoutMs) == 1) {
      fillActiveFromUserSettings(gActiveConfig,
                                 gUserSettings,
                                 ActiveIpSource::DhcpLease,
                                 Ethernet.localIP(),
                                 Ethernet.gatewayIP(),
                                 Ethernet.subnetMask(),
                                 Ethernet.dnsServerIP());
      log.print(F("NETWORK_DHCP_OK ip="));
      log.print(Ethernet.localIP());
      log.print(F(" gateway="));
      log.print(Ethernet.gatewayIP());
      log.print(F(" subnet="));
      log.println(Ethernet.subnetMask());
    } else {
      Ethernet.begin(gMac,
                     toIpAddress(gUserSettings.staticIp),
                     toIpAddress(gUserSettings.dns1),
                     toIpAddress(gUserSettings.gateway),
                     toIpAddress(gUserSettings.subnet));
      fillActiveFromUserSettings(gActiveConfig,
                                 gUserSettings,
                                 gUserSettingsFromFactory
                                     ? ActiveIpSource::DhcpFailedFactoryStaticFallback
                                     : ActiveIpSource::DhcpFailedUserStaticFallback,
                                 IPAddress(0, 0, 0, 0),
                                 IPAddress(0, 0, 0, 0),
                                 IPAddress(0, 0, 0, 0),
                                 IPAddress(0, 0, 0, 0));
      log.print(F("NETWORK_DHCP_FAIL fallback="));
      log.print(gUserSettingsFromFactory ? F("factory_static") : F("user_static"));
      log.print(F(" ip="));
      printIpv4(log, gActiveConfig.ip);
      log.println();
    }
  }

  gLocalIp = toIpAddress(gActiveConfig.ip);
  printActiveLine(log, gActiveConfig);
  log.print(F("ETH_IP="));
  log.println(gLocalIp);
  log.print(F("ETH_MAC="));
  printMac(log, gMac);
  log.println();
  log.println(F("[BOOT_STAGE] after_network_apply"));
  gLastLinkStatus = Ethernet.linkStatus();
  gLastLinkPollMs = millis();
  gLinkOnSinceMs = gLastLinkStatus == LinkON ? gLastLinkPollMs : 0;
  gLinkLossObserved = gLastLinkStatus == LinkOFF;
  return true;
}

bool NetworkConfig_poll() {
  const uint32_t nowMs = millis();
  if (static_cast<int32_t>(nowMs - (gLastLinkPollMs + kLinkPollIntervalMs)) < 0) {
    return false;
  }
  gLastLinkPollMs = nowMs;

  const EthernetLinkStatus status = Ethernet.linkStatus();
  if (status != gLastLinkStatus) {
    if (gLog != 0) {
      gLog->print(F("NETWORK_LINK_CHANGE from="));
      gLog->print(linkStatusName(gLastLinkStatus));
      gLog->print(F(" to="));
      gLog->println(linkStatusName(status));
    }
    gLastLinkStatus = status;
  }

  if (status == LinkOFF) {
    gLinkLossObserved = true;
    gLinkOnSinceMs = 0;
    return false;
  }
  if (status != LinkON || !gLinkLossObserved) {
    return false;
  }
  if (gLinkOnSinceMs == 0) {
    gLinkOnSinceMs = nowMs;
    return false;
  }
  if (static_cast<int32_t>(nowMs - (gLinkOnSinceMs + kLinkRestoreStableMs)) < 0) {
    return false;
  }

  // Reuse the currently active address, including an existing DHCP lease.
  // This avoids a blocking DHCP negotiation in the realtime loop while still
  // rebuilding the W5500 socket state after the physical link returns.
  Ethernet.begin(gMac,
                 toIpAddress(gActiveConfig.ip),
                 toIpAddress(gActiveConfig.dns1),
                 toIpAddress(gActiveConfig.gateway),
                 toIpAddress(gActiveConfig.subnet));
  gLinkLossObserved = false;
  gLinkOnSinceMs = 0;
  gLastLinkStatus = Ethernet.linkStatus();
  if (gLog != 0) {
    gLog->print(F("NETWORK_LINK_RECOVERED ip="));
    gLog->print(gLocalIp);
    gLog->println(F(" reopen_sockets=1"));
  }
  return true;
}

IPAddress NetworkConfig_localIp() {
  return gLocalIp;
}

void NetworkConfig_getMac(uint8_t mac[6]) {
  if (mac == 0) {
    return;
  }
  for (uint8_t index = 0; index < 6; ++index) {
    mac[index] = gMac[index];
  }
}

const char* NetworkConfig_linkStatusName() {
  return linkStatusName(Ethernet.linkStatus());
}

const NetworkSettings& NetworkConfig_factorySettings() {
  return gFactorySettings;
}

const NetworkSettings& NetworkConfig_userSettings() {
  return gUserSettings;
}

const ActiveNetworkConfig& NetworkConfig_activeConfig() {
  return gActiveConfig;
}

ActiveIpSource NetworkConfig_activeIpSource() {
  return gActiveConfig.source;
}

bool NetworkConfig_saveUserSettings(const NetworkSettings& settings, const char*& reason) {
  if (!gStore.saveUserSettings(settings, gLog)) {
    reason = gStore.lastError();
    return false;
  }
  gUserSettings = settings;
  gUserSettingsFromFactory = false;
  reason = "ok";
  return true;
}

bool NetworkConfig_resetUserSettingsToFactoryDefaults(const char*& reason) {
  if (!gStore.resetUserSettingsToFactoryDefaults()) {
    reason = gStore.lastError();
    return false;
  }
  gStore.makeFactoryDefaults(gUserSettings);
  gUserSettingsFromFactory = false;
  reason = "ok";
  return true;
}

bool NetworkConfig_handleCliCommand(const char* line, Print& out) {
  if (line == 0) {
    return false;
  }
  if (strcmp(line, "net-show") == 0) {
    printSettingsSummary(out, "NET_SHOW_FACTORY", gFactorySettings);
    printSettingsSummary(out, gUserSettingsFromFactory ? "NET_SHOW_USER source=factory" : "NET_SHOW_USER source=nvs", gUserSettings);
    out.print(F("NET_SHOW_ACTIVE source="));
    out.print(activeIpSourceName(gActiveConfig.source));
    out.print(F(" ip="));
    printIpv4(out, gActiveConfig.ip);
    out.print(F(" gateway="));
    printIpv4(out, gActiveConfig.gateway);
    out.print(F(" subnet="));
    printIpv4(out, gActiveConfig.subnet);
    out.print(F(" dns1="));
    printIpv4(out, gActiveConfig.dns1);
    out.print(F(" dns2="));
    printIpv4(out, gActiveConfig.dns2);
    out.print(F(" deviceName="));
    out.println(gActiveConfig.deviceName);
    return true;
  }
  if (strcmp(line, "net-dhcp on") == 0) {
    gUserSettings.dhcpEnabled = true;
    if (!gStore.saveUserSettings(gUserSettings)) {
      out.print(F("NET_DHCP_ON failed reason="));
      out.println(gStore.lastError());
      return true;
    }
    gUserSettingsFromFactory = false;
    out.println(F("DHCP enabled; restart required."));
    return true;
  }
  if (strcmp(line, "net-dhcp off") == 0) {
    gUserSettings.dhcpEnabled = false;
    if (!gStore.saveUserSettings(gUserSettings)) {
      out.print(F("NET_DHCP_OFF failed reason="));
      out.println(gStore.lastError());
      return true;
    }
    gUserSettingsFromFactory = false;
    out.println(F("static IP enabled; restart required."));
    return true;
  }
  if (strncmp(line, "net-set-static ", 15) == 0) {
    char ipText[24] = {};
    char gatewayText[24] = {};
    char subnetText[24] = {};
    char dns1Text[24] = {};
    char dns2Text[24] = {};
    const int parsed = sscanf(line + 15,
                              "%23s %23s %23s %23s %23s",
                              ipText,
                              gatewayText,
                              subnetText,
                              dns1Text,
                              dns2Text);
    if (parsed < 3) {
      out.println(F("NET_SET_STATIC failed reason=usage"));
      return true;
    }
    NetworkSettings next = gUserSettings;
    if (!parseIpv4Text(ipText, next.staticIp) ||
        !parseIpv4Text(gatewayText, next.gateway) ||
        !parseIpv4Text(subnetText, next.subnet)) {
      out.println(F("NET_SET_STATIC failed reason=invalid_ip"));
      return true;
    }
    if (parsed >= 4) {
      if (!parseIpv4Text(dns1Text, next.dns1)) {
        out.println(F("NET_SET_STATIC failed reason=invalid_dns1"));
        return true;
      }
    }
    if (parsed >= 5) {
      if (!parseIpv4Text(dns2Text, next.dns2)) {
        out.println(F("NET_SET_STATIC failed reason=invalid_dns2"));
        return true;
      }
    }
    if (!gStore.saveUserSettings(next)) {
      out.print(F("NET_SET_STATIC failed reason="));
      out.println(gStore.lastError());
      return true;
    }
    gUserSettings = next;
    gUserSettingsFromFactory = false;
    out.println(F("NET_SET_STATIC ok restart_required=1"));
    return true;
  }
  if (strncmp(line, "net-set-name ", 13) == 0) {
    const char* name = line + 13;
    if (*name == '\0') {
      out.println(F("NET_SET_NAME failed reason=empty"));
      return true;
    }
    NetworkSettings next = gUserSettings;
    copyText(next.deviceName, sizeof(next.deviceName), name);
    if (!gStore.saveUserSettings(next)) {
      out.print(F("NET_SET_NAME failed reason="));
      out.println(gStore.lastError());
      return true;
    }
    gUserSettings = next;
    gUserSettingsFromFactory = false;
    out.println(F("NET_SET_NAME ok restart_required=1"));
    return true;
  }
  if (strcmp(line, "net-reset-factory") == 0) {
    if (!gStore.resetUserSettingsToFactoryDefaults()) {
      out.print(F("NET_RESET_FACTORY failed reason="));
      out.println(gStore.lastError());
      return true;
    }
    gStore.makeFactoryDefaults(gUserSettings);
    gUserSettingsFromFactory = false;
    out.println(F("NET_RESET_FACTORY ok restart_required=1"));
    return true;
  }
  if (strcmp(line, "net-save-test") == 0) {
    if (!gStore.saveUserSettings(gUserSettings)) {
      out.print(F("NET_SAVE_TEST failed reason="));
      out.println(gStore.lastError());
      return true;
    }
    out.println(F("NET_SAVE_TEST ok"));
    return true;
  }
  return false;
}

}  // namespace r70v2
}  // namespace dbcan
