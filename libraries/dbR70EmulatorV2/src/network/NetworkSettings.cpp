#include "network/NetworkSettings.h"

#include <Preferences.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "network/NetworkFactoryDefaults.h"

namespace dbcan {
namespace r70v2 {

namespace {

static const char* kPrefsNamespace = "r70net";
static const char* kPrefsBlobKey = "cfg";
static const uint32_t kSettingsMagic = 0x52373031UL;
static const uint16_t kSettingsVersion = 1;

uint32_t crc32Update(uint32_t crc, uint8_t value) {
  crc ^= value;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 1u) != 0u ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
  }
  return crc;
}

uint32_t crc32Bytes(const uint8_t* bytes, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t index = 0; index < length; ++index) {
    crc = crc32Update(crc, bytes[index]);
  }
  return ~crc;
}

void copyIpv4(uint8_t dest[4], const uint8_t src[4]) {
  for (uint8_t index = 0; index < 4; ++index) {
    dest[index] = src[index];
  }
}

void printIpv4(Print& out, const uint8_t ip[4]) {
  out.print(ip[0]);
  out.print('.');
  out.print(ip[1]);
  out.print('.');
  out.print(ip[2]);
  out.print('.');
  out.print(ip[3]);
}

void logLoadError(Print* log, const char* reason) {
  if (log == 0) {
    return;
  }
  log->print("[NETWORK LOAD ERROR] reason=");
  log->println(reason != 0 ? reason : "unknown");
}

void logLoadRaw(Print* log, const NetworkSettings& settings, size_t storedSize, uint32_t crcCalc) {
  if (log == 0) {
    return;
  }
  log->print("[NETWORK LOAD] raw magic=0x");
  log->print(settings.magic, HEX);
  log->print(" version=");
  log->print(settings.version);
  log->print(" size=");
  log->print(settings.size);
  log->print(" storedBytes=");
  log->print(static_cast<unsigned long>(storedSize));
  log->print(" crcStored=0x");
  log->print(settings.crc32, HEX);
  log->print(" crcCalc=0x");
  log->println(crcCalc, HEX);
}

void logStoreRaw(Print* log, const NetworkSettings& settings) {
  if (log == 0) {
    return;
  }
  log->println("[NETWORK STORE] begin");
  log->print("[NETWORK STORE] magic=0x");
  log->print(settings.magic, HEX);
  log->print(" version=");
  log->print(settings.version);
  log->print(" size=");
  log->print(settings.size);
  log->print(" crc=0x");
  log->println(settings.crc32, HEX);
}

void logVerifyReadback(Print* log, const NetworkSettings& settings) {
  if (log == 0) {
    return;
  }
  log->print("[NETWORK STORE] verify_readback=OK dhcp=");
  log->print(settings.dhcpEnabled ? 1 : 0);
  log->print(" ip=");
  printIpv4(*log, settings.staticIp);
  log->print(" gateway=");
  printIpv4(*log, settings.gateway);
  log->print(" subnet=");
  printIpv4(*log, settings.subnet);
  log->print(" name=");
  log->println(settings.deviceName);
}

}  // namespace

NetworkSettingsStore::NetworkSettingsStore() : lastError_("not_started"), started_(false) {}

bool NetworkSettingsStore::begin() {
  started_ = true;
  lastError_ = "ok";
  return true;
}

bool NetworkSettingsStore::loadOrFactory(NetworkSettings& out, bool& usedFactory, Print* log) {
  usedFactory = true;
  if (log != 0) {
    log->println("[NETWORK LOAD] begin");
  }
  if (!started_) {
    lastError_ = "not_started";
    makeFactoryDefaults(out);
    logLoadError(log, "not_started");
    return false;
  }

  NetworkSettings loaded = {};
  Preferences preferences;
  if (!preferences.begin(kPrefsNamespace, true)) {
    lastError_ = "prefs_open_failed";
    makeFactoryDefaults(out);
    logLoadError(log, "prefs_open_failed");
    return false;
  }

  const size_t storedSize = preferences.getBytesLength(kPrefsBlobKey);
  if (storedSize == 0) {
    preferences.end();
    lastError_ = "missing";
    makeFactoryDefaults(out);
    logLoadError(log, "missing");
    return false;
  }

  const size_t bytesToRead = storedSize < sizeof(loaded) ? storedSize : sizeof(loaded);
  const size_t loadedBytes = preferences.getBytes(kPrefsBlobKey, &loaded, bytesToRead);
  preferences.end();

  const uint32_t crcCalc = computeCrc(loaded);
  logLoadRaw(log, loaded, storedSize, crcCalc);

  if (loadedBytes != bytesToRead) {
    lastError_ = "read_failed";
    makeFactoryDefaults(out);
    logLoadError(log, "read_failed");
    return false;
  }
  if (loaded.magic != kSettingsMagic) {
    lastError_ = "magic_mismatch";
    makeFactoryDefaults(out);
    logLoadError(log, "magic_mismatch");
    return false;
  }
  if (loaded.version != kSettingsVersion) {
    lastError_ = "version_mismatch";
    makeFactoryDefaults(out);
    logLoadError(log, "version_mismatch");
    return false;
  }
  if (loaded.size != sizeof(NetworkSettings) || storedSize != sizeof(NetworkSettings)) {
    lastError_ = "size_mismatch";
    makeFactoryDefaults(out);
    if (log != 0) {
      log->print("[NETWORK LOAD ERROR] reason=size_mismatch stored=");
      log->print(static_cast<unsigned long>(storedSize));
      log->print(" expected=");
      log->println(static_cast<unsigned long>(sizeof(NetworkSettings)));
    }
    return false;
  }
  if (loaded.crc32 != crcCalc) {
    lastError_ = "crc_mismatch";
    makeFactoryDefaults(out);
    if (log != 0) {
      log->print("[NETWORK LOAD ERROR] reason=crc_mismatch stored=0x");
      log->print(loaded.crc32, HEX);
      log->print(" calculated=0x");
      log->println(crcCalc, HEX);
    }
    return false;
  }
  if (!isValid(loaded)) {
    makeFactoryDefaults(out);
    if (log != 0) {
      if (strcmp(lastError_, "bad_static_ip") == 0) {
        log->print("[NETWORK LOAD ERROR] reason=invalid_ip field=staticIp value=");
        printIpv4(*log, loaded.staticIp);
        log->println();
      } else if (strcmp(lastError_, "bad_gateway") == 0) {
        log->print("[NETWORK LOAD ERROR] reason=invalid_ip field=gateway value=");
        printIpv4(*log, loaded.gateway);
        log->println();
      } else if (strcmp(lastError_, "bad_subnet") == 0) {
        log->print("[NETWORK LOAD ERROR] reason=invalid_ip field=subnet value=");
        printIpv4(*log, loaded.subnet);
        log->println();
      } else if (strcmp(lastError_, "bad_device_name") == 0) {
        log->print("[NETWORK LOAD ERROR] reason=invalid_name value=");
        log->println(loaded.deviceName);
      } else {
        logLoadError(log, lastError_);
      }
    }
    return false;
  }

  out = loaded;
  usedFactory = false;
  lastError_ = "ok";
  if (log != 0) {
    log->println("[NETWORK LOAD] validation=OK");
  }
  return true;
}

bool NetworkSettingsStore::saveUserSettings(const NetworkSettings& in, Print* log) {
  if (!started_) {
    lastError_ = "not_started";
    return false;
  }
  NetworkSettings toSave = {};
  copyWithCrc(toSave, in);
  if (!isValid(toSave)) {
    return false;
  }

  logStoreRaw(log, toSave);

  Preferences preferences;
  if (!preferences.begin(kPrefsNamespace, false)) {
    lastError_ = "prefs_open_failed";
    if (log != 0) {
      log->println("[NETWORK STORE] result=FAILED");
    }
    return false;
  }
  const size_t written = preferences.putBytes(kPrefsBlobKey, &toSave, sizeof(toSave));
  preferences.end();
  if (written != sizeof(toSave)) {
    lastError_ = "prefs_write_failed";
    if (log != 0) {
      log->println("[NETWORK STORE] result=FAILED");
    }
    return false;
  }

  Preferences verify;
  if (!verify.begin(kPrefsNamespace, true)) {
    lastError_ = "verify_open_failed";
    if (log != 0) {
      log->println("[NETWORK STORE] result=FAILED");
    }
    return false;
  }
  const size_t verifyLength = verify.getBytesLength(kPrefsBlobKey);
  NetworkSettings readback = {};
  const size_t readbackBytes = verify.getBytes(kPrefsBlobKey, &readback, sizeof(readback));
  verify.end();
  if (verifyLength != sizeof(NetworkSettings) || readbackBytes != sizeof(NetworkSettings)) {
    lastError_ = "verify_size_mismatch";
    if (log != 0) {
      log->println("[NETWORK STORE] result=FAILED");
    }
    return false;
  }
  if (readback.magic != toSave.magic ||
      readback.version != toSave.version ||
      readback.size != toSave.size ||
      readback.crc32 != computeCrc(readback) ||
      memcmp(&readback, &toSave, sizeof(NetworkSettings)) != 0) {
    lastError_ = "verify_readback_failed";
    if (log != 0) {
      log->println("[NETWORK STORE] result=FAILED");
    }
    return false;
  }

  lastError_ = "ok";
  if (log != 0) {
    log->println("[NETWORK STORE] result=OK");
    logVerifyReadback(log, readback);
  }
  return true;
}

bool NetworkSettingsStore::resetUserSettingsToFactoryDefaults() {
  NetworkSettings defaults = {};
  makeFactoryDefaults(defaults);
  return saveUserSettings(defaults);
}

bool NetworkSettingsStore::isValid(const NetworkSettings& settings) {
  if (settings.magic != kSettingsMagic) {
    lastError_ = "bad_magic";
    return false;
  }
  if (settings.version != kSettingsVersion) {
    lastError_ = "unsupported_version";
    return false;
  }
  if (settings.size != sizeof(NetworkSettings)) {
    lastError_ = "size_mismatch";
    return false;
  }
  if (!hasValidIpv4(settings.staticIp)) {
    lastError_ = "bad_static_ip";
    return false;
  }
  if (!hasValidIpv4(settings.gateway)) {
    lastError_ = "bad_gateway";
    return false;
  }
  if (!hasValidIpv4(settings.subnet)) {
    lastError_ = "bad_subnet";
    return false;
  }
  if (!hasValidDeviceName(settings)) {
    lastError_ = "bad_device_name";
    return false;
  }
  if (computeCrc(settings) != settings.crc32) {
    lastError_ = "bad_crc";
    return false;
  }
  lastError_ = "ok";
  return true;
}

void NetworkSettingsStore::makeFactoryDefaults(NetworkSettings& out) {
  memset(&out, 0, sizeof(out));
  out.magic = kSettingsMagic;
  out.version = kSettingsVersion;
  out.size = sizeof(NetworkSettings);
  out.crc32 = 0;
  out.dhcpEnabled = kFactoryDhcpEnabled;
  copyIpv4(out.staticIp, kFactoryStaticIp);
  copyIpv4(out.gateway, kFactoryGateway);
  copyIpv4(out.subnet, kFactorySubnet);
  copyIpv4(out.dns1, kFactoryDns1);
  copyIpv4(out.dns2, kFactoryDns2);
  strncpy(out.deviceName, kFactoryDeviceName, sizeof(out.deviceName) - 1);
  out.deviceName[sizeof(out.deviceName) - 1] = '\0';
  out.crc32 = computeCrc(out);
}

uint32_t NetworkSettingsStore::computeCrc(const NetworkSettings& settings) {
  NetworkSettings temp = settings;
  temp.crc32 = 0;
  return crc32Bytes(reinterpret_cast<const uint8_t*>(&temp), sizeof(temp));
}

const char* NetworkSettingsStore::lastError() const {
  return lastError_ != 0 ? lastError_ : "unknown";
}

bool NetworkSettingsStore::hasValidDeviceName(const NetworkSettings& settings) {
  bool terminated = false;
  for (size_t index = 0; index < sizeof(settings.deviceName); ++index) {
    if (settings.deviceName[index] == '\0') {
      terminated = true;
      break;
    }
  }
  return terminated && settings.deviceName[0] != '\0';
}

bool NetworkSettingsStore::hasValidIpv4(const uint8_t bytes[4]) {
  return bytes != 0 &&
         (bytes[0] != 0 || bytes[1] != 0 || bytes[2] != 0 || bytes[3] != 0);
}

void NetworkSettingsStore::copyWithCrc(NetworkSettings& dest, const NetworkSettings& src) {
  dest = src;
  dest.magic = kSettingsMagic;
  dest.version = kSettingsVersion;
  dest.size = sizeof(NetworkSettings);
  dest.deviceName[sizeof(dest.deviceName) - 1] = '\0';
  dest.crc32 = 0;
  dest.crc32 = crc32Bytes(reinterpret_cast<const uint8_t*>(&dest), sizeof(dest));
}

}  // namespace r70v2
}  // namespace dbcan
