#include "gpi/GpiConfig.h"

#include <Preferences.h>
#include <driver/gpio.h>
#include <stddef.h>
#include <string.h>

namespace dbcan {
namespace r70v2 {

namespace {

static const char* kPrefsNamespace = "r70gpi";
static const char* kPrefsBlobKey = "cfg";
static const uint32_t kSettingsMagic = 0x47504932UL;
static const uint16_t kSettingsVersion = 2;
static const uint8_t kMinLoadPreset = 1;
static const uint8_t kMaxLoadPreset = 14;
static const uint8_t kMinBackupSlot = 1;
static const uint8_t kMaxBackupSlot = 14;

// Reserved / rejected GPIOs for Gate31 hardware:
// - GPIO1, GPIO2: TWAI CAN RX/TX
// - GPIO4: physical factory-reset button
// - GPIO5: two daisy-chained WS2812 status LEDs
// - GPIO9, GPIO11, GPIO12, GPIO13, GPIO14: W5500 reset/SPI
// - GPIO10: reserved beside the W5500 group
// - GPIO19, GPIO20: native USB
// - GPIO0, GPIO3, GPIO45, GPIO46: conservative boot/strap avoidance
uint32_t crc32(const uint8_t* bytes, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint32_t>(bytes[index]);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

bool pinReserved(uint8_t pin) {
  switch (pin) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 19:
    case 20:
    case 45:
    case 46:
      return true;
    default:
      return false;
  }
}

}  // namespace

GpiSettingsStore::GpiSettingsStore() : lastError_("not_started"), started_(false) {}

bool isGpiPinAllowed(uint8_t pin) {
  if (!GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(pin))) {
    return false;
  }
  return !pinReserved(pin);
}

const char* gpiActionModeName(GpiActionMode mode) {
  switch (mode) {
    case GpiActionMode::None:
      return "None";
    case GpiActionMode::BackupLoadRestore:
      return "BackupLoadRestore";
    default:
      return "Unknown";
  }
}

bool GpiSettingsStore::begin() {
  started_ = true;
  lastError_ = "ok";
  return true;
}

bool GpiSettingsStore::loadOrFactory(GpiSettings& out, bool& usedFactory) {
  usedFactory = true;
  if (!started_) {
    lastError_ = "not_started";
    makeFactoryDefaults(out);
    return false;
  }

  Preferences preferences;
  GpiSettings loaded = {};
  if (!preferences.begin(kPrefsNamespace, true)) {
    lastError_ = "prefs_open_failed";
    makeFactoryDefaults(out);
    return false;
  }
  const size_t storedBytes = preferences.getBytesLength(kPrefsBlobKey);
  if (storedBytes != sizeof(loaded)) {
    preferences.end();
    lastError_ = "missing_or_size_mismatch";
    makeFactoryDefaults(out);
    return false;
  }
  const size_t loadedBytes = preferences.getBytes(kPrefsBlobKey, &loaded, sizeof(loaded));
  preferences.end();
  if (loadedBytes != sizeof(loaded)) {
    lastError_ = "missing_or_size_mismatch";
    makeFactoryDefaults(out);
    return false;
  }
  if (!isValid(loaded)) {
    makeFactoryDefaults(out);
    return false;
  }

  out = loaded;
  usedFactory = false;
  lastError_ = "ok";
  return true;
}

bool GpiSettingsStore::saveUserSettings(const GpiSettings& in) {
  if (!started_) {
    lastError_ = "not_started";
    return false;
  }

  GpiSettings toSave = {};
  copyWithCrc(toSave, in);
  if (!isValid(toSave)) {
    return false;
  }

  Preferences preferences;
  if (!preferences.begin(kPrefsNamespace, false)) {
    lastError_ = "prefs_open_failed";
    return false;
  }
  const size_t written = preferences.putBytes(kPrefsBlobKey, &toSave, sizeof(toSave));
  preferences.end();
  if (written != sizeof(toSave)) {
    lastError_ = "prefs_write_failed";
    return false;
  }
  lastError_ = "ok";
  return true;
}

bool GpiSettingsStore::resetUserSettingsToFactoryDefaults() {
  GpiSettings defaults = {};
  makeFactoryDefaults(defaults);
  return saveUserSettings(defaults);
}

void GpiSettingsStore::makeFactoryDefaults(GpiSettings& out) {
  memset(&out, 0, sizeof(out));
  out.magic = kSettingsMagic;
  out.version = kSettingsVersion;
  out.size = sizeof(out);

  static const uint8_t kDefaultPhysicalGpi[kGpiRouteCount] = {1, 2, 3, 4, 1, 2, 3, 4};
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    GpiRouteRow& route = out.route[index];
    route.enabled = index < 4 ? 1 : 0;
    route.physicalGpi = kDefaultPhysicalGpi[index];
    route.targetSubnet = 2;
    route.targetDevice = index;
    route.loadPreset = 12;
    route.backupSlot = 14;
    route.actionMode = GpiActionMode::BackupLoadRestore;
  }

  out.crc32 = 0;
  out.crc32 = computeCrc(out);
}

bool GpiSettingsStore::isValid(const GpiSettings& settings) {
  if (settings.magic != kSettingsMagic) {
    lastError_ = "bad_magic";
    return false;
  }
  if (settings.version != kSettingsVersion) {
    lastError_ = "unsupported_version";
    return false;
  }
  if (settings.size != sizeof(GpiSettings)) {
    lastError_ = "size_mismatch";
    return false;
  }
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    const GpiRouteRow& route = settings.route[index];
    if (!route.enabled) {
      continue;
    }
    if (route.physicalGpi < 1 || route.physicalGpi > kGpiPhysicalCount) {
      lastError_ = "invalid_physical_gpi";
      return false;
    }
    if (route.targetSubnet > 7) {
      lastError_ = "invalid_subnet";
      return false;
    }
    if (route.targetDevice > 63) {
      lastError_ = "invalid_device";
      return false;
    }
    if (route.loadPreset < kMinLoadPreset || route.loadPreset > kMaxLoadPreset) {
      lastError_ = "invalid_load_preset";
      return false;
    }
    if (route.backupSlot < kMinBackupSlot || route.backupSlot > kMaxBackupSlot) {
      lastError_ = "invalid_backup_slot";
      return false;
    }
    if (route.actionMode != GpiActionMode::None &&
        route.actionMode != GpiActionMode::BackupLoadRestore) {
      lastError_ = "invalid_action_mode";
      return false;
    }
  }
  if (computeCrc(settings) != settings.crc32) {
    lastError_ = "bad_crc";
    return false;
  }
  lastError_ = "ok";
  return true;
}

const char* GpiSettingsStore::lastError() const {
  return lastError_ != 0 ? lastError_ : "unknown";
}

uint32_t GpiSettingsStore::computeCrc(const GpiSettings& settings) const {
  GpiSettings copy = settings;
  copy.crc32 = 0;
  return crc32(reinterpret_cast<const uint8_t*>(&copy), sizeof(copy));
}

void GpiSettingsStore::copyWithCrc(GpiSettings& dest, const GpiSettings& src) const {
  dest = src;
  dest.magic = kSettingsMagic;
  dest.version = kSettingsVersion;
  dest.size = sizeof(dest);
  dest.crc32 = 0;
  dest.crc32 = computeCrc(dest);
}

}  // namespace r70v2
}  // namespace dbcan
