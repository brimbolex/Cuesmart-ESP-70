#include "ConfigManager.h"

#include <Preferences.h>
#include <string.h>

namespace dbcan {

namespace {

static const char kConfigNamespace[] = "dbcan";
static const char kConfigKey[] = "config";
static const uint32_t kConfigSignature = 0x4442434EU;  // "DBCN"
static const uint16_t kStorageVersion = 2;

struct StoredConfigRecord {
  uint32_t signature;
  uint16_t storageVersion;
  uint16_t configSize;
  uint32_t checksum;
  AppConfig config;
};

IPv4AddressConfig ipv4(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  IPv4AddressConfig value = {{a, b, c, d}};
  return value;
}

void copyText(char* dest, size_t destSize, const char* source) {
  if (dest == 0 || destSize == 0) {
    return;
  }

  if (source == 0) {
    dest[0] = '\0';
    return;
  }

  strncpy(dest, source, destSize - 1);
  dest[destSize - 1] = '\0';
}

QueueStatus queueStatus(uint8_t capacity) {
  QueueStatus status = {};
  status.capacity = capacity;
  return status;
}

uint32_t checksumBytes(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t index = 0; index < len; ++index) {
    hash ^= data[index];
    hash *= 16777619UL;
  }
  return hash;
}

bool hasTerminator(const char* text, size_t size) {
  if (text == 0 || size == 0) {
    return false;
  }

  for (size_t index = 0; index < size; ++index) {
    if (text[index] == '\0') {
      return true;
    }
  }

  return false;
}

bool isValidDeviceMode(DeviceMode mode) {
  return mode == DeviceMode::Gateway || mode == DeviceMode::Controller ||
         mode == DeviceMode::ReservedR70Like;
}

bool isValidCommandType(ActionCommandType commandType) {
  return commandType >= ActionCommandType::None &&
         commandType <= ActionCommandType::PresetBackupRestore;
}

bool validateConfig(const AppConfig& config, const AppConfig& defaults) {
  if (config.schemaVersion != defaults.schemaVersion) {
    return false;
  }

  if (!hasTerminator(config.device.deviceName, sizeof(config.device.deviceName)) ||
      !hasTerminator(config.device.hostName, sizeof(config.device.hostName)) ||
      !hasTerminator(config.device.hardwareVersion, sizeof(config.device.hardwareVersion)) ||
      !hasTerminator(config.device.firmwareVersion, sizeof(config.device.firmwareVersion))) {
    return false;
  }

  if (!isValidDeviceMode(config.gateway.deviceMode)) {
    return false;
  }

  if (config.gateway.canBitrate == 0 || config.gateway.maxManagedAmps == 0 ||
      config.gateway.actionQueueLength == 0 || config.gateway.canRxQueueLength == 0 ||
      config.gateway.canTxQueueLength == 0 || config.gateway.ethRxQueueLength == 0 ||
      config.gateway.ethTxQueueLength == 0) {
    return false;
  }

  for (uint8_t index = 0; index < kGpiCount; ++index) {
    if (config.gpi[index].assignedActionIndex >= kActionCount) {
      return false;
    }
  }

  for (uint8_t index = 0; index < kActionCount; ++index) {
    if (!hasTerminator(config.actions[index].name, sizeof(config.actions[index].name)) ||
        !isValidCommandType(config.actions[index].commandType)) {
      return false;
    }
  }

  return true;
}

}  // namespace

ConfigManager::ConfigManager()
    : defaultConfig_(makeDefaultConfig()),
      config_(defaultConfig_),
      runtimeStatus_(makeDefaultRuntimeStatus(defaultConfig_)) {}

const AppConfig& ConfigManager::defaults() const {
  return defaultConfig_;
}

AppConfig& ConfigManager::config() {
  return config_;
}

const AppConfig& ConfigManager::config() const {
  return config_;
}

RuntimeStatus& ConfigManager::runtimeStatus() {
  return runtimeStatus_;
}

const RuntimeStatus& ConfigManager::runtimeStatus() const {
  return runtimeStatus_;
}

bool ConfigManager::load() {
  Preferences preferences;
  StoredConfigRecord record = {};
  bool ok = false;

  if (preferences.begin(kConfigNamespace, true)) {
    const size_t storedLength = preferences.getBytesLength(kConfigKey);
    if (storedLength == sizeof(record)) {
      const size_t readLength = preferences.getBytes(kConfigKey, &record, sizeof(record));
      if (readLength == sizeof(record) && record.signature == kConfigSignature &&
          record.storageVersion == kStorageVersion &&
          record.configSize == sizeof(AppConfig) &&
          record.checksum == checksumBytes(reinterpret_cast<const uint8_t*>(&record.config),
                                          sizeof(record.config)) &&
          validateConfig(record.config, defaultConfig_)) {
        ok = true;
      }
    }
    preferences.end();
  }

  if (ok) {
    config_ = record.config;
    runtimeStatus_ = makeDefaultRuntimeStatus(config_);
    return true;
  }

  resetToDefaults();
  runtimeStatus_.errors.configError = true;
  return false;
}

bool ConfigManager::save() const {
  if (!validateConfig(config_, defaultConfig_)) {
    return false;
  }

  StoredConfigRecord record = {};
  record.signature = kConfigSignature;
  record.storageVersion = kStorageVersion;
  record.configSize = sizeof(AppConfig);
  record.config = config_;
  record.checksum =
      checksumBytes(reinterpret_cast<const uint8_t*>(&record.config), sizeof(record.config));

  Preferences preferences;
  if (!preferences.begin(kConfigNamespace, false)) {
    return false;
  }

  const size_t writtenLength = preferences.putBytes(kConfigKey, &record, sizeof(record));
  preferences.end();

  return writtenLength == sizeof(record);
}

void ConfigManager::resetToDefaults() {
  config_ = defaultConfig_;
  runtimeStatus_ = makeDefaultRuntimeStatus(config_);
}

void ConfigManager::resetRuntimeStatus() {
  runtimeStatus_ = makeDefaultRuntimeStatus(config_);
}

AppConfig ConfigManager::makeDefaultConfig() {
  AppConfig config = {};

  config.schemaVersion = 2;

  copyText(config.device.deviceName, sizeof(config.device.deviceName), "CueSmart-CueCan");
  copyText(config.device.hostName, sizeof(config.device.hostName), "CueSmart-CueCan");
  copyText(config.device.hardwareVersion, sizeof(config.device.hardwareVersion), "esp32-s3-eth");
  copyText(config.device.firmwareVersion, sizeof(config.device.firmwareVersion), "0.1.0");

  config.lan.dhcpEnabled = true;
  config.lan.ipAddress = ipv4(192, 168, 1, 50);
  config.lan.subnetMask = ipv4(255, 255, 255, 0);
  config.lan.gateway = ipv4(192, 168, 1, 1);
  config.lan.dns = ipv4(192, 168, 1, 1);

  config.gateway.canBitrate = 100000UL;
  config.gateway.deviceMode = DeviceMode::Gateway;
  config.gateway.maxManagedAmps = 4;
  config.gateway.pollIntervalFastMs = 250;
  config.gateway.pollIntervalSlowMs = 2000;
  config.gateway.actionQueueLength = 8;
  config.gateway.canRxQueueLength = 16;
  config.gateway.canTxQueueLength = 16;
  config.gateway.ethRxQueueLength = 8;
  config.gateway.ethTxQueueLength = 8;

  const uint8_t defaultGpiPins[kGpiCount] = {15, 16, 17, 18};
  for (uint8_t index = 0; index < kGpiCount; ++index) {
    config.gpi[index].enabled = false;
    config.gpi[index].gpioPin = defaultGpiPins[index];
    config.gpi[index].activeLow = true;
    config.gpi[index].debounceMs = 30;
    config.gpi[index].assignedActionIndex = index;
  }

  copyText(config.actions[0].name, sizeof(config.actions[0].name), "Power on");
  config.actions[0].enabled = false;
  config.actions[0].targetAmpIndex = 0;
  config.actions[0].commandType = ActionCommandType::PowerOn;
  config.actions[0].optionalValue = 0;
  config.actions[0].delayAfterMs = 0;

  copyText(config.actions[1].name, sizeof(config.actions[1].name), "Power off");
  config.actions[1].enabled = false;
  config.actions[1].targetAmpIndex = 0;
  config.actions[1].commandType = ActionCommandType::PowerOff;
  config.actions[1].optionalValue = 0;
  config.actions[1].delayAfterMs = 0;

  copyText(config.actions[2].name, sizeof(config.actions[2].name), "Mute A");
  config.actions[2].enabled = false;
  config.actions[2].targetAmpIndex = 0;
  config.actions[2].commandType = ActionCommandType::MuteAOn;
  config.actions[2].optionalValue = 0;
  config.actions[2].delayAfterMs = 0;

  copyText(config.actions[3].name, sizeof(config.actions[3].name), "Unmute A");
  config.actions[3].enabled = false;
  config.actions[3].targetAmpIndex = 0;
  config.actions[3].commandType = ActionCommandType::MuteAOff;
  config.actions[3].optionalValue = 0;
  config.actions[3].delayAfterMs = 0;

  return config;
}

RuntimeStatus ConfigManager::makeDefaultRuntimeStatus(const AppConfig& config) {
  RuntimeStatus status = {};

  status.canRxQueue = queueStatus(config.gateway.canRxQueueLength);
  status.canTxQueue = queueStatus(config.gateway.canTxQueueLength);
  status.ethRxQueue = queueStatus(config.gateway.ethRxQueueLength);
  status.ethTxQueue = queueStatus(config.gateway.ethTxQueueLength);
  status.actionQueue = queueStatus(config.gateway.actionQueueLength);

  for (uint8_t index = 0; index < kGpiCount; ++index) {
    // Derived initial level follows inactive state for the configured polarity.
    status.gpi[index].currentLevel = config.gpi[index].activeLow;
    status.gpi[index].lastTriggerMs = 0;
    status.gpi[index].triggerCount = 0;
  }

  return status;
}

}  // namespace dbcan
