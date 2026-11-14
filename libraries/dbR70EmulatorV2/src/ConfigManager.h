#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stddef.h>
#include <stdint.h>

namespace dbcan {

static const uint8_t kMaxDeviceNameLength = 32;
static const uint8_t kMaxHostNameLength = 32;
static const uint8_t kMaxVersionLength = 16;
static const uint8_t kGpiCount = 4;
static const uint8_t kActionCount = 4;
static const uint8_t kActionNameLength = 32;

enum class DeviceMode : uint8_t {
  Gateway = 0,
  Controller = 1,
  ReservedR70Like = 2
};

enum class ActionCommandType : uint8_t {
  None = 0,
  PowerOn,
  PowerOff,
  MuteAOn,
  MuteAOff,
  MuteBOn,
  MuteBOff,
  DelayAOn,
  DelayAOff,
  DelayBOn,
  DelayBOff,
  GainASet,
  GainBSet,
  PresetLoad,
  PresetBackupRestore
};

struct IPv4AddressConfig {
  uint8_t bytes[4];
};

// Persistent config: editable device identity and version labels.
struct DeviceSettingsConfig {
  char deviceName[kMaxDeviceNameLength];
  char hostName[kMaxHostNameLength];
  char hardwareVersion[kMaxVersionLength];
  char firmwareVersion[kMaxVersionLength];
};

// Persistent config: LAN settings. DHCP uses ip/subnet/gateway as fallback/static values.
struct LanSettingsConfig {
  bool dhcpEnabled;
  IPv4AddressConfig ipAddress;
  IPv4AddressConfig subnetMask;
  IPv4AddressConfig gateway;
  IPv4AddressConfig dns;
};

// Persistent config: gateway/dbCAN scheduling and queue sizing.
struct GatewaySettingsConfig {
  uint32_t canBitrate;
  DeviceMode deviceMode;
  uint8_t maxManagedAmps;
  uint16_t pollIntervalFastMs;
  uint16_t pollIntervalSlowMs;
  uint8_t actionQueueLength;
  uint8_t canRxQueueLength;
  uint8_t canTxQueueLength;
  uint8_t ethRxQueueLength;
  uint8_t ethTxQueueLength;
};

// Persistent config: physical GPI input mapping.
struct GpiInputConfig {
  bool enabled;
  uint8_t gpioPin;
  bool activeLow;
  uint16_t debounceMs;
  uint8_t assignedActionIndex;
};

// Persistent config: action definition. optionalValue is command-specific and raw.
struct ActionConfig {
  bool enabled;
  char name[kActionNameLength];
  uint8_t targetAmpIndex;
  ActionCommandType commandType;
  int32_t optionalValue;
  uint32_t delayAfterMs;
};

// Persistent config root. Keep this fixed-size and plain-data friendly.
struct AppConfig {
  uint16_t schemaVersion;
  DeviceSettingsConfig device;
  LanSettingsConfig lan;
  GatewaySettingsConfig gateway;
  GpiInputConfig gpi[kGpiCount];
  ActionConfig actions[kActionCount];
};

// Runtime status: sticky error flags. Not persisted.
struct ErrorStatus {
  bool canRxOverflow;
  bool canTxOverflow;
  bool canTimeout;
  bool canErrorCounterOverflow;
  bool canBusOff;
  bool ethRxOverflow;
  bool ethTxOverflow;
  bool ethLinkDown;
  bool actionQueueOverflow;
  bool configError;
};

// Runtime status: generic queue/buffer counters. Not persisted.
struct QueueStatus {
  uint8_t currentFill;
  uint8_t peakFill;
  uint8_t capacity;
  uint32_t overflowCount;
  uint32_t droppedCount;
};

// Runtime status: debounced GPI state and trigger counters. Not persisted.
struct GpiRuntimeStatus {
  bool currentLevel;
  uint32_t lastTriggerMs;
  uint32_t triggerCount;
};

// Runtime status root. Derived from live system behavior, never saved as config.
struct RuntimeStatus {
  ErrorStatus errors;
  QueueStatus canRxQueue;
  QueueStatus canTxQueue;
  QueueStatus ethRxQueue;
  QueueStatus ethTxQueue;
  QueueStatus actionQueue;
  GpiRuntimeStatus gpi[kGpiCount];
};

class ConfigManager {
 public:
  ConfigManager();

  const AppConfig& defaults() const;
  AppConfig& config();
  const AppConfig& config() const;
  RuntimeStatus& runtimeStatus();
  const RuntimeStatus& runtimeStatus() const;

  bool load();
  bool save() const;
  void resetToDefaults();
  void resetRuntimeStatus();

 private:
  static AppConfig makeDefaultConfig();
  static RuntimeStatus makeDefaultRuntimeStatus(const AppConfig& config);

  AppConfig defaultConfig_;
  AppConfig config_;
  RuntimeStatus runtimeStatus_;
};

}  // namespace dbcan

#endif  // CONFIG_MANAGER_H
