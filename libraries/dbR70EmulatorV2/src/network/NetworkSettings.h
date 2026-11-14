#ifndef DB_R70_V2_NETWORK_SETTINGS_H
#define DB_R70_V2_NETWORK_SETTINGS_H

#include <Print.h>
#include <stdint.h>

namespace dbcan {
namespace r70v2 {

struct NetworkSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  bool dhcpEnabled;
  uint8_t staticIp[4];
  uint8_t gateway[4];
  uint8_t subnet[4];
  uint8_t dns1[4];
  uint8_t dns2[4];
  char deviceName[32];
};

enum class ActiveIpSource : uint8_t {
  UserStatic = 0,
  DhcpLease = 1,
  DhcpFailedUserStaticFallback = 2,
  FactoryStaticFallback = 3,
  DhcpFailedFactoryStaticFallback = 4,
};

struct ActiveNetworkConfig {
  ActiveIpSource source;
  uint8_t ip[4];
  uint8_t gateway[4];
  uint8_t subnet[4];
  uint8_t dns1[4];
  uint8_t dns2[4];
  char deviceName[32];
};

class NetworkSettingsStore {
 public:
  NetworkSettingsStore();

  bool begin();
  bool loadOrFactory(NetworkSettings& out, bool& usedFactory, Print* log = 0);
  bool saveUserSettings(const NetworkSettings& in, Print* log = 0);
  bool resetUserSettingsToFactoryDefaults();
  bool isValid(const NetworkSettings& settings);
  void makeFactoryDefaults(NetworkSettings& out);
  uint32_t computeCrc(const NetworkSettings& settings);
  const char* lastError() const;

 private:
  static bool hasValidDeviceName(const NetworkSettings& settings);
  static bool hasValidIpv4(const uint8_t bytes[4]);
  static void copyWithCrc(NetworkSettings& dest, const NetworkSettings& src);

  const char* lastError_;
  bool started_;
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_NETWORK_SETTINGS_H
