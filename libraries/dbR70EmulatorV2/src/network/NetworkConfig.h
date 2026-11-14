#ifndef DB_R70_V2_NETWORK_CONFIG_H
#define DB_R70_V2_NETWORK_CONFIG_H

#include <IPAddress.h>
#include <Print.h>
#include <Stream.h>
#include <stdint.h>

#include "network/NetworkSettings.h"

namespace dbcan {
namespace r70v2 {

bool NetworkConfig_begin(Stream& log);
// Returns true once after a lost Ethernet link has been restored and the
// W5500 network configuration has been reapplied. Callers must then reopen
// their TCP/UDP listeners because reapplying the W5500 clears its sockets.
bool NetworkConfig_poll();
IPAddress NetworkConfig_localIp();
void NetworkConfig_getMac(uint8_t mac[6]);
const char* NetworkConfig_linkStatusName();
const NetworkSettings& NetworkConfig_factorySettings();
const NetworkSettings& NetworkConfig_userSettings();
const ActiveNetworkConfig& NetworkConfig_activeConfig();
ActiveIpSource NetworkConfig_activeIpSource();
bool NetworkConfig_saveUserSettings(const NetworkSettings& settings, const char*& reason);
bool NetworkConfig_resetUserSettingsToFactoryDefaults(const char*& reason);
bool NetworkConfig_handleCliCommand(const char* line, Print& out);

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_NETWORK_CONFIG_H
