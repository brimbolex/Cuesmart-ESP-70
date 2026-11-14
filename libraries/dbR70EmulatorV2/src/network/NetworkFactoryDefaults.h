#ifndef DB_R70_V2_NETWORK_FACTORY_DEFAULTS_H
#define DB_R70_V2_NETWORK_FACTORY_DEFAULTS_H

#include <stdint.h>

namespace dbcan {
namespace r70v2 {

static const bool kFactoryDhcpEnabled = false;
static const uint8_t kFactoryStaticIp[4] = {192, 168, 254, 70};
static const uint8_t kFactoryGateway[4] = {192, 168, 254, 1};
static const uint8_t kFactorySubnet[4] = {255, 255, 255, 0};
static const uint8_t kFactoryDns1[4] = {192, 168, 254, 1};
static const uint8_t kFactoryDns2[4] = {0, 0, 0, 0};
static const char kFactoryDeviceName[] = "CueSmart-CueCan";

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_NETWORK_FACTORY_DEFAULTS_H
