#ifndef DB_R70_V2_GPI_ACTIONS_H
#define DB_R70_V2_GPI_ACTIONS_H

#include <Print.h>
#include <stdint.h>

#include "gpi/GpiConfig.h"

namespace dbcan {
namespace r70v2 {

class R70GatewayV2;

void GpiActions_begin(R70GatewayV2& gateway, Print& log);
uint16_t makePrimaryMasterCanId(uint8_t subnet, uint8_t device);
bool GpiActions_triggerPress(uint8_t slotIndex,
                             const GpiPresetRouteConfig& input,
                             const char* source);
bool GpiActions_triggerRelease(uint8_t slotIndex,
                               const GpiPresetRouteConfig& input,
                               const char* source,
                               uint8_t repeatNumber = 1,
                               uint8_t repeatTotal = 1);

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_GPI_ACTIONS_H
