#include "PollPlanner.h"

namespace dbcan {
namespace r70v2 {

void PollPlanner::tick(GatewayState&, uint32_t, DebugCounters&) {
  // Milestone 1 keeps background planning conservative. Requests are created by
  // TCP input and confirmed CAN replies only.
}

}  // namespace r70v2
}  // namespace dbcan
