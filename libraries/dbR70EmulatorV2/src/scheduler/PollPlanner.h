#ifndef DB_R70_V2_POLL_PLANNER_H
#define DB_R70_V2_POLL_PLANNER_H

#include "../debug/DebugCounters.h"
#include "../state/GatewayState.h"
#include "HotWarmColdQueues.h"

namespace dbcan {
namespace r70v2 {

class PollPlanner {
 public:
  void tick(GatewayState& state, uint32_t nowMs, DebugCounters& counters);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_POLL_PLANNER_H
