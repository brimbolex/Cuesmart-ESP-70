#ifndef DB_R70_V2_WEB_CONFIG_SERVER_H
#define DB_R70_V2_WEB_CONFIG_SERVER_H

#include <Stream.h>

namespace dbcan {
namespace r70v2 {

struct DebugCounters;

void WebConfigServer_begin(Stream& log, DebugCounters* counters = 0);
void WebConfigServer_restart();
void WebConfigServer_poll();
void WebConfigServer_setFactoryResetIndicator(void (*indicator)());

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_WEB_CONFIG_SERVER_H
