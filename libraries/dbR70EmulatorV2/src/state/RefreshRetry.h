#ifndef DB_R70_REFRESH_RETRY_H
#define DB_R70_REFRESH_RETRY_H
#include <stdint.h>

namespace dbcan { namespace r70v2 {
// Keep pending reply ownership during backoff so late replies still refresh
// the cache without an unsolicited duplicate TCP response.
struct RefreshRetry {
  uint16_t timeouts = 0;
  uint16_t retries = 0;
  uint8_t failures = 0;
  bool expired = false;

  uint32_t backoffMs() const {
    return failures >= 6 ? 30000u : (1000u << (failures ? failures - 1 : 0));
  }
  bool ready(bool pending, uint32_t queuedMs, uint32_t nowMs) {
    if (!pending) return true;
    const uint32_t age = nowMs - queuedMs;
    if (age < 2000u) return false;
    if (!expired) {
      expired = true;
      if (timeouts != UINT16_MAX) ++timeouts;
      if (failures < 6) ++failures;
    }
    return age >= 2000u + backoffMs();
  }
  void enqueued(bool retry) {
    if (retry && retries != UINT16_MAX) ++retries;
    expired = false;
  }
  void completed() { expired = false; failures = 0; }
};
} }
#endif
