#ifndef DB_R70_V2_PENDING_REQUEST_H
#define DB_R70_V2_PENDING_REQUEST_H

#include <stdint.h>

#include "../protocol/DbCanMapping.h"

namespace dbcan {
namespace r70v2 {

struct PendingRequest {
  bool used;
  uint8_t tcpNode;
  uint32_t startedMs;
  uint32_t expiresMs;
  NormalizedRequest request;

  void clear();
  bool expired(uint32_t nowMs) const;
  bool matchesReply(const NormalizedRequest& reply) const;
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_PENDING_REQUEST_H
