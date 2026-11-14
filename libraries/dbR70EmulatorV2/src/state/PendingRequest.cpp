#include "PendingRequest.h"

#include "../protocol/CanObjectCodec.h"

namespace dbcan {
namespace r70v2 {

void PendingRequest::clear() {
  used = false;
  tcpNode = 0;
  startedMs = 0;
  expiresMs = 0;
  request = {};
}

bool PendingRequest::expired(uint32_t nowMs) const {
  return used && static_cast<int32_t>(nowMs - expiresMs) >= 0;
}

bool PendingRequest::matchesReply(const NormalizedRequest& reply) const {
  if (!used || !reply.valid || !request.valid) {
    return false;
  }
  if (request.canId.subnet != reply.canId.subnet || request.canId.device != reply.canId.device) {
    return false;
  }
  if (!isReadFunction(request.admin.functionCode) || !isReadFunction(reply.admin.functionCode)) {
    return false;
  }
  if (request.admin.telegramType != reply.admin.telegramType ||
      request.objectAddress.channelNumber != reply.objectAddress.channelNumber ||
      request.objectAddress.boxNumber != reply.objectAddress.boxNumber ||
      request.objectAddress.objectNumber != reply.objectAddress.objectNumber) {
    return false;
  }
  if (request.objectAddress.hasRecord &&
      request.objectAddress.recordNumber != reply.objectAddress.recordNumber) {
    return false;
  }
  if (request.objectAddress.hasSegment &&
      request.objectAddress.segmentNumber != reply.objectAddress.segmentNumber) {
    return false;
  }
  return true;
}

}  // namespace r70v2
}  // namespace dbcan
