#include "AmpState.h"

namespace dbcan {
namespace r70v2 {

void AmpState::bind(FieldCache* storage, uint16_t capacity) {
  caches = storage;
  cacheCapacity = capacity;
  clear();
}

void AmpState::clear() {
  used = false;
  subnet = 0;
  device = 0;
  tcpNode = 0;
  lastSeenMs = 0;
  lastTcpTxToR1Ms = 0;
  lastCanRxMs = 0;
  lastCacheReplyMs = 0;
  lastCanTxPollMs = 0;
  lastTcpRxRequestMs = 0;
  lastTcpRxRequestField = FieldId::Unknown;
  r1TimeoutActive = false;
  lastServiceGapLevel = 0;
  tcpTxCount = 0;
  realCanRxCount = 0;
  cacheReplyCount = 0;
  canTxPollCount = 0;
  timeoutRiskCount = 0;
  lastUnknown0111RequestMs = 0;
  lastUnknown0111RxMs = 0;
  lastUnknown0111TcpTxMs = 0;
  lastAutoTimeoutSnapshotMs = 0;
  maxRequestToTcpTxLatencyMs = 0;
  pendingFieldCount = 0;
  oldestPendingFieldAgeMs = 0;
  unknown0111Requests = 0;
  unknown0111CanTx = 0;
  unknown0111CanRx = 0;
  unknown0111TcpTx = 0;
  unknown0111Pending = 0;
  unknown0111MaxLatencyMs = 0;
  unknown0111LastValue = 0;
  unknown0111CacheValid = false;
  for (uint8_t i = 0; i < kLiveRefreshCount; ++i) liveRefresh[i] = {};
  unknown0111RefreshPending = false;
  unknown0111RefreshRequest = {};
  unknown0111Recovery = {};
  unknown0111RefreshQueuedMs = 0;
  unknown0111RefreshQueuedCount = 0;
  unknown0111RefreshDoneCount = 0;
  unknown0111RefreshSkippedCount = 0;
  unknown0111CacheUpdateMs = 0;
  unknown0111CacheFrame = {};
  object0100HeartbeatQueuedMs = 0;
  object0100HeartbeatLastTriggerMs = 0;
  bootstrapStartedMs = 0;
  bootstrapLastBurstMs = 0;
  bootstrapLastIncompleteLogMs = 0;
  bootstrapSeenMask = 0;
  bootstrapRequestedMask = 0;
  startupBlockerRequestedMask = 0;
  startupBlockerRepliedMask = 0;
  bootstrapRetryPendingMask = 0;
  for (uint8_t index = 0; index < kBootstrapObjectCount; ++index) {
    bootstrapRetrySentMs[index] = 0;
  }
  powerOnBootstrapPendingConfirmation = false;
  powerOnBootstrapUntilMs = 0;
  semanticEpoch = 0;
  for (uint8_t index = 0; index < kSemanticFamilyCount; ++index) {
    semanticFamilies[index] = {};
  }
  if (caches == 0) {
    return;
  }
  for (uint16_t index = 0; index < cacheCapacity; ++index) {
    caches[index].clear();
  }
}

FieldCache* AmpState::cache(FieldId fieldId) {
  if (caches == 0) {
    return 0;
  }
  const uint16_t index = fieldIndex(fieldId);
  if (index >= cacheCapacity) {
    return 0;
  }
  return &caches[index];
}

const FieldCache* AmpState::cache(FieldId fieldId) const {
  if (caches == 0) {
    return 0;
  }
  const uint16_t index = fieldIndex(fieldId);
  if (index >= cacheCapacity) {
    return 0;
  }
  return &caches[index];
}

}  // namespace r70v2
}  // namespace dbcan
