#ifndef DB_R70_V2_AMP_STATE_H
#define DB_R70_V2_AMP_STATE_H

#include <stdint.h>

#include "FieldCache.h"
#include "RefreshRetry.h"

namespace dbcan {
namespace r70v2 {

struct AmpState {
  static constexpr uint8_t kLiveRefreshCount = 5;
  static const uint8_t kBootstrapObjectCount = 17;
  static const uint8_t kSemanticFamilyCount = 9;

  struct SemanticFamilyState {
    bool valid;
    bool pending;
    uint8_t source;
    uint16_t objectKey;
    uint32_t requestMs;
    uint32_t lastUpdateMs;
    uint16_t epoch;
  };

  bool used;
  uint8_t subnet;
  uint8_t device;
  uint8_t tcpNode;
  uint32_t lastSeenMs;
  uint32_t lastTcpTxToR1Ms;
  uint32_t lastCanRxMs;
  uint32_t lastCacheReplyMs;
  uint32_t lastCanTxPollMs;
  uint32_t lastTcpRxRequestMs;
  FieldId lastTcpRxRequestField;
  bool r1TimeoutActive;
  uint8_t lastServiceGapLevel;
  uint32_t tcpTxCount;
  uint32_t realCanRxCount;
  uint32_t cacheReplyCount;
  uint32_t canTxPollCount;
  uint32_t timeoutRiskCount;
  uint32_t lastUnknown0111RequestMs;
  uint32_t lastUnknown0111RxMs;
  uint32_t lastUnknown0111TcpTxMs;
  uint32_t lastAutoTimeoutSnapshotMs;
  uint32_t maxRequestToTcpTxLatencyMs;
  uint16_t pendingFieldCount;
  uint32_t oldestPendingFieldAgeMs;
  uint32_t unknown0111Requests;
  uint32_t unknown0111CanTx;
  uint32_t unknown0111CanRx;
  uint32_t unknown0111TcpTx;
  uint16_t unknown0111Pending;
  uint32_t unknown0111MaxLatencyMs;
  uint8_t unknown0111LastValue;
  bool unknown0111CacheValid;
  struct LiveRefresh {
    Frame request;
    bool learned;
    bool pending;
    bool sent;
    uint32_t lastR1RequestMs;
    uint32_t queuedMs;
    uint32_t queued;
    uint32_t completed;
    uint32_t skipped;
    RefreshRetry recovery;
  } liveRefresh[kLiveRefreshCount];  // 0500, ISP A/B, Signal A/B; observed reads only.
  bool unknown0111RefreshPending;
  Frame unknown0111RefreshRequest;
  RefreshRetry unknown0111Recovery;
  uint32_t unknown0111RefreshQueuedMs;
  uint32_t unknown0111RefreshQueuedCount;
  uint32_t unknown0111RefreshDoneCount;
  uint32_t unknown0111RefreshSkippedCount;
  uint32_t unknown0111CacheUpdateMs;
  Frame unknown0111CacheFrame;
  uint32_t object0100HeartbeatQueuedMs;
  uint32_t object0100HeartbeatLastTriggerMs;
  uint32_t bootstrapStartedMs;
  uint32_t bootstrapLastBurstMs;
  uint32_t bootstrapLastIncompleteLogMs;
  uint32_t bootstrapSeenMask;
  uint32_t bootstrapRequestedMask;
  uint16_t startupBlockerRequestedMask;
  uint16_t startupBlockerRepliedMask;
  uint32_t bootstrapRetryPendingMask;
  uint32_t bootstrapRetrySentMs[kBootstrapObjectCount];
  bool powerOnBootstrapPendingConfirmation;
  uint32_t powerOnBootstrapUntilMs;
  uint16_t semanticEpoch;
  SemanticFamilyState semanticFamilies[kSemanticFamilyCount];
  FieldCache* caches;
  uint16_t cacheCapacity;

  void bind(FieldCache* storage, uint16_t capacity);
  void clear();
  FieldCache* cache(FieldId fieldId);
  const FieldCache* cache(FieldId fieldId) const;
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_AMP_STATE_H
