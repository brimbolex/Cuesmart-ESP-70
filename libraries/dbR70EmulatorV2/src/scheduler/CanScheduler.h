#ifndef DB_R70_V2_CAN_SCHEDULER_H
#define DB_R70_V2_CAN_SCHEDULER_H

#include <stdint.h>

#include "../debug/DebugCli.h"
#include "../debug/DebugCounters.h"
#include "../state/GatewayState.h"
#include "../transport/CanTwaiTransport.h"
#include "HotWarmColdQueues.h"

namespace dbcan {
namespace r70v2 {

class CanScheduler {
 public:
  typedef void (*CanTxObserverFn)(void* context,
                                  const NormalizedRequest& request,
                                  QueueClass queueClass,
                                  uint32_t enqueuedMs,
                                  uint32_t nowMs);

  static const uint32_t kPacingFastNormalUs = 1500;
  static const uint32_t kPacingFastWriteUs = 1500;
  static const uint32_t kPacingFastScanUs = 2000;
  static const uint32_t kPacingSafeNormalUs = 2500;
  static const uint32_t kPacingSafeWriteUs = 2000;
  static const uint32_t kPacingSafeScanUs = 2500;
  static const uint32_t kPacingVerySafeNormalUs = 3000;
  static const uint32_t kPacingVerySafeWriteUs = 2500;
  static const uint32_t kPacingVerySafeScanUs = 3500;
  static const uint8_t kMaxConsecutiveNormalReadsPerSubnet = 2;

  CanScheduler();

  void begin(CanTwaiTransport& transport,
             uint32_t minSpacingUsNormal,
             uint32_t minSpacingUsWrite,
             uint32_t minSpacingUsScan,
             uint32_t identWindowMs,
             uint32_t identQuietHoldMs,
             bool enableWriteRetry,
             uint32_t writeAckTimeoutMs,
             const DebugCli& debugCli);
  bool enqueue(const NormalizedRequest& request,
               uint8_t tcpNode,
               bool expectReply,
               uint32_t nowMs,
               DebugCounters& counters,
               const char*& failureReason);
  bool enqueueMaintenance(const NormalizedRequest& request,
                          uint32_t nowMs,
                          DebugCounters& counters,
                          const char*& failureReason);
  bool enqueueBootstrapRetry(const NormalizedRequest& request,
                             uint32_t nowMs,
                             DebugCounters& counters,
                             const char*& failureReason);
  void tick(GatewayState& state, uint32_t nowMs, DebugCounters& counters);
  void observeCanRx(const NormalizedRequest& request, uint32_t nowMs, DebugCounters& counters);
  void setCanTxObserver(CanTxObserverFn observer, void* context);
  uint32_t currentSpacingUs(uint32_t nowMs) const;
  bool identPhaseActive(uint32_t nowMs) const;
  uint8_t identSubnet() const;
  uint32_t identWindowRemainingMs(uint32_t nowMs) const;
  uint16_t queueDepth() const;
  uint16_t writeQueueDepth() const;
  uint16_t maintenanceQueueDepth() const;
  uint16_t identQueueDepth() const;
  uint16_t normalQueueDepth() const;
  uint16_t queueHighWatermark() const;
  uint16_t queueLowWatermark() const;
  uint32_t normalReadQueueOldestAgeMs(uint32_t nowMs) const;
  uint8_t consecutiveNormalReadsForSubnet(uint8_t subnet) const;

 private:
  struct FailureKeyCount {
    bool used;
    uint16_t key;
    uint32_t count;
  };

  struct PendingMuteWrite {
    bool used;
    uint8_t subnet;
    uint8_t device;
    char channel;
    uint8_t value;
    uint32_t sentMs;
  };

  static const uint8_t kFailureBucketCount = 32;
  static const uint8_t kPendingMuteWriteCount = 64;

  CanTwaiTransport* transport_;
  HotWarmColdQueues queues_;
  uint32_t minSpacingUsNormal_;
  uint32_t minSpacingUsWrite_;
  uint32_t minSpacingUsScan_;
  uint32_t lastTxUs_;
  uint32_t identWindowMs_;
  uint32_t identQuietHoldMs_;
  bool enableWriteRetry_;
  uint32_t writeAckTimeoutMs_;
  uint32_t identWindowUntilMs_;
  uint32_t identQuietUntilMs_;
  uint8_t identSubnet_;
  uint8_t lastNormalReadSubnet_;
  uint8_t consecutiveNormalReadSubnetCount_;
  uint8_t currentConsecutiveReadsBySubnet_[DebugCounters::kSubnetCount];
  const DebugCli* debugCli_;
  CanTxObserverFn canTxObserver_;
  void* canTxObserverContext_;
  FailureKeyCount canTxEnqueueFailedBySubnetDevice_[kFailureBucketCount];
  FailureKeyCount canTxEnqueueFailedByObject_[kFailureBucketCount];
  PendingMuteWrite pendingMuteWrites_[kPendingMuteWriteCount];

  void noteQueueDepth(DebugCounters& counters, uint32_t nowMs) const;
  void noteEnqueueFailure(const NormalizedRequest& request, DebugCounters& counters);
  static void noteFailureKey(FailureKeyCount* buckets, uint8_t bucketCount, uint16_t key);
  static bool isIdentScanFrame(const NormalizedRequest& request);
  static bool isWriteCandidateFrame(const NormalizedRequest& request);
  static bool isCoalescingEligibleRead(const NormalizedRequest& request);
  static bool isMuteWriteFrame(const NormalizedRequest& request, char& channel, uint8_t& value);
  static bool isMuteAckFrame(const NormalizedRequest& request, char& channel, uint8_t& value);
  QueueClass classifyQueueClass(const NormalizedRequest& request) const;
  void noteCanTx(const NormalizedRequest& request, uint32_t nowMs, DebugCounters& counters);
  void expireMuteWrites(uint32_t nowMs, DebugCounters& counters);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_CAN_SCHEDULER_H
