#ifndef DB_R70_V2_GATEWAY_STATE_H
#define DB_R70_V2_GATEWAY_STATE_H

#include <stdint.h>

#include "../debug/DebugCounters.h"
#include "../debug/DebugCli.h"
#include "AmpState.h"
#include "PendingRequest.h"
#include "SegmentedAssembler.h"

namespace dbcan {
namespace r70v2 {

enum class NameSegmentCacheMissReason : uint8_t {
  None,
  Disabled,
  NoSlot,
  NoField,
  NoSegment,
  Stale,
  TtlExpired
};

class GatewayState {
 public:
  static const uint16_t kMaxAddressSlots = 512;
  static const uint8_t kPreferredMaxActiveAmps = 64;
  static const uint8_t kFallbackMaxActiveAmps = 32;
  static const uint16_t kPreferredScalarFieldsPerAmp = 160;
  static const uint16_t kFallbackScalarFieldsPerAmp = 96;
  static const uint8_t kSegmentCachePerAmp = 8;
  static const uint8_t kSegmentCacheMaxLen = 32;
  // Device/channel names plus device/channel error texts.
  static const uint8_t kNameSegmentFieldCount = 6;
  static const uint8_t kFirmwareSegmentFieldCount = 1;
  static const uint8_t kSegmentedReplyCacheFieldCount = 7;
  static const uint8_t kNameSegmentMaxSegments = 4;
  static const uint8_t kNameSegmentPayloadMaxLen = 8;
  static const uint8_t kMaxPendingRequests = 16;
  static const uint8_t kMaxRecentRequests = 16;

  GatewayState();
  ~GatewayState();

  bool begin(bool preferPsram,
             uint8_t nameSegmentMaxSegments = kNameSegmentMaxSegments,
             uint8_t nameSegmentPayloadMaxLen = kNameSegmentPayloadMaxLen);
  void reset();
  void tick(uint32_t nowMs, DebugCounters& counters);
  void updateDebugStats(DebugCounters& counters) const;

  AmpState* amp(uint8_t subnet, uint8_t device, bool createIfMissing);
  const AmpState* amp(uint8_t subnet, uint8_t device) const;

  bool shouldSuppressPending(const NormalizedRequest& request) const;
  bool shouldSuppressRecent(const NormalizedRequest& request, uint32_t nowMs) const;
  void rememberSentRequest(const NormalizedRequest& request, uint32_t nowMs);
  void addPending(const NormalizedRequest& request, uint8_t tcpNode, uint32_t nowMs, uint32_t timeoutMs);
  bool completePending(const NormalizedRequest& reply, uint8_t& tcpNode);

  bool updateCacheFromCan(const NormalizedRequest& reply,
                          uint32_t nowMs,
                          bool fromWrite,
                          DebugCounters& counters,
                          const DebugCli* debugCli = 0);
  bool updateCacheFromWrite(const NormalizedRequest& request, uint32_t nowMs, DebugCounters& counters);
  bool cacheLookup(const NormalizedRequest& request, FieldCache& cache) const;
  bool buildCacheReply(const NormalizedRequest& request,
                       uint32_t ttlMs,
                       uint32_t nowMs,
                       Frame& replyFrame,
                       uint32_t& ageMs,
                       bool& stale,
                       uint32_t nodeFreshMaxAgeMs = 0) const;
  bool buildObservedFrameCacheReply(const NormalizedRequest& request,
                                    uint32_t ttlMs,
                                    uint32_t nowMs,
                                    Frame& replyFrame,
                                    uint32_t& ageMs,
                                    bool& stale,
                                    uint32_t nodeFreshMaxAgeMs = 0) const;
  bool buildNameSegmentCacheReply(const NormalizedRequest& request,
                                  uint32_t ttlMs,
                                  uint32_t nowMs,
                                  Frame& replyFrame,
                                  uint32_t& ageMs,
                                  NameSegmentCacheMissReason& missReason,
                                  uint32_t nodeFreshMaxAgeMs = 0,
                                  bool requireCompleteSet = false) const;
  bool buildUnknown0111CacheReply(uint8_t subnet,
                                  uint8_t device,
                                  uint32_t ttlMs,
                                  uint32_t nowMs,
                                  Frame& replyFrame,
                                  uint32_t& ageMs,
                                  bool& stale,
                                  uint32_t nodeFreshMaxAgeMs = 0) const;
  void invalidateFieldCache(uint8_t subnet, uint8_t device, FieldId fieldId);
  void invalidateAllCachedValuesForNode(uint8_t subnet, uint8_t device);
  void invalidateUnknown0111Cache(uint8_t subnet, uint8_t device);
  void invalidateNameSegmentCache(uint8_t subnet,
                                  uint8_t device,
                                  FieldId fieldId,
                                  const char* reason,
                                  DebugCounters& counters,
                                  const DebugCli* debugCli = 0);
  bool usingPsram() const;
  bool fallbackMode() const;
  uint8_t activeAmpCount() const;
  const AmpState* activeAmpByIndex(uint8_t index) const;
  uint8_t maxActiveAmps() const;
  uint16_t fieldCacheCapacityPerAmp() const;
  uint8_t segmentCachePerAmp() const;
  uint16_t nameCacheSlotsCapacity() const;
  uint8_t nameSegmentMaxSegments() const;
  uint8_t nameSegmentPayloadMaxLen() const;
  uint32_t bytesAllocated() const;
  void noteTcpTxToNode(uint8_t subnet, uint8_t device, uint32_t nowMs);
  void noteCanRxForNode(uint8_t subnet, uint8_t device, uint32_t nowMs);
  void noteCacheReplyForNode(uint8_t subnet, uint8_t device, uint32_t nowMs);
  void noteCanTxPollForNode(uint8_t subnet, uint8_t device, uint32_t nowMs);
  void noteTcpRxRequestForNode(uint8_t subnet, uint8_t device, FieldId fieldId, uint32_t nowMs);
  void noteTimeoutRisk(uint8_t subnet, uint8_t device);
  void noteUnknown0111Request(uint8_t subnet, uint8_t device, uint32_t nowMs);
  void noteUnknown0111CanTx(uint8_t subnet, uint8_t device, uint32_t nowMs);
  void noteUnknown0111CanRx(uint8_t subnet, uint8_t device, const Frame& frame, uint32_t nowMs);
  void noteUnknown0111TcpTx(uint8_t subnet, uint8_t device, uint8_t value, uint32_t nowMs);

 private:
  struct NameSegmentCacheEntry {
    bool valid;
    uint8_t length;
    uint32_t lastUpdateMs;
    uint8_t payload[kNameSegmentPayloadMaxLen];
  };

  struct RecentRequest {
    bool used;
    uint32_t sentMs;
    NormalizedRequest request;
  };

  AmpState amps_[kPreferredMaxActiveAmps];
  PendingRequest pending_[kMaxPendingRequests];
  RecentRequest recent_[kMaxRecentRequests];
  uint8_t addressToAmpIndex_[kMaxAddressSlots];
  FieldCache* fieldCachePool_;
  void* segmentSlotPool_;
  NameSegmentCacheEntry* nameSegmentCachePool_;
  uint16_t fieldCacheCapacityPerAmp_;
  uint8_t maxActiveAmps_;
  uint8_t activeAmpCount_;
  bool usingPsram_;
  bool fallbackMode_;
  uint32_t bytesAllocated_;
  uint8_t recentHead_;
  uint8_t nameSegmentMaxSegments_;
  uint8_t nameSegmentPayloadMaxLen_;
  SegmentedAssembler segmented_;

  void releaseStorage();
  static uint16_t addressIndex(uint8_t subnet, uint8_t device);
  static int8_t nameFieldIndex(FieldId fieldId);
  uint8_t ampIndex(uint8_t subnet, uint8_t device) const;
  NameSegmentCacheEntry* nameSegmentEntry(uint8_t ampIndex, FieldId fieldId, uint8_t segmentNumber);
  const NameSegmentCacheEntry* nameSegmentEntry(uint8_t ampIndex,
                                                FieldId fieldId,
                                                uint8_t segmentNumber) const;
  void clearNameSegmentEntries();
  void clearNameSegmentEntriesForAmp(uint8_t ampIndex);
  void clearNameSegmentFieldForAmp(uint8_t ampIndex, FieldId fieldId);
  bool storeNameSegmentFromCan(const NormalizedRequest& reply,
                               uint32_t nowMs,
                               DebugCounters& counters);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_GATEWAY_STATE_H
