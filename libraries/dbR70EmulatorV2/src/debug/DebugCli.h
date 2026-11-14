#ifndef DB_R70_V2_DEBUG_CLI_H
#define DB_R70_V2_DEBUG_CLI_H

#include <Print.h>
#include <stddef.h>
#include <stdint.h>

#include "../DbCanFrame.h"
#include "../protocol/DbCanMapping.h"
#include "../protocol/R70Frame.h"
#include "DebugCounters.h"

namespace dbcan {
namespace r70v2 {

class DebugCli {
 public:
  DebugCli();

  void begin(Print& out);
  void setVerboseFrameLogEnabled(bool enabled);
  void logText(const char* text) const;
  void logHex(const char* prefix, const uint8_t* bytes, size_t length) const;
  void logTcpFrame(const char* prefix,
                   const uint8_t* bytes,
                   uint8_t length,
                   bool hasCrcStatus = false,
                   bool crcOk = false) const;
  void logCanFrame(const char* prefix, const Frame& frame, const NormalizedRequest* request) const;
  void logTcpCanTx(const NormalizedRequest& request) const;
  void logTcpCanTxFail(const NormalizedRequest& request, const char* reason) const;
  void logTcpCanRxForward(const NormalizedRequest& request) const;
  void logTcpTxWrappedCan(const Frame& frame, const uint8_t* bytes, uint8_t length) const;
  void logChecksumMismatch(const TcpEnvelope& envelope) const;
  void logTcpBackpressureOn(uint16_t queueDepth, uint16_t highWater) const;
  void logTcpBackpressureOff(uint16_t queueDepth, uint16_t lowWater, uint32_t activeMs) const;
  void logTcpBackpressureLimited(const char* reason) const;
  void logReadCoalesced(const NormalizedRequest& request, uint32_t count) const;
  void logCacheReply(const NormalizedRequest& request,
                     const Frame& frame,
                     uint32_t ageMs,
                     uint32_t ttlMs) const;
  void logNameAssembled(uint8_t subnet,
                        uint8_t device,
                        FieldId fieldId,
                        const uint8_t* bytes,
                        uint8_t length) const;
  void logNameSegmentCacheReply(uint8_t subnet,
                                uint8_t device,
                                FieldId fieldId,
                                uint8_t segmentNumber,
                                const Frame& frame,
                                uint32_t ageMs) const;
  void logFirmwareSegmentCacheReply(uint8_t subnet,
                                    uint8_t device,
                                    uint8_t segmentNumber,
                                    const Frame& frame,
                                    uint32_t ageMs) const;
  void logStableSetupCacheReply(uint8_t subnet,
                                uint8_t device,
                                FieldId fieldId,
                                const Frame& frame,
                                uint32_t ageMs) const;
  void logStableSetupCacheInvalidate(uint8_t subnet,
                                     uint8_t device,
                                     FieldId fieldId,
                                     const char* reason) const;
  void logTopCanTxObject(uint8_t rank,
                         uint16_t objectKey,
                         const char* label,
                         uint32_t tcpReadRequests,
                         uint32_t canTxForwarded,
                         uint32_t cacheReplies,
                         uint32_t cacheStale,
                         uint32_t cacheMiss,
                         uint32_t coalescedDuplicates,
                         uint32_t queueFull) const;
  void logNameSegmentCacheMiss(uint8_t subnet,
                               uint8_t device,
                               FieldId fieldId,
                               uint8_t segmentNumber,
                               const char* reason) const;
  void logNameCacheInvalidate(uint8_t subnet,
                              uint8_t device,
                              FieldId fieldId,
                              const char* reason) const;
  void logNormalQueueLatency(uint32_t oldestAgeMs, uint16_t depth, bool critical) const;
  void logWriteQueueFull(const Frame& frame) const;
  void logIdentQueued(uint8_t subnet, const Frame& frame) const;
  void logMuteWriteQueued(uint8_t subnet,
                          uint8_t device,
                          char channel,
                          uint8_t value,
                          const Frame& frame) const;
  void logMuteWriteTx(uint8_t subnet,
                      uint8_t device,
                      char channel,
                      uint8_t value,
                      const Frame& frame) const;
  void logMuteWriteAck(uint8_t subnet,
                       uint8_t device,
                       char channel,
                       uint8_t value,
                       const Frame& frame) const;
  void logMuteWriteTimeout(uint8_t subnet,
                           uint8_t device,
                           char channel,
                           uint8_t value,
                           uint32_t ageMs) const;
  void logIdentTx(uint8_t subnet, const Frame& frame) const;
  void logIdentRx(uint8_t subnet, uint8_t device, const Frame& frame) const;
  void logIdentSummary(uint8_t subnet,
                       uint32_t rxCount,
                       uint32_t rx441,
                       uint32_t rx442) const;
  void logNodeReplyAgeTop(uint8_t subnet,
                          uint8_t device,
                          uint8_t rank,
                          uint32_t nowMs,
                          uint32_t lastTcpTxAgeMs,
                          uint32_t lastRealCanRxAgeMs,
                          uint32_t lastCacheReplyAgeMs,
                          uint32_t lastCanTxPollAgeMs,
                          uint32_t tcpTxCount,
                          uint32_t realCanRxCount,
                          uint32_t cacheReplyCount,
                          uint32_t canTxPollCount) const;
  void logNodeTimeoutRisk(uint8_t subnet,
                          uint8_t device,
                          uint32_t nowMs,
                          uint32_t lastTcpTxAgeMs,
                          uint32_t lastRealCanRxAgeMs,
                          uint32_t lastCacheReplyAgeMs,
                          uint32_t lastCanTxPollAgeMs) const;
  void logNodeRequestWithoutReply(uint8_t subnet,
                                  uint8_t device,
                                  uint32_t nowMs,
                                  FieldId fieldId,
                                  uint32_t ageMs) const;
  void logStateSummary(const DebugCounters& counters,
                       uint32_t nowMs,
                       uint32_t canTxFramesDelta,
                       uint32_t canRxFramesDelta,
                       uint32_t tcpRxFramesDelta,
                       uint32_t tcpTxFramesDelta,
                       uint32_t tcpBackpressureActiveMs,
                       uint32_t currentSpacingUs) const;
  void logLongRunPerfSummary(const DebugCounters& counters, uint32_t nowMs, uint32_t durationSeconds) const;
  void printCounters(const DebugCounters& counters) const;
  void printBootstrapDiagnostics(const DebugCounters& counters) const;

 private:
  Print* out_;
  bool verboseFrameLogEnabled_;
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_DEBUG_CLI_H
