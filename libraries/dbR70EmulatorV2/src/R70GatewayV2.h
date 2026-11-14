#ifndef DB_R70_GATEWAY_V2_H
#define DB_R70_GATEWAY_V2_H

#include <Arduino.h>
#include <IPAddress.h>
#include <Print.h>
#include <Stream.h>
#include <stdint.h>

#include "debug/DebugCounters.h"
#include "debug/DebugCli.h"
#include "protocol/DbCanMapping.h"
#include "scheduler/CanScheduler.h"
#include "scheduler/PollPlanner.h"
#include "state/GatewayState.h"
#include "transport/CanTwaiTransport.h"
#include "transport/TcpR1Server.h"
#include "transport/UdpDiscovery.h"

namespace dbcan {
namespace r70v2 {

#ifndef R70_BASELINE_0_2_SAFE
#define R70_BASELINE_0_2_SAFE 1
#endif

#ifndef R70_CACHE_CONFIRMED_READS_EXPERIMENT
#define R70_CACHE_CONFIRMED_READS_EXPERIMENT 1
#endif

#ifndef R70_EXPERIMENTAL_DEBUG_CODE
#define R70_EXPERIMENTAL_DEBUG_CODE 0
#endif

#ifndef ENABLE_GENTLE_STATUS_SHAPING
#define ENABLE_GENTLE_STATUS_SHAPING 1
#endif

#ifndef ENABLE_BOOTSTRAP_PROTECTION
#define ENABLE_BOOTSTRAP_PROTECTION 0
#endif

#ifndef ENABLE_BOOTSTRAP_RETRY
#define ENABLE_BOOTSTRAP_RETRY 0
#endif

#ifndef ENABLE_SEGMENTED_DAMPENING
#define ENABLE_SEGMENTED_DAMPENING 0
#endif

#ifndef ENABLE_FANOUT_ATTACH
#define ENABLE_FANOUT_ATTACH 1
#endif

#ifndef ENABLE_POST_BOOTSTRAP_STATUS_CACHE
#define ENABLE_POST_BOOTSTRAP_STATUS_CACHE 1
#endif

#ifndef STATEFUL_CACHE_WARMUP_MS
#define STATEFUL_CACHE_WARMUP_MS 20000u
#endif

#ifndef AMP_COUNT_STABLE_REQUIRED_MS
#define AMP_COUNT_STABLE_REQUIRED_MS 5000u
#endif

#if defined(R70_PACING_FAST)
static const uint32_t kDefaultCanTxMinSpacingUsNormal = CanScheduler::kPacingFastNormalUs;
static const uint32_t kDefaultCanTxMinSpacingUsWrite = CanScheduler::kPacingFastWriteUs;
static const uint32_t kDefaultCanTxMinSpacingUsScan = CanScheduler::kPacingFastScanUs;
#elif defined(R70_PACING_VERY_SAFE)
static const uint32_t kDefaultCanTxMinSpacingUsNormal = CanScheduler::kPacingVerySafeNormalUs;
static const uint32_t kDefaultCanTxMinSpacingUsWrite = CanScheduler::kPacingVerySafeWriteUs;
static const uint32_t kDefaultCanTxMinSpacingUsScan = CanScheduler::kPacingVerySafeScanUs;
#else
static const uint32_t kDefaultCanTxMinSpacingUsNormal = CanScheduler::kPacingSafeNormalUs;
static const uint32_t kDefaultCanTxMinSpacingUsWrite = CanScheduler::kPacingSafeWriteUs;
static const uint32_t kDefaultCanTxMinSpacingUsScan = CanScheduler::kPacingSafeScanUs;
#endif

struct GatewayIdentity {
  IPAddress ip;
  uint8_t mac[6];
  char product[23];
  char hostname[29];
};

enum class GatewayProfile : uint8_t {
  PROFILE_CUSTOM = 0,
  PROFILE_GATE31_STABLE = 1,
};

struct GatewayConfig {
  GatewayProfile profile;
  gpio_num_t canTxPin;
  gpio_num_t canRxPin;
  uint32_t canBitrate;
  bool canListenOnly;
  uint16_t udpPort;
  uint16_t tcpPort;
  uint32_t canTxGapMs;
  uint32_t canTxMinSpacingUs;
  uint32_t canTxMinSpacingUsNormal;
  uint32_t canTxMinSpacingUsWrite;
  uint32_t canTxMinSpacingUsScan;
  uint32_t identWindowMs;
  uint32_t identQuietHoldMs;
  bool enableWriteRetry;
  uint32_t writeAckTimeoutMs;
  bool enableEmergencyTcpBackpressure;
  uint32_t maxTcpBackpressureBurstMs;
  uint32_t maxTcpBackpressureTotalPerSecondMs;
  bool enableConfirmedReadCache;
  uint32_t cacheTtlDynamicMs;
  bool enableConfigScalarTtlExperiment;
  uint32_t configScalarTtlMs;
  uint32_t configScalarTtlExperimentMs;
  uint32_t configScalarTtlPreset;
  uint32_t cacheTtlNamesMs;
  uint32_t nameSegmentTtlMs;
  uint8_t nameSegmentMaxSegments;
  uint8_t nameSegmentPayloadMaxLen;
  bool enableFirmwareSegmentCache;
  uint32_t firmwareSegmentTtlMs;
  uint8_t firmwareSegmentMaxSegments;
  bool enableUnknown0111ExactCache;
  uint32_t unknown0111TtlMs;
  bool enableUnknown0111ExactCacheDebug;
  uint32_t watchdogStatusTtlMs;
  uint32_t ispTtlMs;
  bool enableCriticalStatusCache;
  uint32_t criticalStatusTtlMs;
  uint32_t criticalStatusTtlExperimentMs;
  uint32_t criticalStatusTtlPreset;
  bool enableCriticalStatusTtlExperiment;
  bool enableStableSetupCache;
  bool enableSpeakerIdCache;
  bool enableInputModeCache;
  bool enableDataLinkPresetCache;
  uint32_t speakerIdTtlMs;
  uint32_t inputModeTtlMs;
  uint32_t datalinkPresetTtlMs;
  bool enableSmallStatusCacheExperiment;
  bool enableExpandedStatusCache;
  bool enableErrorStatusCache;
  bool enableRemHoldCache;
  bool enableNameSegmentCacheReplies;
  bool enableVerboseFrameLog;
  bool enableTimeoutCorrelation;
  bool enableAutoGroupTimeoutSnapshot;
  bool enablePerNodeRecentRequestRing;
  bool enableLowRateSummary;
  bool enableSoftInflightCollapse;
  uint32_t softInflightCollapseTimeoutMs;
  bool enableGentleStatusShaping;
  GatewayIdentity identity;
};

class R70GatewayV2 {
 public:
  R70GatewayV2();

  bool begin(const GatewayConfig& config, Print& log = Serial);
  void poll();
  void restartNetworkServices();
  bool enqueueLocalPrimaryMasterFrame(const Frame& frame, const char*& failureReason);
  bool sendDirectPrimaryMasterFrame(const Frame& frame, uint32_t timeoutMs, const char*& failureReason);

  const DebugCounters& counters() const;
  DebugCounters& mutableCounters();
  bool tcpControlConnected() const;
  void printCounters(Print& out) const;
  void printBootstrapDiagnostics(Print& out) const;

  GatewayState& state();
  const GatewayConfig& config() const;

 private:
  static const uint8_t kObjectStatCapacity = 64;
  static const uint8_t kSemanticCandidateCapacity = 32;
  static const uint32_t kSemanticCandidateLogIntervalMs = 1000;
  static const uint32_t kSemanticCandidateSummaryIntervalMs = 30000;
  static const uint32_t kPresenceRoundWaitMs = 1600;
  static const uint32_t kPresenceRetryDecisionMaxMs = 5000;
  static const uint32_t kPresenceRecentCanMaxAgeMs = 5000;
  static const uint16_t kPresenceRetryMaxQueueDepth = 32;
  static const uint8_t kPresenceRetryMaxBusUtilPercent = 80;
  static const uint16_t kRecentRequestTraceCapacity = 512;
  // Compact, completed R1 reads. The detailed request ring turns over in about
  // 1.3 s, while R1 may report a timeout more than two seconds later. Keep the
  // compact history in PSRAM long enough to retain the requests that actually
  // preceded the external R1 timeout. This is diagnostics only; it does not
  // alter caching, scheduling, CAN traffic, or replies to R1.
  static const uint16_t kAliveDiagnosticTraceCapacity = 4096;
  static const uint16_t kTimeoutTraceLookbackMs = 3000;
  static const uint16_t kSlowFieldLookbackMs = 10000;
  static const uint16_t kWatchdogWarnAgeMs = 1000;
  static const uint16_t kWatchdogCriticalAgeMs = 1500;
  static const uint16_t kAutoSnapshotRateLimitMs = 10000;
  static const uint8_t kCliBufferCapacity = 96;
  static const uint32_t kBootstrapRetryStartMs = 10000;
  static const uint32_t kPowerOnBootstrapRetryStartMs = 1000;
  static const uint8_t kPowerOnBootstrapRetriesPerSecond = 32;
  static const uint32_t kBootstrapRetryBurstMs = 500;
  static const uint32_t kBootstrapIncompleteLogThresholdMs = 10000;
  static const uint32_t kBootstrapProtectionMs = 15000;
  static const uint32_t kBootstrapSteadyDelayMs = 5000;
  static const uint32_t kStartupFreshnessProtectionMs = 60000;
  static const uint32_t kRuntimeBaselineSummaryMs = 10000;
  static const uint32_t kRuntimeStableRequiredMs = 30000;
  static const uint32_t kR1FreshnessWarnMs = 2000;
  static const uint32_t kR1FreshnessCriticalMs = 5000;
  static const uint32_t kStartupFreshnessPriorityMs = 1200;
  static const uint8_t kStartupBlockerFamilyCount = 6;

  enum class TraceSource : uint8_t {
    None = 0,
    Cache = 1,
    RealCan = 2,
    Mirror = 3,
  };

  enum class TraceMirrorDecision : uint8_t {
    None = 0,
    MirrorServed = 1,
    ForcedCanMissing = 2,
    ForcedCanStale = 3,
    ForcedCanNotSafe = 4,
    ForcedCanWriteInvalidated = 5,
  };

  enum class TraceBackgroundRefreshState : uint8_t {
    None = 0,
    Sent = 1,
    SkippedTcpPressure = 2,
    SkippedCanPending = 3,
    SkippedServiceGap = 4,
    SkippedRateLimit = 5,
    SkippedDiagnosticHold = 6,
  };

  enum class TracePendingState : uint8_t {
    Created = 0,
    WaitingCan = 1,
    CanComplete = 2,
    PartialComplete = 3,
    ReplySent = 4,
    TimeoutCleanup = 5,
  };

  enum class TraceReleaseReason : uint8_t {
    None = 0,
    MirrorReply = 1,
    CacheReply = 2,
    RealCanReply = 3,
    AgedOutNoReply = 4,
    TcpSendFailed = 5,
  };

  enum class SlowTraceCategory : uint8_t {
    Unknown = 0,
    Unknown0111 = 1,
    NameSegment = 2,
    FirmwareSegment = 3,
    StableSetup = 4,
    SegmentedRead = 5,
    UnstructuredRead = 6,
  };

  enum class ExpensiveObjectGroup : uint8_t {
    None = 0,
    Object0100 = 1,
    Object2400_4400 = 2,
    Object4205_4206 = 3,
    NameSegment = 4,
  };

  enum class ShapingSimGroup : uint8_t {
    None = 0,
    Object2400_4400 = 1,
    Object4205_4206 = 2,
    Object0100 = 3,
    Object2205_2206 = 4,
    Object241D_241E_441D_441E = 5,
    Object0500_2600_4600 = 6,
  };

  enum class RegressionFeature : uint8_t {
    GentleStatusShaping = 0,
    BootstrapProtection = 1,
    BootstrapRetry = 2,
    SegmentedDampening = 3,
    FanoutAttach = 4,
    PostBootstrapStatusCache = 5,
  };

  enum class StatefulStatusCacheGroup : uint8_t {
    None = 0,
    Object241D_241E_441D_441E = 1,
    Object0500_2600_4600 = 2,
    Object2400_4400 = 3,
    Object0300 = 4,
  };

  enum class SemanticMirrorGroup : uint8_t {
    None = 0,
    Object0100_010D_0111 = 1,
    Object0205_0206 = 2,
    Object0300 = 3,
    Object0400_041D_041E = 4,
    Object0500_0600 = 5,
    Object2400_2600 = 6,
    Object4205_4206 = 7,
    Object4400_4600 = 8,
    Object241D_241E_441D_441E = 9,
  };

  static const uint8_t kSlowObjectBucketCapacity = 96;
  static const uint8_t kSlowNodeBucketCapacity = 64;
  static const uint8_t kExpensiveObjectGroupCount = 4;
  static const uint8_t kShapingSimGroupCount = 6;
  static const uint8_t kSoftInflightCollapseCapacity = 48;
  static const uint8_t kGentleStatusShapingCapacity = 64;
  static const uint8_t kDeferredSegmentedCapacity = 24;
  static const uint8_t kDeferredIdentCapacity = 8;
  static const uint8_t kActiveShapingCadenceGroupCount = 2;
  static const uint16_t kStatefulStatusCacheCapacity = 192;
  static const uint8_t kStatefulStatusCacheGroupCount = 4;
  // 32 amplifiers need enough independent entries for the wider decoded-read
  // mirror without evicting the established alive/status objects.
  static const uint16_t kSemanticMirrorCapacity = 768;
  static const uint8_t kSemanticMirrorGroupCount = 9;

  struct ObjectStat {
    bool used;
    uint16_t objectKey;
    FieldId fieldId;
    uint32_t tcpReadRequests;
    uint32_t canTxForwarded;
    uint32_t cacheReplies;
    uint32_t cacheStale;
    uint32_t cacheMiss;
    uint32_t coalescedDuplicates;
    uint32_t queueFull;
  };

  struct RequestTrace {
    bool used;
    uint32_t requestId;
    uint8_t subnet;
    uint8_t device;
    uint8_t channel;
    uint16_t objectKey;
    FieldId fieldId;
    uint8_t length;
    uint8_t data[8];
    uint32_t timestampMs;
    uint32_t canTxTimestampMs;
    uint32_t canRxTimestampMs;
    uint32_t replyTimestampMs;
    uint32_t cleanupTimestampMs;
    uint16_t pendingTcpRepliesAtRequest;
    uint16_t normalQueueDepthAtRequest;
    uint16_t canTxQueueDepthAtRequest;
    uint16_t latencyMs;
    uint16_t queueWaitMs;
    uint16_t canRoundtripMs;
    uint16_t postReplyMs;
    uint8_t duplicatePendingAtRequest;
    uint8_t waiterFanoutCount;
    uint8_t outstandingObjectCount;
    uint8_t outstandingSegmentCount;
    uint8_t flags;
    TraceSource source;
    TraceMirrorDecision mirrorDecision;
    TraceBackgroundRefreshState backgroundRefreshState;
    TracePendingState pendingState;
    TraceReleaseReason releaseReason;
    SlowTraceCategory category;
    ExpensiveObjectGroup expensiveGroup;
  };

  struct AliveDiagnosticTrace {
    uint32_t timestampMs;
    uint32_t replyTimestampMs;
    uint16_t objectKey;
    uint16_t latencyMs;
    uint8_t subnet;
    uint8_t device;
    TraceSource source;
    uint8_t channel;
  };

  struct SlowObjectBucket {
    bool used;
    uint16_t objectKey;
    uint32_t lt100Count;
    uint32_t bucket100To250Count;
    uint32_t bucket250To500Count;
    uint32_t bucket500To1000Count;
    uint32_t gt1000Count;
    uint32_t over500Count;
    uint32_t over1000Count;
    uint16_t maxLatencyMs;
  };

  struct SlowNodeBucket {
    bool used;
    uint8_t subnet;
    uint8_t device;
    uint32_t over500Count;
    uint32_t over1000Count;
    uint16_t maxLatencyMs;
  };

  struct SlowSubnetBucket {
    uint32_t over500Count;
    uint32_t over1000Count;
    uint16_t maxLatencyMs;
  };

  struct SlowObjectFairnessState {
    bool used;
    uint16_t objectKey;
    uint32_t lastTxMs;
  };

  struct ExpensiveObjectWindow {
    ExpensiveObjectGroup group;
    uint32_t tcpReadCount;
    uint32_t canTxCount;
    uint32_t canRxCount;
    uint32_t duplicateInflightCount;
    uint32_t burstRepeatCount;
    uint32_t segmentedActiveRepeatCount;
    uint16_t duplicatePendingMax;
    uint32_t coalescedCount;
    uint32_t over500Count;
    uint32_t over1000Count;
    uint16_t maxCanRoundtripMs;
    uint16_t requestPer100msBins[100];
    uint16_t requestPer1sBins[10];
    uint16_t peakRequestPer100ms;
    uint16_t peakRequestPer1s;
  };

  struct ShapingSimGroupWindow {
    ShapingSimGroup group;
    uint32_t actualCanTxCount;
    uint32_t simulatedCanTxCount;
    uint32_t simulatedSavedTx;
    uint32_t shapedSavedTx;
    uint32_t softCollapseSavedTx;
    uint32_t shapedRequests;
    uint32_t attachedToRecentCount;
    uint32_t deferredRequests;
    uint32_t reusedRecentResponseCount;
    uint32_t bypassDueToAgeCount;
    uint32_t bypassFirstRequestCount;
    uint32_t bypassCacheReadyCount;
    uint32_t bypassDisabledCount;
    uint32_t bypassIneligibleCount;
    uint32_t bypassNoCacheCount;
    uint32_t bypassCacheTooOldCount;
    uint32_t bypassNoInflightCount;
    uint32_t bypassReleaseForcedCount;
    uint32_t bypassWriteOrControlCount;
    uint32_t shapedDispatchCount;
    uint32_t shapedDispatchAddedWaitSumMs;
    uint16_t shapedDispatchAddedWaitMaxMs;
    uint16_t maxExtraWaitMs;
    uint16_t affectedNodeCount;
    uint32_t lastSimulatedTxMsByNode[GatewayState::kMaxAddressSlots];
    uint8_t affectedNodeFlags[GatewayState::kMaxAddressSlots];
  };

  struct SoftInflightCollapseEntry {
    bool used;
    uint8_t subnet;
    uint8_t device;
    uint8_t channel;
    uint16_t objectKey;
    FieldId fieldId;
    TelegramType telegramType;
    bool hasRecord;
    uint8_t recordNumber;
    bool hasSegment;
    uint8_t segmentNumber;
    uint8_t length;
    uint8_t data[8];
    uint32_t startedMs;
    uint32_t expiresMs;
    uint8_t waiterCount;
    ShapingSimGroup group;
  };

  struct GentleStatusShapingEntry {
    bool used;
    bool pendingActive;
    bool lastReplyValid;
    uint8_t subnet;
    uint8_t device;
    uint8_t channel;
    uint16_t objectKey;
    FieldId fieldId;
    TelegramType telegramType;
    bool hasRecord;
    uint8_t recordNumber;
    bool hasSegment;
    uint8_t segmentNumber;
    uint8_t length;
    uint8_t data[8];
    uint32_t lastCanTxMs;
    uint32_t lastReplyMs;
    uint32_t deferredStartedMs;
    uint32_t deferredReleaseMs;
    uint32_t oldestWaiterAttachedMs;
    uint8_t waiterCount;
    ShapingSimGroup group;
    Frame lastReplyFrame;
  };

  struct DeferredSegmentedRequestEntry {
    bool used;
    NormalizedRequest request;
    uint32_t firstDeferredMs;
    uint32_t releaseAfterMs;
    uint8_t duplicateCount;
  };

  struct DeferredIdentRequestEntry {
    bool used;
    NormalizedRequest request;
    uint32_t firstDeferredMs;
    uint32_t releaseAfterMs;
    uint8_t duplicateCount;
  };

  struct ActiveShapingCadenceState {
    uint32_t windowStartedMs;
    uint32_t lastCanTxMs;
    uint32_t lastReplyMs;
    bool pendingActive;
  };

  struct StatefulStatusCacheEntry {
    bool used;
    bool valid;
    bool inflight;
    uint8_t subnet;
    uint8_t device;
    uint8_t channel;
    uint16_t objectKey;
    FieldId fieldId;
    uint8_t requestLength;
    uint8_t requestData[8];
    Frame replyFrame;
    uint32_t lastUpdateMs;
    uint32_t inflightExpiresMs;
    uint8_t waiterCount;
  };

  struct StatefulStatusCacheGroupWindow {
    StatefulStatusCacheGroup group;
    uint32_t cacheHits;
    uint32_t cacheMisses;
    uint32_t canReadsPrevented;
    uint32_t forcedCanReads;
    uint32_t inflightJoined;
    uint16_t cacheAgeMaxMs;
  };

  struct SemanticMirrorEntry {
    bool used;
    bool learned;
    bool refreshInflight;
    bool invalidatedByWrite;
    bool hasRequestTemplate;
    uint8_t subnet;
    uint8_t device;
    uint8_t channel;
    uint16_t objectKey;
    FieldId fieldId;
    TelegramType telegramType;
    uint8_t requestLength;
    uint8_t requestData[8];
    Frame replyFrame;
    uint32_t lastUpdateMs;
    uint32_t lastServedMs;
    uint32_t refreshEnqueuedMs;
    uint32_t staleServedCountWindow;
    uint32_t lastRefreshBlockedMs;
    uint8_t lastRefreshBlockedReason;
  };

  struct SemanticMirrorGroupWindow {
    SemanticMirrorGroup group;
    uint32_t hits;
    uint32_t misses;
    uint32_t refreshSent;
    uint32_t proactiveRefreshSent;
    uint32_t priorityRefreshSent;
    uint32_t protectedRefreshSent;
    uint32_t protectedRefreshDelayed;
    uint16_t maxAgeMs;
  };

  struct SemanticMirrorWindow {
    uint32_t learnAttempts;
    uint32_t learnSkips;
    uint32_t learnSuccesses;
    uint32_t cacheHits;
    uint32_t cacheMisses;
    uint32_t staleForcedCanReads;
    uint32_t writeInvalidations;
    uint32_t backgroundRefreshSent;
    uint32_t backgroundRefreshSkipped;
    uint32_t backgroundRefreshRateLimited;
    uint32_t proactiveRefreshSent;
    uint32_t priorityRefreshSent;
    uint32_t refreshSkippedPressure;
    uint32_t protectedRefreshSent;
    uint32_t protectedRefreshDelayed;
    uint32_t protectedRefreshHardlimitSkipped;
    uint32_t staleServedOver1s;
    uint32_t staleServedOver1500ms;
    uint32_t refreshBlockedByTcpPressure;
    uint32_t refreshBlockedByQueue;
    uint16_t protectedRefreshAgeMax;
    uint32_t protectedRefreshBypassedTcp;
    uint32_t protectedRefreshBypassedShaping;
    uint32_t protectedRefreshOverdue;
    uint32_t refreshIdleOnlyCount;
    uint32_t refreshSkippedDueToFifoPressure;
  };

  // Fixed-size discovery table. It intentionally stores only a payload
  // fingerprint, never whole histories; the host-side capture owns those.
  struct SemanticCandidateEntry {
    bool used;
    uint8_t channel;
    uint8_t lastSubnet;
    uint8_t lastDevice;
    uint16_t objectKey;
    uint32_t payloadHash;
    uint32_t firstSeenMs;
    uint32_t lastSeenMs;
    uint32_t lastLogMs;
    uint32_t seenCount;
    uint16_t changeCount;
    uint16_t r1RequestedCount;
  };

  struct PresenceRoundState {
    bool active;
    bool retryQueued;
    bool retrySent;
    Frame scanFrame;
    uint64_t expectedMask;
    uint64_t seenMask;
    uint64_t missingAtRetryMask;
    uint32_t startedMs;
    uint32_t evaluateAfterMs;
    uint32_t retrySentMs;
  };

  struct SlowCanWindow {
    bool initialized;
    uint32_t startedMs;
    uint32_t tx10sCount;
    uint32_t rx10sCount;
    uint32_t over500Count;
    uint32_t over1000Count;
    uint32_t identSlowOver500Count;
    uint32_t identSlowOver1000Count;
    uint32_t identTxCount;
    uint64_t estimatedBusBitsTotal;
    uint16_t peakTxFramesPer100ms;
    uint16_t peakRxFramesPer100ms;
    uint16_t peakTxFramesPer1s;
    uint16_t peakRxFramesPer1s;
    uint32_t peakBusBitsPer100ms;
    uint16_t worstLatencyMs;
    uint16_t worstQueueWaitMs;
    uint16_t worstCanRoundtripMs;
    uint16_t worstPostReplyMs;
    uint8_t worstSubnet;
    uint8_t worstDevice;
    uint8_t worstChannel;
    uint16_t worstObjectKey;
    SlowTraceCategory worstCategory;
    uint16_t maxConsecutiveNodeRequests;
    uint16_t maxConsecutiveObjectRequests;
    uint32_t maxNodeStarvationGapMs;
    uint32_t maxObjectStarvationGapMs;
    SlowObjectBucket objectBuckets[kSlowObjectBucketCapacity];
    SlowNodeBucket nodeBuckets[kSlowNodeBucketCapacity];
    SlowSubnetBucket subnetBuckets[DebugCounters::kSubnetCount];
    uint32_t txPerSubnet[DebugCounters::kSubnetCount];
    uint32_t rxPerSubnet[DebugCounters::kSubnetCount];
    uint32_t normalReadTxPerSubnet[DebugCounters::kSubnetCount];
    uint16_t txFramesPer100msBins[100];
    uint16_t rxFramesPer100msBins[100];
    uint32_t estimatedBusBitsPer100msBins[100];
    uint16_t txFramesPer1sBins[10];
    uint16_t rxFramesPer1sBins[10];
    uint64_t canResponseLatencySumMs;
    uint32_t canResponseCount;
    uint32_t duplicateInflightReadCount;
    uint32_t burstRepeatReadCount;
    uint32_t segmentedActiveRepeatCount;
    uint32_t inflightCollapsedCount;
    uint32_t inflightTimeoutReleaseCount;
    uint32_t collapseSavedCanTx;
    uint16_t inflightWaitersMax;
    uint32_t shapedDeferredCount;
    uint32_t shapedSentCount;
    uint32_t shapedBypassDueAgeCount;
    uint32_t shapingSavedCanTx;
    uint32_t shapingExtraWaitSumMs;
    uint32_t shapingExtraWaitCount;
    uint16_t shapingExtraWaitMaxMs;
    uint32_t shapingAddedWaitSumMs;
    uint32_t shapingAddedWaitCount;
    uint16_t shapingAddedWaitMaxMs;
    uint32_t shapingReusedRecentResponseCount;
    uint32_t shapingAttachedToInflightCount;
    uint32_t shapingShapedRequestsCount;
    uint32_t r1MissingDataEvents;
    uint32_t shapingCandidatesCount;
    uint32_t shapingAllowedImmediateCount;
    uint32_t shapingReleaseSentCount;
    uint8_t shapingDisabledReasonCode;
    uint32_t segmentedGuardCandidates;
    uint32_t segmentedGuardImmediate;
    uint32_t segmentedGuardDeferred;
    uint32_t segmentedGuardReleased;
    uint32_t segmentedGuardForcedRelease;
    uint32_t segmentedGuardCoalesced;
    uint32_t segmentedGuardBlockedByUtil;
    uint32_t segmentedGuardBlockedByQueue;
    uint32_t segmentedGuardBlockedBySlow500;
    uint32_t segmentedGuardWaitSumMs;
    uint32_t segmentedGuardWaitCount;
    uint16_t segmentedGuardWaitMaxMs;
    uint8_t segmentedGuardPendingMax;
    uint32_t identGuardCandidates;
    uint32_t identGuardImmediate;
    uint32_t identGuardDeferred;
    uint32_t identGuardReleased;
    uint32_t identGuardForcedRelease;
    uint32_t identGuardCoalesced;
    uint32_t identGuardBlockedByUtil;
    uint32_t identGuardBlockedByQueue;
    uint32_t identGuardBlockedBySlow500;
    uint32_t identGuardWaitSumMs;
    uint32_t identGuardWaitCount;
    uint16_t identGuardWaitMaxMs;
    uint8_t identGuardPendingMax;
    uint32_t subnetReplyGapMaxMs[DebugCounters::kSubnetCount];
    uint32_t worstDeviceReplyGapMs;
    uint8_t worstDeviceReplyGapSubnet;
    uint8_t worstDeviceReplyGapDevice;
    uint32_t missedServiceWindowCount;
    ExpensiveObjectWindow expensiveGroups[kExpensiveObjectGroupCount];
    ShapingSimGroupWindow shapingGroups[kShapingSimGroupCount];
  };

  GatewayConfig config_;
  bool started_;
  DebugCounters counters_;
  DebugCli debugCli_;
  GatewayState state_;
  UdpDiscovery discovery_;
  TcpR1Server tcpServer_;
  CanTwaiTransport canTransport_;
  CanScheduler scheduler_;
  PollPlanner pollPlanner_;
  Print* log_;
  Stream* cliStream_;
  char cliBuffer_[kCliBufferCapacity];
  uint8_t cliBufferLength_;
  bool tcpBackpressureActive_;
  bool tcpBackpressureBurstLimited_;
  bool tcpBackpressureTotalLimited_;
  uint32_t tcpBackpressureStartMs_;
  uint32_t tcpBackpressureWindowStartMs_;
  uint32_t tcpBackpressureWindowUsedMs_;
  uint16_t canTxQueueHighWater_;
  uint16_t canTxQueueLowWater_;
  uint32_t lastStateSummaryMs_;
  uint32_t lastSummaryCanTxFrames_;
  uint32_t lastSummaryCanRxFrames_;
  uint32_t lastSummaryTcpRxFrames_;
  uint32_t lastSummaryTcpTxFrames_;
  uint32_t lastSummaryMuteWriteTxCount_;
  uint32_t lastSummaryMuteWriteAckCount_;
  uint32_t lastSummaryMuteWriteTimeoutCount_;
  uint32_t lastSummaryIdentRxCountBySubnet_[DebugCounters::kSubnetCount];
  uint32_t lastSummaryRx441_;
  uint32_t lastSummaryRx442_;
  uint32_t lastLongRunSummaryMs_;
  uint32_t lastLongRunQueueFullTotal_;
  uint32_t lastLongRunTcpBackpressureActiveMs_;
  uint32_t lastLongRunCacheRepliesSent_;
  uint32_t lastLongRunCanTxFrames_;
  uint32_t lastLongRunCanRxFrames_;
  uint32_t lastRuntimeBaselineSummaryMs_;
  uint32_t runtimeStableCandidateSinceMs_;
  uint32_t runtimeStableEnteredMs_;
  uint32_t longRunNormalOldestMin_;
  uint32_t longRunNormalOldestMax_;
  uint64_t longRunNormalOldestSum_;
  uint32_t longRunNormalOldestSamples_;
  uint32_t longRunNormalDepthMax_;
  uint32_t longRunCanTxDepthMax_;
  uint32_t lastAutoGroupTimeoutSnapshotMs_;
  uint32_t lastPollMicros_;
  uint32_t lastTcpPollMicros_;
  uint32_t lastCanPollMicros_;
  uint32_t loopMaxUsWindow_;
  uint64_t loopSumUsWindow_;
  uint32_t loopSamplesWindow_;
  uint32_t canPollMaxGapUsWindow_;
  uint32_t tcpPollMaxGapUsWindow_;
  uint32_t maxSerialPrintBurstMsWindow_;
  uint32_t serialBytesPrintedEstimateWindow_;
  uint64_t canResponseLatencySumMs_;
  uint32_t canResponseLatencyCount_;
  mutable uint32_t lastPrintedCanResponseOver1000Ms_;
  mutable uint8_t canResponseOver1000GrowthStreak_;
  uint32_t lastIdentTxMs_;
  bool lastTcpConnected_;
  uint32_t r1ConnectedSinceMs_;
  uint32_t bootstrapProtectionUntilMs_;
  uint32_t bootstrapCompleteStableSinceMs_;
  uint32_t bootstrapRecoveryWindowStartedMs_;
  uint8_t bootstrapRecoverySentInWindow_;
  uint8_t powerOnBootstrapRecoveryNextAmp_;
  uint8_t lastActiveAmpCount_;
  uint32_t firstAmpDiscoveryMs_;
  uint32_t lastAmpDiscoveryMs_;
  uint32_t lastActiveAmpCountChangeMs_;
  uint16_t lastRuntimePendingTcpReplies_;
  uint16_t lastRuntimePendingCanReads_;
  uint16_t lastRuntimeMissingStartupBlockers_;
  uint16_t lastRuntimeBootstrapNodesIncomplete_;
  uint32_t lastRuntimeR1TimeoutEventsSnapshot_;
  bool runtimeStableEntered_;
  bool gentleStatusShapingRuntimeEnabled_;
  bool bootstrapProtectionRuntimeEnabled_;
  bool bootstrapRetryRuntimeEnabled_;
  bool segmentedDampeningRuntimeEnabled_;
  bool fanoutAttachRuntimeEnabled_;
  bool postBootstrapStatusCacheRuntimeEnabled_;
  bool gentleStatusShapingHasEverBeenActive_;
  bool gentleStatusShapingLastEnabled_;
  bool statefulStatusCacheHasEverBeenActive_;
  bool statefulStatusCacheLastEnabled_;
  uint32_t lastR1TimeoutEvents_;
  uint32_t r1TimeoutEventsAtGentleStatusShapingEnable_;
  uint32_t r1TimeoutEventsAtStatefulCacheEnable_;
  uint32_t lastChannelIncompleteCount_;
  uint32_t lastSubnetServiceGapCount_;
  uint8_t statefulStatusCacheDisabledReasonCode_;
  uint32_t lastNodeCanTxMs_[GatewayState::kMaxAddressSlots];
  uint32_t lastSlowOver500Ms_;
  SlowObjectFairnessState objectFairnessStates_[kSlowObjectBucketCapacity];
  uint16_t fairnessLastNodeKey_;
  uint16_t fairnessCurrentNodeStreak_;
  uint16_t fairnessLastObjectKey_;
  uint16_t fairnessCurrentObjectStreak_;
  mutable SlowCanWindow slowCanWindow_;
  SoftInflightCollapseEntry softInflightCollapse_[kSoftInflightCollapseCapacity];
  GentleStatusShapingEntry gentleStatusShaping_[kGentleStatusShapingCapacity];
  DeferredSegmentedRequestEntry deferredSegmented_[kDeferredSegmentedCapacity];
  DeferredIdentRequestEntry deferredIdent_[kDeferredIdentCapacity];
  ActiveShapingCadenceState activeShapingCadence_[GatewayState::kMaxAddressSlots][kActiveShapingCadenceGroupCount];
  StatefulStatusCacheEntry statefulStatusCache_[kStatefulStatusCacheCapacity];
  StatefulStatusCacheGroupWindow statefulStatusCacheGroups_[kStatefulStatusCacheGroupCount];
  SemanticMirrorEntry semanticMirror_[kSemanticMirrorCapacity];
  SemanticMirrorGroupWindow semanticMirrorGroups_[kSemanticMirrorGroupCount];
  SemanticMirrorWindow semanticMirrorWindow_;
  SemanticCandidateEntry semanticCandidates_[kSemanticCandidateCapacity];
  uint32_t semanticCandidateEvictions_;
  uint32_t semanticCandidateLastSummaryMs_;
  PresenceRoundState presenceRounds_[DebugCounters::kSubnetCount];
  uint32_t semanticMirrorBgRefreshPauseUntilMs_;
  uint32_t semanticMirrorBgRefreshWindow100MsStartedMs_;
  uint32_t semanticMirrorBgRefreshWindow1sStartedMs_;
  uint8_t semanticMirrorBgRefreshCount100Ms_;
  uint8_t semanticMirrorBgRefreshCount1s_;
  uint32_t semanticMirrorBgRefreshLastNodeMs_[GatewayState::kMaxAddressSlots];
  uint32_t semanticMirrorBgRefreshLastSubnetMs_[DebugCounters::kSubnetCount];
  uint16_t semanticMirrorRefreshCursor_;
  uint32_t semanticMirrorProtectedRefreshWindow100MsStartedMs_;
  uint32_t semanticMirrorProtectedRefreshWindow1sStartedMs_;
  uint8_t semanticMirrorProtectedRefreshCount100Ms_;
  uint8_t semanticMirrorProtectedRefreshCount1s_;
  uint32_t semanticMirrorProtectedRefreshLastNodeMs_[GatewayState::kMaxAddressSlots];
  uint32_t semanticMirrorProtectedRefreshLastSubnetMs_[DebugCounters::kSubnetCount];
  uint16_t semanticMirrorProtectedRefreshCursor_;
  ObjectStat objectStats_[kObjectStatCapacity];
  RequestTrace recentRequestTraces_[kRecentRequestTraceCapacity];
  uint16_t recentRequestTraceHead_;
  uint32_t nextRequestTraceId_;
  AliveDiagnosticTrace* aliveDiagnosticTraces_;
  uint16_t aliveDiagnosticTraceHead_;
  bool timeoutWindowDumpActive_;
  uint32_t timeoutWindowDumpEventMs_;
  uint32_t timeoutWindowDumpDueMs_;
  uint8_t timeoutWindowDumpCount_;
  uint8_t timeoutWindowDumpSubnets_[8];
  uint8_t timeoutWindowDumpDevices_[8];

  void resetObjectStats();
  void resetRequestTraces();
  void recordAliveDiagnosticTrace(const RequestTrace& trace);
  void recordAliveDiagnosticCanReply(const NormalizedRequest& reply, uint32_t nowMs);
  ObjectStat* findOrCreateObjectStat(uint16_t objectKey, FieldId fieldId);
  RequestTrace* recordRequestTrace(const NormalizedRequest& request,
                                   uint32_t nowMs,
                                   uint16_t pendingTcpReplies,
                                   uint16_t normalQueueDepth,
                                   uint16_t canTxQueueDepth,
                                   uint8_t duplicatePendingCount);
  RequestTrace* findMatchingPendingTrace(const NormalizedRequest& request);
  void markTraceForwardedToCan(RequestTrace* trace);
  void markTraceCacheReply(RequestTrace* trace, uint32_t nowMs);
  void markTraceMirrorReply(RequestTrace* trace, uint32_t nowMs);
  void markTraceStaleAtRequest(RequestTrace* trace);
  void markTraceMirrorForceCan(RequestTrace* trace, TraceMirrorDecision decision);
  void markTraceBackgroundRefresh(RequestTrace* trace, TraceBackgroundRefreshState state);
  void markTraceCanComplete(RequestTrace* trace, uint32_t nowMs, bool partialSegment);
  void markTraceTimeoutCleanup(RequestTrace* trace, uint32_t nowMs, TraceReleaseReason reason);
  void processTraceTimeoutCleanup(uint32_t nowMs);
  void logTcpPendingTrace(const RequestTrace& trace, const char* stateText) const;
  void logTcpPendingSummary(uint32_t nowMs) const;
  uint32_t maxPendingTcpOldestMs(uint32_t nowMs) const;
  void markTraceRealCanRx(const NormalizedRequest& reply, uint32_t nowMs);
  void markTraceTcpTx(const NormalizedRequest& reply, TraceSource source, uint32_t nowMs);
  static void handleSchedulerCanTx(void* context,
                                   const NormalizedRequest& request,
                                   QueueClass queueClass,
                                   uint32_t enqueuedMs,
                                   uint32_t nowMs);
  void noteSchedulerCanTx(const NormalizedRequest& request,
                          QueueClass queueClass,
                          uint32_t enqueuedMs,
                          uint32_t nowMs);
  static SlowTraceCategory classifySlowTraceCategory(const NormalizedRequest& request);
  static const char* slowTraceCategoryName(SlowTraceCategory category);
  static ExpensiveObjectGroup classifyExpensiveObjectGroup(const NormalizedRequest& request);
  static const char* expensiveObjectGroupName(ExpensiveObjectGroup group);
  static ShapingSimGroup classifyShapingSimGroup(const NormalizedRequest& request);
  static const char* shapingSimGroupName(ShapingSimGroup group);
  static uint16_t shapingSimIntervalMs(ShapingSimGroup group);
  static uint16_t gentleStatusShapingWindowMs(ShapingSimGroup group);
  static uint16_t gentleStatusShapingMaxWaitMs(ShapingSimGroup group);
  static uint8_t classifyStartupBlockerFamily(const NormalizedRequest& request);
  static const char* startupBlockerFamilyName(uint8_t family);
  static uint8_t activeShapingCadenceGroupIndex(ShapingSimGroup group);
  static const char* shapingDisabledReasonName(uint8_t code);
  static uint16_t estimateCanFrameBits(uint8_t dlc);
  void resetSlowCanWindow(uint32_t nowMs) const;
  SlowObjectBucket* findOrCreateSlowObjectBucket(uint16_t objectKey);
  SlowNodeBucket* findOrCreateSlowNodeBucket(uint8_t subnet, uint8_t device);
  SlowObjectFairnessState* findOrCreateObjectFairnessState(uint16_t objectKey);
  ExpensiveObjectWindow* expensiveObjectWindow(ExpensiveObjectGroup group);
  const ExpensiveObjectWindow* expensiveObjectWindow(ExpensiveObjectGroup group) const;
  ShapingSimGroupWindow* shapingSimGroupWindow(ShapingSimGroup group);
  const ShapingSimGroupWindow* shapingSimGroupWindow(ShapingSimGroup group) const;
  uint8_t countPendingExactDuplicates(const NormalizedRequest& request) const;
  void noteExpensiveObjectRequest(const NormalizedRequest& request,
                                  uint32_t nowMs,
                                  uint8_t duplicatePendingCount,
                                  bool burstRepeat,
                                  bool segmentedActiveRepeat);
  void noteExpensiveObjectCanTx(const NormalizedRequest& request);
  void noteExpensiveObjectCanRx(const RequestTrace& trace);
  void noteExpensiveObjectCoalesced(const NormalizedRequest& request);
  void noteShapingSimCanTx(const NormalizedRequest& request, uint32_t nowMs);
  static bool isGentleStatusShapingGroup(ShapingSimGroup group);
  void noteActualShapingSavedCanTx(ShapingSimGroup group, bool softCollapse);
  bool startupFreshnessProtectionActive(uint32_t nowMs) const;
  bool isStartupFreshnessPriorityRequest(const NormalizedRequest& request, uint32_t nowMs) const;
  uint32_t r1ReplyAgeMs(const AmpState& ampState, uint32_t nowMs) const;
  uint32_t maxR1ReplyAgeMs(uint32_t nowMs, uint8_t* devicesOver5s) const;
  uint16_t pendingTcpReplyCount() const;
  void printR1FreshnessSummary(Print& out, uint32_t nowMs) const;
  void logR1FreshnessSummary(uint32_t nowMs, const char* prefix) const;
  GentleStatusShapingEntry* findGentleStatusShapingEntry(const NormalizedRequest& request);
  const GentleStatusShapingEntry* findGentleStatusShapingEntry(const NormalizedRequest& request) const;
  GentleStatusShapingEntry* reserveGentleStatusShapingEntry(const NormalizedRequest& request);
  void clearGentleStatusShapingEntries();
  uint8_t activeGentleDeferredCount() const;
  bool gentleStatusShapingFeatureEnabled() const;
  bool gentleStatusShapingEnabled() const;
  bool gentleStatusShapingAutoDisableEligible(const char* reason) const;
  uint32_t currentBusUtilAvgPercent(uint32_t nowMs) const;
  bool slow500GuardActive(uint32_t nowMs) const;
  bool postBootstrapStatusCacheWouldServe(const NormalizedRequest& request, uint32_t nowMs) const;
  ActiveShapingCadenceState* activeShapingCadenceState(const NormalizedRequest& request);
  const ActiveShapingCadenceState* activeShapingCadenceState(const NormalizedRequest& request) const;
  void resetActiveShapingCadence();
  bool tryDeferGentleStatusShaping(const NormalizedRequest& request, uint32_t nowMs);
  void processGentleStatusShaping(uint32_t nowMs);
  void noteGentleStatusShapingCanTx(const NormalizedRequest& request, uint32_t nowMs);
  uint8_t releaseGentleStatusShapingWaiters(const NormalizedRequest& reply, uint32_t nowMs);
  static bool exactWrappedReadMatch(const NormalizedRequest& lhs, const NormalizedRequest& rhs);
  bool hasRecentExactBurst(const NormalizedRequest& request,
                           uint32_t nowMs,
                           uint32_t windowMs) const;
  uint32_t softInflightCollapseTimeoutMs() const;
  SoftInflightCollapseEntry* findSoftInflightCollapseEntry(const NormalizedRequest& request);
  const SoftInflightCollapseEntry* findSoftInflightCollapseEntry(const NormalizedRequest& request) const;
  void clearSoftInflightCollapseEntries();
  uint32_t activeFanoutWaiterCount() const;
  bool fanoutAttachFeatureEnabled() const;
  bool segmentedDampeningFeatureEnabled() const;
  bool semanticMirrorActive(uint32_t nowMs) const;
  static bool isSemanticMirrorRequest(const NormalizedRequest& request);
  static SemanticMirrorGroup classifySemanticMirrorGroup(const NormalizedRequest& request);
  static const char* semanticMirrorGroupName(SemanticMirrorGroup group);
  static bool isProtectedSemanticMirrorGroup(SemanticMirrorGroup group);
  static uint32_t semanticMirrorHardMaxAgeMs();
  static uint32_t semanticMirrorGroupTargetAgeMs(SemanticMirrorGroup group);
  static uint32_t semanticMirrorGroupHardMaxAgeMs(SemanticMirrorGroup group);
  uint32_t semanticMirrorFreshTtlMs(const NormalizedRequest& request) const;
  SemanticMirrorEntry* findSemanticMirrorEntry(const NormalizedRequest& request);
  const SemanticMirrorEntry* findSemanticMirrorEntry(const NormalizedRequest& request) const;
  SemanticMirrorEntry* reserveSemanticMirrorEntry(const NormalizedRequest& request);
  void clearSemanticMirror();
  void resetSemanticMirrorSummaryWindow();
  void resetSemanticMirrorBackgroundRefreshControl();
  bool semanticMirrorProtectedRefreshAllowed(const NormalizedRequest& request, uint32_t nowMs, const char*& reason);
  void noteSemanticMirrorProtectedRefreshSent(const NormalizedRequest& request, uint32_t nowMs);
  void rememberSemanticMirrorRequestTemplate(const NormalizedRequest& request);
  bool buildSemanticMirrorRefreshRequest(const SemanticMirrorEntry& entry, NormalizedRequest& request) const;
  void processSemanticMirrorFreshness(uint32_t nowMs);
  void logSemanticHealth(uint32_t nowMs) const;
  void invalidateSemanticMirror(uint8_t subnet, uint8_t device, uint8_t channel);
  bool updateSemanticMirrorOptimisticallyForWrite(const NormalizedRequest& request,
                                                   uint32_t nowMs);
  void confirmSemanticMirrorWriteAck(const NormalizedRequest& reply, uint32_t nowMs);
  void confirmPowerOnFromNodeActivity(const NormalizedRequest& reply, uint32_t nowMs);
  void invalidateSemanticMirrorForWrite(const NormalizedRequest& request, const char* reason);
  bool isSemanticMirrorBackgroundRefreshReply(const NormalizedRequest& reply) const;
  bool semanticMirrorBackgroundRefreshPaused(uint32_t nowMs) const;
  bool semanticMirrorBackgroundRefreshAllowed(const NormalizedRequest& request,
                                              uint32_t nowMs,
                                              const char*& reason,
                                              bool& rateLimited);
  void noteSemanticMirrorBackgroundRefreshSent(const NormalizedRequest& request, uint32_t nowMs);
  void pauseSemanticMirrorBackgroundRefresh(uint32_t nowMs, const char* reason);
  void noteSemanticMirrorCanReply(const NormalizedRequest& reply, uint32_t nowMs, bool forwardedToR1);
  void clearSemanticCandidates();
  void noteSemanticCandidate(uint8_t subnet,
                             uint8_t device,
                             uint8_t channel,
                             uint16_t objectKey,
                             const uint8_t* payload,
                             uint8_t payloadLength,
                             uint32_t nowMs,
                             bool r1Requested,
                             const char* reason);
  void logSemanticCandidateSummary(uint32_t nowMs);
  void clearPresenceRounds();
  void startPresenceRound(const NormalizedRequest& request, uint32_t nowMs);
  void notePresenceReply(const NormalizedRequest& reply, uint32_t nowMs);
  void processPresenceRetries(uint32_t nowMs);
  bool tryHandleSemanticMirrorRequest(const NormalizedRequest& request,
                                      RequestTrace* trace,
                                      uint32_t nowMs);
  void logSemanticMirrorSummary() const;
  void clearDeferredSegmentedRequests();
  void clearDeferredIdentRequests();
  bool tryQueueDeferredSegmentedRequest(const NormalizedRequest& request, uint32_t nowMs);
  bool processDeferredSegmentedRequests(uint32_t nowMs);
  bool tryQueueDeferredIdentRequest(const NormalizedRequest& request, uint32_t nowMs);
  bool processDeferredIdentRequests(uint32_t nowMs);
  static bool isPostBootstrapStatusCacheField(FieldId fieldId);
  static bool isPostBootstrapStatusCacheRequest(const NormalizedRequest& request);
  static StatefulStatusCacheGroup classifyStatefulStatusCacheGroup(const NormalizedRequest& request);
  static const char* statefulStatusCacheGroupName(StatefulStatusCacheGroup group);
  static uint8_t statefulStatusCacheDisabledReason(bool featureEnabled,
                                                   bool tcpConnected,
                                                   uint8_t activeAmpCount,
                                                   uint32_t firstAmpDiscoveryMs,
                                                   uint32_t lastAmpDiscoveryMs,
                                                   uint32_t lastActiveAmpCountChangeMs,
                                                   uint32_t bootstrapCompleteStableSinceMs,
                                                   uint32_t bootstrapNodesIncomplete,
                                                   uint32_t nowMs);
  static const char* statefulStatusCacheDisabledReasonName(uint8_t reasonCode);
  void clearStatefulStatusCache();
  void resetStatefulStatusCacheGroupWindows();
  StatefulStatusCacheGroupWindow* statefulStatusCacheGroupWindow(StatefulStatusCacheGroup group);
  const StatefulStatusCacheGroupWindow* statefulStatusCacheGroupWindow(StatefulStatusCacheGroup group) const;
  void autoDisableStatefulStatusCache(uint8_t reasonCode, const char* reason);
  void invalidateStatefulStatusCache(uint8_t subnet, uint8_t device, uint8_t channel);
  StatefulStatusCacheEntry* findStatefulStatusCacheEntry(const NormalizedRequest& request);
  const StatefulStatusCacheEntry* findStatefulStatusCacheEntry(const NormalizedRequest& request) const;
  StatefulStatusCacheEntry* reserveStatefulStatusCacheEntry(const NormalizedRequest& request);
  bool postBootstrapStatusCacheFeatureEnabled() const;
  bool postBootstrapStatusCacheActive(uint32_t nowMs) const;
  void printStatefulStatusCacheActivationStatus(Print& out, uint32_t nowMs) const;
  void printStatefulStatusCacheGroupSummaries(Print& out) const;
  bool tryHandlePostBootstrapStatusCacheRequest(const NormalizedRequest& request,
                                                RequestTrace* trace,
                                                uint32_t nowMs);
  uint8_t releaseStatefulStatusCacheWaiters(const NormalizedRequest& reply, uint32_t nowMs);
  void noteStatefulStatusCacheCanReply(const NormalizedRequest& reply, uint32_t nowMs);
  void resetStatefulStatusCacheSummaryWindow();
  void expireSoftInflightCollapse(uint32_t nowMs);
  bool tryAttachSoftInflightCollapse(const NormalizedRequest& request, uint32_t nowMs);
  void noteSoftInflightPrimary(const NormalizedRequest& request, uint32_t nowMs);
  uint8_t releaseSoftInflightCollapseWaiters(const NormalizedRequest& reply, uint32_t nowMs);
  void noteCanBusFrame(uint32_t nowMs, uint8_t subnet, bool txFrame, uint8_t dlc);
  void noteSlowCanResponse(const RequestTrace& trace);
  void printSlowCanSummary(Print& out) const;
  void rebuildWatchdogState(uint32_t nowMs);
  void logTimeoutCorrelation(uint8_t subnet, uint8_t device, uint32_t nowMs, const char* prefix) const;
  void logAliveCandidateCorrelation(uint8_t subnet, uint8_t device, uint32_t nowMs) const;
  void logHealthyPeerComparison(uint8_t timeoutSubnet,
                                uint8_t timeoutDevice,
                                uint32_t nowMs) const;
  void logGroupTimeoutCorrelation(const uint8_t* subnets,
                                  const uint8_t* devices,
                                  uint8_t count,
                                  uint32_t nowMs,
                                  const char* prefix) const;
  void beginTimeoutWindowDump(uint32_t nowMs);
  void processTimeoutWindowDump(uint32_t nowMs);
  void logTimeoutWindowDump(uint32_t startMs,
                            uint32_t endMs,
                            uint32_t nowMs,
                            const char* prefix) const;
  static const char* traceSourceName(TraceSource source);
  static const char* traceMirrorDecisionName(TraceMirrorDecision decision);
  static const char* traceBackgroundRefreshStateName(TraceBackgroundRefreshState state);
  static const char* tracePendingStateName(TracePendingState state);
  static const char* traceReleaseReasonName(TraceReleaseReason reason);
  const RequestTrace* findLastSuccessfulTraceForNode(uint8_t subnet, uint8_t device) const;
  static uint8_t semanticFamilyIndex(SemanticMirrorGroup group);
  static const char* semanticSourceName(uint8_t sourceCode);
  void noteSemanticFamilyRequested(const NormalizedRequest& request, uint32_t nowMs);
  void noteSemanticFamilyUpdated(const NormalizedRequest& request,
                                 uint32_t nowMs,
                                 uint8_t sourceCode,
                                 bool servedToR1);
  void logNodeSemanticEpoch(uint8_t subnet, uint8_t device, uint32_t nowMs, const char* prefix) const;
  void logNodeSemanticInconsistency(uint8_t subnet, uint8_t device, uint32_t nowMs, const char* prefix) const;
  void logUnknown0111Summary(uint32_t nowMs) const;
  void queueUnknown0111Refresh(const NormalizedRequest& request, uint32_t nowMs);
  void queueObject0100HeartbeatFromAliveRequest(const NormalizedRequest& request, uint32_t nowMs);
  void queueLiveRefresh(const NormalizedRequest& request, uint32_t nowMs, bool r1Requested = false);
  void processLocalControlRefresh(uint32_t nowMs);
  void processIspRefresh(uint32_t nowMs);
  void processOverdueAliveRefresh(uint32_t nowMs);
  uint8_t nextAliveRefreshSlot_ = 0;
  uint16_t nextStableAliveRefreshSlot_ = 0;
  uint32_t r1Object0100HeartbeatCanTx_ = 0;
  uint32_t r1Object0100HeartbeatCanRx_ = 0;
  uint32_t r1Object0100HeartbeatTcpTx_ = 0;
  uint32_t r1Object0100HeartbeatTcpFailed_ = 0;
  uint32_t lastIspRefreshMs_ = 0;
  uint32_t lastIspScanMs_ = 0;
  uint8_t nextIspRefreshSlot_ = 0;
  uint8_t nextFastLiveRefreshSlot_ = 0;
  static constexpr uint8_t kExtraLiveRefreshCount = 24;
  struct ExtraLiveRefresh {
    bool used;
    uint8_t subnet;
    uint8_t device;
    FieldId fieldId;
    AmpState::LiveRefresh track;
    bool pushedValueValid;
    Frame lastPushedValue;
    uint32_t lastPushMs;
  } extraLiveRefresh_[kExtraLiveRefreshCount] = {};
  uint8_t nextExtraLiveRefreshSlot_ = 0;
  uint32_t lastLiveStatusPushMs_ = 0;
  uint32_t liveStatusPushSent_ = 0;
  uint32_t liveStatusPushRateLimited_ = 0;
  uint32_t lastLocalControlRefreshMs_ = 0;
  uint32_t lastLocalControlRefreshDiagMs_ = 0;
  uint16_t nextLocalControlRefreshSlot_ = 0;
  void logTrueCacheAges(uint8_t subnet, uint8_t device, uint32_t nowMs) const;
  void pollCli(uint32_t nowMs);
  void handleCliLine(const char* line, uint32_t nowMs);
  void logFocusedLiveMeter(const char* stage,
                           const NormalizedRequest& request,
                           const Frame& frame,
                           uint32_t nowMs,
                           uint32_t ageMs = 0) const;
  void noteObjectTcpRead(const NormalizedRequest& request, uint32_t nowMs);
  void noteObjectCanTxForwarded(const NormalizedRequest& request);
  void noteObjectCacheReply(const NormalizedRequest& request, uint32_t nowMs);
  void noteObjectCacheStale(const NormalizedRequest& request);
  void noteObjectCacheMiss(const NormalizedRequest& request);
  void noteObjectCoalesced(const NormalizedRequest& request);
  void noteObjectQueueFull(const NormalizedRequest& request);
  void logTopObjectStats() const;
  void noteBootstrapReply(const NormalizedRequest& reply, uint32_t nowMs);
  void noteBootstrapRequestObserved(const NormalizedRequest& request, uint32_t nowMs);
  void noteStartupBlockerRequestObserved(const NormalizedRequest& request, uint32_t nowMs);
  void noteStartupBlockerReplyForwarded(const NormalizedRequest& reply, uint32_t nowMs);
  uint16_t currentMissingStartupBlockerCount(uint8_t familyCounts[kStartupBlockerFamilyCount]) const;
  const char* worstMissingStartupBlockerFamilyName(uint8_t* missingNodes = 0) const;
  void resetRuntimeStabilityTracking();
  void logRuntimeBaselineSummary(uint32_t nowMs);
  void restartBootstrapPhase(uint32_t nowMs, bool resetPerNodeState);
  void armPowerOnBootstrap(uint8_t subnet, uint8_t device, uint32_t nowMs);
  bool powerOnBootstrapActive(const AmpState& ampState, uint32_t nowMs) const;
  bool bootstrapProtectionFeatureEnabled() const;
  bool bootstrapProtectionActive(uint32_t nowMs) const;
  uint32_t bootstrapRequiredMask(const AmpState& ampState) const;
  bool buildBootstrapRetryRequest(uint8_t subnet,
                                  uint8_t device,
                                  uint8_t bootstrapIndex,
                                  NormalizedRequest& request) const;
  bool bootstrapRetryFeatureEnabled() const;
  void processBootstrapRecovery(uint32_t nowMs);
  void updateBootstrapCounters(uint32_t nowMs);
  void logBootstrapIncompleteNodes(uint32_t nowMs);
  static const char* regressionFeatureName(RegressionFeature feature);
  void disableRegressionFeature(RegressionFeature feature, const char* reason);
  void disableActiveRegressionFeatures(const char* reason, bool includePostBootstrapStatusCache = true);
  void updateRegressionCounters(uint32_t nowMs);
  void evaluateRegressionGuards(uint32_t nowMs);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_GATEWAY_V2_H
