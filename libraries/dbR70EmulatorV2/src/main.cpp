#include "R70GatewayV2.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#endif
#include <string.h>

#include "DbCanProtocol.h"
#include "gpi/GpiManager.h"
#include "protocol/CanObjectCodec.h"
#include "protocol/R70Frame.h"
#include "network/NetworkConfig.h"

namespace dbcan {
namespace r70v2 {

namespace {

static const uint8_t kTraceFlagForwardedToCan = 0x01;
static const uint8_t kTraceFlagCacheReplied = 0x02;
static const uint8_t kTraceFlagRealCanRxSeen = 0x04;
static const uint8_t kTraceFlagTcpTxSent = 0x08;
static const uint8_t kTraceFlagStaleAtRequest = 0x10;
static const uint8_t kTraceFlagCanTxSeen = 0x20;
static const uint8_t kTraceFlagNearIdentBurst = 0x40;
static const uint8_t kTraceFlagSlowAttributed = 0x80;
static const uint32_t kSoftInflightBurstWindowMs = 100u;
static const uint32_t kGentleStatusShapingFreshnessMaxMs = 250u;
static const uint32_t kSlow500GuardHoldMs = 1500u;
static const uint32_t kSegmentedGuardBaseDeferMs = 70u;
static const uint32_t kNameSegmentGuardBaseDeferMs = 110u;
static const uint32_t kSegmentedGuardMaxDeferMs = 150u;
static const uint32_t kIdentGuardBaseDeferMs = 150u;
static const uint32_t kIdentGuardMaxDeferMs = 250u;
static const uint32_t kSemanticMirrorBackgroundRefreshPauseMs = 60000u;
// A write ACK only confirms transport, not that the amplifier's readable state
// has already settled.  Keep the optimistic R1 value protected while the slow
// power transition completes; mute can be verified much sooner.
static const uint32_t kPowerWriteThroughVerifyDelayMs = 5000u;
static const uint32_t kMuteWriteThroughVerifyDelayMs = 250u;
// Replies already queued before a group Power ON can arrive just after the
// write and must not be mistaken for proof that the amplifier has restarted.
static const uint32_t kPowerOnNodeActivityGuardMs = 1000u;
static const uint32_t kWriteThroughRefreshRetryMs = 2000u;
// A recent genuine CAN reply confirms that the amplifier is alive.  During the
// normal runtime this lets R1 reuse all already learned values for that node,
// while each object's payload and CAN refresh age remain independent.
static const uint32_t kNodeFreshMirrorServeMaxAgeMs = 3000u;
static const uint32_t kSemanticMirrorBackgroundRefreshNodeCooldownMs = 100u;
static const uint32_t kSemanticMirrorBackgroundRefreshSubnetCooldownMs = 25u;
static const uint32_t kSemanticMirrorBackgroundRefreshMaxServiceGapMs = 400u;
static const uint32_t kSemanticMirrorBackgroundRefreshPauseServiceGapMs = 750u;
static const uint16_t kSemanticMirrorBackgroundRefreshMaxPendingTcpReplies = 20u;
static const uint16_t kSemanticMirrorBackgroundRefreshMaxCanTxQueueDepth = 4u;
static const uint8_t kSemanticMirrorBackgroundRefreshMaxPer100Ms = 3u;
// 32 amps * (0111 + 0500) / 3 seconds = 21.34 reads/s plus headroom.
static const uint8_t kSemanticMirrorBackgroundRefreshMaxPer1s = 24u;
static const uint32_t kSemanticMirrorProtectedRefreshNodeCooldownMs = 500u;
static const uint32_t kSemanticMirrorProtectedRefreshSubnetCooldownMs = 100u;
static const uint8_t kSemanticMirrorProtectedRefreshMaxPer100Ms = 1u;
static const uint8_t kSemanticMirrorProtectedRefreshMaxPer1s = 8u;
static const uint8_t kSemanticMirrorReservedSlotsPerSecond = 8u;
static const uint8_t kSemanticMirrorReservedMaintenanceBacklog = 2u;
static const uint8_t kSemanticRefreshBlockedNone = 0u;
static const uint8_t kSemanticRefreshBlockedTcpPressure = 1u;
static const uint8_t kSemanticRefreshBlockedQueue = 2u;
static const uint8_t kSemanticRefreshBlockedRateLimit = 3u;
static const uint8_t kSemanticRefreshBlockedNodeCooldown = 4u;
static const uint8_t kSemanticRefreshBlockedSubnetCooldown = 5u;
static const uint8_t kSemanticRefreshBlockedHardlimit = 6u;
static const uint8_t kSemanticRefreshWriteReadbackCaptured = 7u;
static const uint8_t kSemanticPowerOffAwaitingWriteAck = 8u;
static const uint8_t kSemanticPowerOffWriteAckConfirmed = 9u;
static const uint8_t kSegmentedGuardQueueThreshold = 10u;
static const uint8_t kNameSegmentGuardQueueThreshold = 6u;
static const uint8_t kIdentGuardQueueThreshold = 8u;
static const uint8_t kSegmentedGuardUtilThresholdPercent = 55u;
static const uint8_t kNameSegmentGuardUtilThresholdPercent = 50u;
static const uint8_t kIdentGuardUtilThresholdPercent = 52u;

uint16_t normalizedObjectKey(const NormalizedRequest& request);

const char* semanticRefreshBlockedReasonName(uint8_t code) {
  switch (code) {
    case kSemanticRefreshBlockedTcpPressure:
      return "tcp_pressure";
    case kSemanticRefreshBlockedQueue:
      return "queue";
    case kSemanticRefreshBlockedRateLimit:
      return "rate_limit";
    case kSemanticRefreshBlockedNodeCooldown:
      return "node_cooldown";
    case kSemanticRefreshBlockedSubnetCooldown:
      return "subnet_cooldown";
    case kSemanticRefreshBlockedHardlimit:
      return "hardlimit";
    case kSemanticRefreshBlockedNone:
    default:
      return "none";
  }
}

uint16_t semanticMirrorNodeIndex(uint8_t subnet, uint8_t device) {
  return static_cast<uint16_t>((static_cast<uint16_t>(subnet) << 6) | (device & 0x3Fu));
}

bool runtimeReadable(const MappingEntry* mapping) {
  return mapping != 0 && mapping->allowReadPoll &&
         (mapping->behaviorConfidence == BehaviorConfidence::ConfirmedCapture ||
          mapping->behaviorConfidence == BehaviorConfidence::ConfirmedRoundTrip);
}

bool runtimeWritable(const MappingEntry* mapping) {
  return mapping != 0 && mapping->allowWrite &&
         (mapping->behaviorConfidence == BehaviorConfidence::ConfirmedCapture ||
          mapping->behaviorConfidence == BehaviorConfidence::ConfirmedRoundTrip);
}

bool isDefaultConfirmedReadCacheField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::settings_power:
    case FieldId::config_delay_on_a:
    case FieldId::config_delay_on_b:
    case FieldId::config_mute_a:
    case FieldId::config_mute_b:
    case FieldId::config_gain_a:
    case FieldId::config_gain_b:
    case FieldId::config_delay_value_a:
    case FieldId::config_delay_value_b:
      return true;
    default:
      return false;
  }
}

bool isConfigScalarTtlField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::settings_power:
    case FieldId::config_mute_a:
    case FieldId::config_mute_b:
    case FieldId::config_gain_a:
    case FieldId::config_gain_b:
      return true;
    default:
      return false;
  }
}

bool isNameSegmentCacheField(FieldId fieldId) {
  return fieldId == FieldId::settings_device_name ||
         fieldId == FieldId::config_name_a ||
         fieldId == FieldId::config_name_b ||
         fieldId == FieldId::error_text ||
         fieldId == FieldId::cherr_error_text_a ||
         fieldId == FieldId::cherr_error_text_b;
}

bool isNameSegmentReadRequest(const NormalizedRequest& request) {
  return request.valid &&
         request.tcpWrappedCan &&
         isReadFunction(request.admin.functionCode) &&
         request.admin.telegramType == TelegramType::Segmented &&
         request.objectAddress.hasSegment &&
         isNameSegmentCacheField(request.fieldId);
}

bool isNameSegmentWriteRequest(const NormalizedRequest& request) {
  return request.valid &&
         request.tcpWrappedCan &&
         isWriteFunction(request.admin.functionCode) &&
         isSegmentedType(request.admin.telegramType) &&
         isNameSegmentCacheField(request.fieldId);
}

bool isSegmentedReadRequest(const NormalizedRequest& request) {
  return request.valid &&
         request.tcpWrappedCan &&
         request.admin.valid &&
         isReadFunction(request.admin.functionCode) &&
         request.admin.telegramType == TelegramType::Segmented &&
         request.canId.device != 0;
}

bool isSmallStatusCacheExperimentField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::status_pwr_ok:
    case FieldId::error_general:
    case FieldId::chstatus_isp_a:
    case FieldId::chstatus_isp_b:
    case FieldId::chstatus_remhold_gr_a:
    case FieldId::chstatus_remhold_gr_b:
    case FieldId::chstatus_remhold_ovl_a:
    case FieldId::chstatus_remhold_ovl_b:
    case FieldId::cherr_channel_err_a:
    case FieldId::cherr_channel_err_b:
      return true;
    default:
      return false;
  }
}

bool parseNodeSpecInText(const char* text, uint8_t* subnet, uint8_t* device) {
  if (text == 0) {
    return false;
  }
  for (const char* cursor = text; *cursor != '\0'; ++cursor) {
    if (*cursor < '0' || *cursor > '9') {
      continue;
    }
    char* subnetEnd = 0;
    const unsigned long parsedSubnet = strtoul(cursor, &subnetEnd, 10);
    if (subnetEnd == cursor || parsedSubnet > 7) {
      continue;
    }
    const char* deviceCursor = subnetEnd;
    if (*deviceCursor == '.') {
      ++deviceCursor;
    } else if (*deviceCursor == ' ') {
      while (*deviceCursor == ' ') {
        ++deviceCursor;
      }
    } else {
      continue;
    }
    char* deviceEnd = 0;
    const unsigned long parsedDevice = strtoul(deviceCursor, &deviceEnd, 10);
    if (deviceEnd == deviceCursor || parsedDevice > 63) {
      continue;
    }
    if (subnet != 0) {
      *subnet = static_cast<uint8_t>(parsedSubnet);
    }
    if (device != 0) {
      *device = static_cast<uint8_t>(parsedDevice);
    }
    return true;
  }
  return false;
}

uint8_t parseNodeListInText(const char* text,
                            uint8_t* subnets,
                            uint8_t* devices,
                            uint8_t capacity) {
  if (text == 0 || subnets == 0 || devices == 0 || capacity == 0) {
    return 0;
  }
  uint8_t count = 0;
  const char* cursor = text;
  while (*cursor != '\0' && count < capacity) {
    while (*cursor != '\0' && (*cursor < '0' || *cursor > '9')) {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }
    char* subnetEnd = 0;
    const unsigned long parsedSubnet = strtoul(cursor, &subnetEnd, 10);
    if (subnetEnd == cursor || parsedSubnet > 7) {
      ++cursor;
      continue;
    }
    const char* deviceCursor = subnetEnd;
    if (*deviceCursor == '.') {
      ++deviceCursor;
    } else if (*deviceCursor == ' ') {
      while (*deviceCursor == ' ') {
        ++deviceCursor;
      }
    } else {
      cursor = subnetEnd;
      continue;
    }
    char* deviceEnd = 0;
    const unsigned long parsedDevice = strtoul(deviceCursor, &deviceEnd, 10);
    if (deviceEnd == deviceCursor || parsedDevice > 63) {
      cursor = subnetEnd;
      continue;
    }
    bool duplicate = false;
    for (uint8_t index = 0; index < count; ++index) {
      if (subnets[index] == parsedSubnet && devices[index] == parsedDevice) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      subnets[count] = static_cast<uint8_t>(parsedSubnet);
      devices[count] = static_cast<uint8_t>(parsedDevice);
      ++count;
    }
    cursor = deviceEnd;
  }
  return count;
}

bool nodeInList(uint8_t subnet,
                uint8_t device,
                const uint8_t* subnets,
                const uint8_t* devices,
                uint8_t count,
                uint8_t* matchIndex = 0) {
  for (uint8_t index = 0; index < count; ++index) {
    if (subnets[index] == subnet && devices[index] == device) {
      if (matchIndex != 0) {
        *matchIndex = index;
      }
      return true;
    }
  }
  return false;
}

uint8_t bitCount32(uint32_t value) {
  uint8_t count = 0;
  while (value != 0) {
    count += static_cast<uint8_t>(value & 0x01u);
    value >>= 1;
  }
  return count;
}

bool isCriticalStatusCacheField(FieldId fieldId) {
  return isSmallStatusCacheExperimentField(fieldId);
}

bool isWatchdogStatusCacheField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::status_pwr_ok:
    case FieldId::error_general:
    case FieldId::cherr_channel_err_a:
    case FieldId::cherr_channel_err_b:
    case FieldId::chstatus_remhold_gr_a:
    case FieldId::chstatus_remhold_gr_b:
    case FieldId::chstatus_remhold_ovl_a:
    case FieldId::chstatus_remhold_ovl_b:
      return true;
    default:
      return false;
  }
}

bool isIspCacheField(FieldId fieldId) {
  return fieldId == FieldId::chstatus_isp_a || fieldId == FieldId::chstatus_isp_b;
}

bool isFirmwareSegmentReadRequest(const NormalizedRequest& request) {
  return request.valid &&
         request.tcpWrappedCan &&
         request.fieldId == FieldId::fixed_firmware_version &&
         isReadFunction(request.admin.functionCode) &&
         request.admin.telegramType == TelegramType::Segmented &&
         request.objectAddress.hasSegment;
}

bool isStableSetupField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::config_speaker_id_a:
    case FieldId::config_speaker_id_b:
    case FieldId::settings_input_mode:
    case FieldId::settings_data_link_preset:
      return true;
    default:
      return false;
  }
}

bool isStableSetupReadRequest(const NormalizedRequest& request) {
  return request.valid &&
         request.tcpWrappedCan &&
         isReadFunction(request.admin.functionCode) &&
         request.admin.telegramType == TelegramType::Unstructured &&
         isStableSetupField(request.fieldId);
}

bool isStableSetupWriteRequest(const NormalizedRequest& request) {
  return request.valid &&
         request.tcpWrappedCan &&
         isWriteFunction(request.admin.functionCode) &&
         request.admin.telegramType == TelegramType::Unstructured &&
         isStableSetupField(request.fieldId);
}

bool isSpeakerIdField(FieldId fieldId) {
  return fieldId == FieldId::config_speaker_id_a || fieldId == FieldId::config_speaker_id_b;
}

bool isInputModeField(FieldId fieldId) {
  return fieldId == FieldId::settings_input_mode;
}

bool isDataLinkPresetField(FieldId fieldId) {
  return fieldId == FieldId::settings_data_link_preset;
}

bool isStableSetupCacheEnabled(const GatewayConfig& config, FieldId fieldId) {
  if (!config.enableStableSetupCache) {
    return false;
  }
  if (isSpeakerIdField(fieldId)) {
    return config.enableSpeakerIdCache;
  }
  if (isInputModeField(fieldId)) {
    return config.enableInputModeCache;
  }
  if (isDataLinkPresetField(fieldId)) {
    return config.enableDataLinkPresetCache;
  }
  return false;
}

bool isExpandedStatusCacheField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::status_pwr_ok:
    case FieldId::status_digin_status:
    case FieldId::status_hardware_type:
    case FieldId::chstatus_isp_a:
    case FieldId::chstatus_isp_b:
    case FieldId::chstatus_gr_a:
    case FieldId::chstatus_gr_b:
    case FieldId::chstatus_ovl_a:
    case FieldId::chstatus_ovl_b:
    case FieldId::chstatus_osp_a:
    case FieldId::chstatus_osp_b:
    case FieldId::chstatus_ms_delay_a:
    case FieldId::chstatus_ms_delay_b:
    case FieldId::chstatus_amp_temperature_a:
    case FieldId::chstatus_amp_temperature_b:
    case FieldId::chstatus_input_overload_a:
    case FieldId::chstatus_input_overload_b:
    case FieldId::chstatus_output_overload_a:
    case FieldId::chstatus_output_overload_b:
      return true;
    default:
      return false;
  }
}

bool isErrorStatusCacheField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::error_general:
    case FieldId::error_number:
    case FieldId::cherr_channel_err_a:
    case FieldId::cherr_channel_err_b:
    case FieldId::cherr_err_nr_a:
    case FieldId::cherr_err_nr_b:
      return true;
    default:
      return false;
  }
}

bool isRemHoldCacheField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::chstatus_remhold_isp_a:
    case FieldId::chstatus_remhold_isp_b:
    case FieldId::chstatus_remhold_gr_a:
    case FieldId::chstatus_remhold_gr_b:
    case FieldId::chstatus_remhold_ovl_a:
    case FieldId::chstatus_remhold_ovl_b:
    case FieldId::chstatus_remhold_osp_a:
    case FieldId::chstatus_remhold_osp_b:
      return true;
    default:
      return false;
  }
}

uint32_t nameSegmentCacheTtlMs(const GatewayConfig& config) {
  return config.nameSegmentTtlMs != 0 ? config.nameSegmentTtlMs : 30000;
}

uint32_t firmwareSegmentCacheTtlMs(const GatewayConfig& config) {
  return config.firmwareSegmentTtlMs != 0 ? config.firmwareSegmentTtlMs : 60000;
}

uint32_t stableSetupTtlMs(const GatewayConfig& config, FieldId fieldId) {
  if (isSpeakerIdField(fieldId)) {
    return config.speakerIdTtlMs != 0 ? config.speakerIdTtlMs : 30000;
  }
  if (isInputModeField(fieldId)) {
    return config.inputModeTtlMs != 0 ? config.inputModeTtlMs : 30000;
  }
  if (isDataLinkPresetField(fieldId)) {
    return config.datalinkPresetTtlMs != 0 ? config.datalinkPresetTtlMs : 30000;
  }
  return 0;
}

const char* nameSegmentMissReasonText(NameSegmentCacheMissReason reason) {
  switch (reason) {
    case NameSegmentCacheMissReason::Disabled:
      return "disabled";
    case NameSegmentCacheMissReason::NoSlot:
      return "no_slot";
    case NameSegmentCacheMissReason::NoField:
      return "no_field";
    case NameSegmentCacheMissReason::NoSegment:
      return "no_segment";
    case NameSegmentCacheMissReason::Stale:
      return "stale";
    case NameSegmentCacheMissReason::TtlExpired:
      return "ttl_expired";
    case NameSegmentCacheMissReason::None:
    default:
      return "unknown";
  }
}

void noteNameSegmentMiss(DebugCounters& counters, NameSegmentCacheMissReason reason) {
  ++counters.name_segment_cache_miss;
  switch (reason) {
    case NameSegmentCacheMissReason::Disabled:
      ++counters.name_segment_cache_miss_disabled;
      break;
    case NameSegmentCacheMissReason::NoSlot:
      ++counters.name_segment_cache_miss_no_slot;
      break;
    case NameSegmentCacheMissReason::NoField:
      ++counters.name_segment_cache_miss_no_field;
      break;
    case NameSegmentCacheMissReason::NoSegment:
      ++counters.name_segment_cache_miss_no_segment;
      break;
    case NameSegmentCacheMissReason::Stale:
      ++counters.name_segment_cache_miss_stale;
      break;
    case NameSegmentCacheMissReason::TtlExpired:
      ++counters.name_segment_cache_miss_ttl;
      break;
    case NameSegmentCacheMissReason::None:
    default:
      break;
  }
}

bool shouldLogLowRateCounter(uint32_t count) {
  return count < 8 || (count % 128u) == 0u;
}

bool isActiveConfirmedReadCacheField(const GatewayConfig& config, FieldId fieldId) {
  if (isDefaultConfirmedReadCacheField(fieldId)) {
    return true;
  }
  if (config.enableCriticalStatusCache && isCriticalStatusCacheField(fieldId)) {
    return true;
  }
  if (config.enableSmallStatusCacheExperiment && isSmallStatusCacheExperimentField(fieldId)) {
    return true;
  }
  if (config.enableExpandedStatusCache && isExpandedStatusCacheField(fieldId)) {
    return true;
  }
  if (config.enableErrorStatusCache && isErrorStatusCacheField(fieldId)) {
    return true;
  }
  if (config.enableRemHoldCache && isRemHoldCacheField(fieldId)) {
    return true;
  }
  return false;
}

bool canAttemptConfirmedReadCache(const GatewayConfig& config, const NormalizedRequest& request) {
  if (!config.enableConfirmedReadCache ||
      !request.valid ||
      !request.tcpWrappedCan ||
      request.mapping == 0 ||
      !isReadFunction(request.admin.functionCode) ||
      request.admin.telegramType != TelegramType::Unstructured ||
      request.mapping->behaviorConfidence != BehaviorConfidence::ConfirmedRoundTrip ||
      !request.mapping->allowCacheReply ||
      !isActiveConfirmedReadCacheField(config, request.fieldId)) {
    return false;
  }
  return request.canId.device != 0;
}

uint32_t confirmedReadCacheTtlMs(const GatewayConfig& config, const NormalizedRequest& request) {
  if (config.enableConfigScalarTtlExperiment &&
      config.configScalarTtlMs != 0 &&
      isConfigScalarTtlField(request.fieldId)) {
    return config.configScalarTtlMs;
  }
  if (isWatchdogStatusCacheField(request.fieldId)) {
    return config.watchdogStatusTtlMs != 0 ? config.watchdogStatusTtlMs :
           (config.enableCriticalStatusTtlExperiment ? 1000u :
            (config.criticalStatusTtlMs != 0 ? config.criticalStatusTtlMs : 750u));
  }
  if (isIspCacheField(request.fieldId)) {
    return config.ispTtlMs != 0 ? config.ispTtlMs :
           (config.enableCriticalStatusTtlExperiment ?
                (config.criticalStatusTtlMs != 0 ? config.criticalStatusTtlMs : 750u) :
                500u);
  }
  switch (request.fieldId) {
    case FieldId::status_hardware_type:
      return 10000;
    case FieldId::chstatus_amp_temperature_a:
    case FieldId::chstatus_amp_temperature_b:
      return 1500;
    case FieldId::config_delay_value_a:
    case FieldId::config_delay_value_b:
    case FieldId::chstatus_ms_delay_a:
    case FieldId::chstatus_ms_delay_b:
    case FieldId::error_number:
    case FieldId::cherr_err_nr_a:
    case FieldId::cherr_err_nr_b:
      return 1000;
    default:
      return config.cacheTtlDynamicMs;
  }
}

enum StatefulStatusCacheReason : uint8_t {
  kStatefulStatusCacheActive = 0,
  kStatefulStatusCacheFlagOff = 1,
  kStatefulStatusCacheTcpDisconnected = 2,
  kStatefulStatusCacheNoActiveAmps = 3,
  kStatefulStatusCacheNoDiscoveryYet = 4,
  kStatefulStatusCacheWarmup = 5,
  kStatefulStatusCacheAmpCountUnstable = 6,
  kStatefulStatusCacheBootstrapIncomplete = 7,
  kStatefulStatusCacheBootstrapSteadyDelay = 8,
  kStatefulStatusCacheAutoDisabled = 9,
  kStatefulStatusCacheAutoDisabledChannelIncrease = 10,
  kStatefulStatusCacheAutoDisabledAge = 11,
  kStatefulStatusCacheAutoDisabledServiceGap = 12,
};

bool isPostBootstrapStatusCacheObjectKey(uint16_t objectKey) {
  switch (objectKey) {
    case 0x0300:
    case 0x0500:
    case 0x2400:
    case 0x4400:
    case 0x241D:
    case 0x241E:
    case 0x441D:
    case 0x441E:
    case 0x2600:
    case 0x4600:
      return true;
    default:
      return false;
  }
}

uint32_t postBootstrapStatusCacheTtlMs(const NormalizedRequest& request) {
  const uint32_t nodeKey =
      (static_cast<uint32_t>(request.canId.subnet & 0x07u) << 6) |
      static_cast<uint32_t>(request.canId.device & 0x3Fu);
  const uint32_t objectKey = static_cast<uint32_t>(normalizedObjectKey(request));
  const uint32_t hash = (nodeKey * 131u) ^ (objectKey * 17u);
  const int32_t jitterMs = static_cast<int32_t>(hash % 301u) - 150;
  return static_cast<uint32_t>(1000 + jitterMs);
}

uint32_t postBootstrapStatusCacheHardMaxAgeMs() {
  return 1500u;
}

uint32_t statefulStatusCacheWarmupMs() {
  return static_cast<uint32_t>(STATEFUL_CACHE_WARMUP_MS);
}

uint32_t ampCountStableRequiredMs() {
  return static_cast<uint32_t>(AMP_COUNT_STABLE_REQUIRED_MS);
}

void noteCacheHitByField(FieldId fieldId, DebugCounters& counters) {
  switch (fieldId) {
    case FieldId::settings_power:
      ++counters.cache_hit_power;
      ++counters.cache_hit_settings_power;
      break;
    case FieldId::config_mute_a:
      ++counters.cache_hit_mute_a;
      break;
    case FieldId::config_mute_b:
      ++counters.cache_hit_mute_b;
      break;
    case FieldId::config_gain_a:
      ++counters.cache_hit_gain_a;
      break;
    case FieldId::config_gain_b:
      ++counters.cache_hit_gain_b;
      break;
    case FieldId::status_pwr_ok:
      ++counters.cache_hit_status_pwr_ok;
      break;
    case FieldId::error_general:
      ++counters.cache_hit_error_general;
      break;
    case FieldId::chstatus_isp_a:
      ++counters.cache_hit_isp_a;
      break;
    case FieldId::chstatus_isp_b:
      ++counters.cache_hit_isp_b;
      break;
    case FieldId::chstatus_remhold_gr_a:
      ++counters.cache_hit_remhold_gr_a;
      break;
    case FieldId::chstatus_remhold_gr_b:
      ++counters.cache_hit_remhold_gr_b;
      break;
    case FieldId::chstatus_remhold_ovl_a:
      ++counters.cache_hit_remhold_ovl_a;
      break;
    case FieldId::chstatus_remhold_ovl_b:
      ++counters.cache_hit_remhold_ovl_b;
      break;
    case FieldId::cherr_channel_err_a:
      ++counters.cache_hit_cherr_a;
      break;
    case FieldId::cherr_channel_err_b:
      ++counters.cache_hit_cherr_b;
      break;
    default:
      break;
  }
}

bool forwardCanFrameToTcp(TcpR1Server& tcpServer,
                          GatewayState& state,
                          DebugCli& debugCli,
                          DebugCounters& counters,
                          const NormalizedRequest& request,
                          uint32_t nowMs) {
  if (!tcpServer.connected()) {
    ++counters.can_rx_forward_failed;
    return false;
  }

  uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
  uint8_t tcpFrameLength = 0;
  if (!R70Frame::encodeWrappedCanFrame(request.frame, tcpFrame, sizeof(tcpFrame), tcpFrameLength)) {
    ++counters.can_rx_forward_failed;
    return false;
  }

  debugCli.logTcpCanRxForward(request);
  debugCli.logTcpTxWrappedCan(request.frame, tcpFrame, tcpFrameLength);
  if (!tcpServer.sendFrame(tcpFrame, tcpFrameLength, &counters)) {
    ++counters.can_rx_forward_failed;
    return false;
  }

  ++counters.can_rx_forwarded_to_tcp;
  state.noteTcpTxToNode(request.canId.subnet, request.canId.device, nowMs);
  return true;
}

uint32_t psramTotalBytes() {
#if defined(ARDUINO_ARCH_ESP32)
  return psramFound() ? heap_caps_get_total_size(MALLOC_CAP_SPIRAM) : 0;
#else
  return 0;
#endif
}

uint32_t psramFreeBytes() {
#if defined(ARDUINO_ARCH_ESP32)
  return psramFound() ? heap_caps_get_free_size(MALLOC_CAP_SPIRAM) : 0;
#else
  return 0;
#endif
}

uint16_t normalizedObjectKey(const NormalizedRequest& request) {
  if (!request.valid || request.frame.length < 3) {
    return 0xFFFF;
  }
  return static_cast<uint16_t>((static_cast<uint16_t>(request.frame.data[1]) << 8) | request.frame.data[2]);
}

uint16_t semanticMirrorDisplayObjectKey(uint16_t objectKey) {
  switch (objectKey) {
    case 0x2205:
      return 0x0205;
    case 0x2206:
      return 0x0206;
    default:
      return objectKey;
  }
}

bool isSemanticMirrorFocusObjectKey(uint16_t objectKey) {
  switch (semanticMirrorDisplayObjectKey(objectKey)) {
    case 0x0100:
    case 0x010D:
    case 0x0111:
    case 0x000B:
    case 0x0205:
    case 0x0206:
    case 0x0300:
    case 0x2400:
    case 0x2600:
    case 0x4205:
    case 0x4206:
    case 0x4400:
    case 0x4600:
    case 0x241D:
    case 0x241E:
    case 0x441D:
    case 0x441E:
      return true;
    default:
      return false;
  }
}

bool isSemanticCandidateStatusObjectKey(uint16_t objectKey) {
  switch (semanticMirrorDisplayObjectKey(objectKey)) {
    case 0x0300:
    case 0x0500:
    case 0x0600:
    case 0x2600:
    case 0x4600:
      return true;
    default:
      return false;
  }
}

bool isIdentPresenceReply(const NormalizedRequest& request) {
  return request.valid &&
         request.canId.objectClass == 2 &&
         request.canId.device != 0 &&
         request.frame.length == 8 &&
         request.frame.data[0] == 0x00 &&
         request.frame.data[1] == 0xDB;
}

bool isUnknown0111ReadRequest(const uint8_t* data, uint8_t dlc) {
  return data != 0 &&
         dlc == 3 &&
         data[0] == 0x01 &&
         data[1] == 0x01 &&
         data[2] == 0x11;
}

bool isUnknown0111CanReply(const uint8_t* data, uint8_t dlc) {
  return data != 0 &&
         dlc >= 4 &&
         data[0] == 0x01 &&
         data[1] == 0x01 &&
         data[2] == 0x11;
}

bool isUnknown0111ReadRequest(const NormalizedRequest& request) {
  return request.valid && isUnknown0111ReadRequest(request.frame.data, request.frame.length);
}

bool isUnknown0111CanReply(const NormalizedRequest& request) {
  return request.valid && isUnknown0111CanReply(request.frame.data, request.frame.length);
}

uint8_t unknown0111Value(const Frame& frame) {
  return frame.length >= 4 ? frame.data[3] : 0;
}

enum BootstrapObjectIndex : uint8_t {
  kBootstrap0100 = 0,
  kBootstrap010D = 1,
  kBootstrap0111 = 2,
  kBootstrap0300 = 3,
  kBootstrap0500 = 4,
  kBootstrap2205 = 5,
  kBootstrap4205 = 6,
  kBootstrap2206 = 7,
  kBootstrap4206 = 8,
  kBootstrap2400 = 9,
  kBootstrap4400 = 10,
  kBootstrap2600 = 11,
  kBootstrap4600 = 12,
  kBootstrap2214 = 13,
  kBootstrap4214 = 14,
  kBootstrap2215 = 15,
  kBootstrap4215 = 16,
};

enum StartupBlockerFamilyIndex : uint8_t {
  kStartupBlockerPowerStatus = 0,
  kStartupBlockerMuteGain = 1,
  kStartupBlockerErrorStatus = 2,
  kStartupBlockerChannelStatus = 3,
  kStartupBlockerSpeakerAndNameBase = 4,
  kStartupBlockerSpeakerAndNameChannel = 5,
};

struct BootstrapRequestSpec {
  uint16_t objectKey;
  uint8_t payload[4];
  uint8_t payloadLength;
  bool segmented;
  const char* label;
};

static const BootstrapRequestSpec kBootstrapRequestSpecs[AmpState::kBootstrapObjectCount] = {
    {0x0100, {0x01, 0x01, 0x00, 0x00}, 3, false, "0100"},
    {0x010D, {0x11, 0x01, 0x0D, 0x00}, 4, true, "010D"},
    {0x0111, {0x01, 0x01, 0x11, 0x00}, 3, false, "0111"},
    {0x0300, {0x01, 0x03, 0x00, 0x00}, 3, false, "0300"},
    {0x0500, {0x01, 0x05, 0x00, 0x00}, 3, false, "0500"},
    {0x2205, {0x01, 0x22, 0x05, 0x00}, 3, false, "2205"},
    {0x4205, {0x01, 0x42, 0x05, 0x00}, 3, false, "4205"},
    {0x2206, {0x01, 0x22, 0x06, 0x00}, 3, false, "2206"},
    {0x4206, {0x01, 0x42, 0x06, 0x00}, 3, false, "4206"},
    {0x2400, {0x01, 0x24, 0x00, 0x00}, 3, false, "2400"},
    {0x4400, {0x01, 0x44, 0x00, 0x00}, 3, false, "4400"},
    {0x2600, {0x01, 0x26, 0x00, 0x00}, 3, false, "2600"},
    {0x4600, {0x01, 0x46, 0x00, 0x00}, 3, false, "4600"},
    {0x2214, {0x01, 0x22, 0x14, 0x00}, 3, false, "2214"},
    {0x4214, {0x01, 0x42, 0x14, 0x00}, 3, false, "4214"},
    {0x2215, {0x01, 0x22, 0x15, 0x00}, 3, false, "2215"},
    {0x4215, {0x01, 0x42, 0x15, 0x00}, 3, false, "4215"},
};

static const uint32_t kBootstrapAlwaysRequiredMask =
    (1u << kBootstrap0100) |
    (1u << kBootstrap0111) |
    (1u << kBootstrap0300) |
    (1u << kBootstrap0500) |
    (1u << kBootstrap2205) |
    (1u << kBootstrap4205) |
    (1u << kBootstrap2206) |
    (1u << kBootstrap4206) |
    (1u << kBootstrap2400) |
    (1u << kBootstrap4400) |
    (1u << kBootstrap2600) |
    (1u << kBootstrap4600);
static const uint32_t kBootstrapOptionalMask =
    (1u << kBootstrap010D) |
    (1u << kBootstrap2214) |
    (1u << kBootstrap4214) |
    (1u << kBootstrap2215) |
    (1u << kBootstrap4215);
static const uint32_t kBootstrapChannelRequiredMask =
    (1u << kBootstrap2205) |
    (1u << kBootstrap4205) |
    (1u << kBootstrap2206) |
    (1u << kBootstrap4206) |
    (1u << kBootstrap2400) |
    (1u << kBootstrap4400) |
    (1u << kBootstrap2600) |
    (1u << kBootstrap4600);
static const uint16_t kBootstrapPrimaryCanIdBase = 0x200u;

uint32_t bootstrapMaskFor(uint8_t index) {
  return static_cast<uint32_t>(1u << index);
}

const BootstrapRequestSpec* bootstrapSpec(uint8_t index) {
  return index < AmpState::kBootstrapObjectCount ? &kBootstrapRequestSpecs[index] : 0;
}

const char* bootstrapLabel(uint8_t index) {
  const BootstrapRequestSpec* spec = bootstrapSpec(index);
  return spec != 0 ? spec->label : "unknown";
}

uint16_t bootstrapObjectKey(uint8_t index) {
  const BootstrapRequestSpec* spec = bootstrapSpec(index);
  return spec != 0 ? spec->objectKey : 0;
}

bool classifyBootstrapReply(const NormalizedRequest& reply, uint8_t& bootstrapIndex) {
  bootstrapIndex = 0;
  // CAN replies are not guaranteed to retain the request-side ReadObject
  // function classification.  A successfully normalized reply with the
  // expected object (and segment, where applicable) is sufficient proof that
  // the bootstrap value was observed. Request tracking applies its own strict
  // read-function check before calling this classifier.
  if (!reply.valid || reply.canId.device == 0) {
    return false;
  }
  const uint16_t objectKey = normalizedObjectKey(reply);
  if (isUnknown0111CanReply(reply) || objectKey == 0x0111) {
    bootstrapIndex = kBootstrap0111;
    return true;
  }
  for (uint8_t index = 0; index < AmpState::kBootstrapObjectCount; ++index) {
    const BootstrapRequestSpec& spec = kBootstrapRequestSpecs[index];
    if (spec.objectKey != objectKey) {
      continue;
    }
    if (spec.segmented) {
      if (!reply.objectAddress.hasSegment || reply.objectAddress.segmentNumber != spec.payload[3]) {
        return false;
      }
    }
    bootstrapIndex = index;
    return true;
  }
  return false;
}

const char* gatewayProfileName(GatewayProfile profile) {
  switch (profile) {
    case GatewayProfile::PROFILE_GATE31_STABLE:
      return "PROFILE_GATE31_STABLE";
    case GatewayProfile::PROFILE_CUSTOM:
    default:
      return "PROFILE_CUSTOM";
  }
}

}  // namespace

R70GatewayV2::R70GatewayV2()
    : started_(false),
      log_(0),
      cliStream_(0),
      cliBufferLength_(0),
      tcpBackpressureActive_(false),
      tcpBackpressureBurstLimited_(false),
      tcpBackpressureTotalLimited_(false),
      tcpBackpressureStartMs_(0),
      tcpBackpressureWindowStartMs_(0),
      tcpBackpressureWindowUsedMs_(0),
      canTxQueueHighWater_(0),
      canTxQueueLowWater_(0),
      lastStateSummaryMs_(0),
      lastSummaryCanTxFrames_(0),
      lastSummaryCanRxFrames_(0),
      lastSummaryTcpRxFrames_(0),
      lastSummaryTcpTxFrames_(0),
      lastSummaryMuteWriteTxCount_(0),
      lastSummaryMuteWriteAckCount_(0),
      lastSummaryMuteWriteTimeoutCount_(0),
      lastSummaryRx441_(0),
      lastSummaryRx442_(0),
      lastLongRunSummaryMs_(0),
      lastLongRunQueueFullTotal_(0),
      lastLongRunTcpBackpressureActiveMs_(0),
      lastLongRunCacheRepliesSent_(0),
      lastLongRunCanTxFrames_(0),
      lastLongRunCanRxFrames_(0),
      lastRuntimeBaselineSummaryMs_(0),
      runtimeStableCandidateSinceMs_(0),
      runtimeStableEnteredMs_(0),
      longRunNormalOldestMin_(0),
      longRunNormalOldestMax_(0),
      longRunNormalOldestSum_(0),
      longRunNormalOldestSamples_(0),
      longRunNormalDepthMax_(0),
      longRunCanTxDepthMax_(0),
      lastAutoGroupTimeoutSnapshotMs_(0),
      lastPollMicros_(0),
      lastTcpPollMicros_(0),
      lastCanPollMicros_(0),
      loopMaxUsWindow_(0),
      loopSumUsWindow_(0),
      loopSamplesWindow_(0),
      canPollMaxGapUsWindow_(0),
      tcpPollMaxGapUsWindow_(0),
      maxSerialPrintBurstMsWindow_(0),
      serialBytesPrintedEstimateWindow_(0),
      canResponseLatencySumMs_(0),
      canResponseLatencyCount_(0),
      lastPrintedCanResponseOver1000Ms_(0),
      canResponseOver1000GrowthStreak_(0),
      lastIdentTxMs_(0),
      lastTcpConnected_(false),
      r1ConnectedSinceMs_(0),
      bootstrapProtectionUntilMs_(0),
      bootstrapCompleteStableSinceMs_(0),
      bootstrapRecoveryWindowStartedMs_(0),
      bootstrapRecoverySentInWindow_(0),
      powerOnBootstrapRecoveryNextAmp_(0),
      lastActiveAmpCount_(0),
      firstAmpDiscoveryMs_(0),
      lastAmpDiscoveryMs_(0),
      lastActiveAmpCountChangeMs_(0),
      lastRuntimePendingTcpReplies_(0),
      lastRuntimePendingCanReads_(0),
      lastRuntimeMissingStartupBlockers_(0),
      lastRuntimeBootstrapNodesIncomplete_(0),
      lastRuntimeR1TimeoutEventsSnapshot_(0),
      runtimeStableEntered_(false),
      gentleStatusShapingRuntimeEnabled_(ENABLE_GENTLE_STATUS_SHAPING != 0),
      bootstrapProtectionRuntimeEnabled_(ENABLE_BOOTSTRAP_PROTECTION != 0),
      bootstrapRetryRuntimeEnabled_(ENABLE_BOOTSTRAP_RETRY != 0),
      segmentedDampeningRuntimeEnabled_(ENABLE_SEGMENTED_DAMPENING != 0),
      fanoutAttachRuntimeEnabled_(ENABLE_FANOUT_ATTACH != 0),
      postBootstrapStatusCacheRuntimeEnabled_(ENABLE_POST_BOOTSTRAP_STATUS_CACHE != 0),
      gentleStatusShapingHasEverBeenActive_(false),
      gentleStatusShapingLastEnabled_(false),
      statefulStatusCacheHasEverBeenActive_(false),
      statefulStatusCacheLastEnabled_(false),
      lastR1TimeoutEvents_(0),
      r1TimeoutEventsAtGentleStatusShapingEnable_(0),
      r1TimeoutEventsAtStatefulCacheEnable_(0),
      lastChannelIncompleteCount_(0),
      lastSubnetServiceGapCount_(0),
      statefulStatusCacheDisabledReasonCode_(kStatefulStatusCacheFlagOff),
      semanticCandidateEvictions_(0),
      semanticCandidateLastSummaryMs_(0),
      lastSlowOver500Ms_(0),
      fairnessLastNodeKey_(0xFFFFu),
      fairnessCurrentNodeStreak_(0),
      fairnessLastObjectKey_(0xFFFFu),
      fairnessCurrentObjectStreak_(0),
      nextRequestTraceId_(1u),
      aliveDiagnosticTraces_(0),
      aliveDiagnosticTraceHead_(0u),
      timeoutWindowDumpActive_(false),
      timeoutWindowDumpEventMs_(0),
      timeoutWindowDumpDueMs_(0),
      timeoutWindowDumpCount_(0) {
  counters_.clear();
  slowCanWindow_ = {};
  clearSoftInflightCollapseEntries();
  clearGentleStatusShapingEntries();
  clearDeferredSegmentedRequests();
  clearDeferredIdentRequests();
  resetActiveShapingCadence();
  clearStatefulStatusCache();
  resetStatefulStatusCacheGroupWindows();
  clearSemanticMirror();
  clearSemanticCandidates();
  clearPresenceRounds();
  resetObjectStats();
  resetRequestTraces();
  for (uint16_t index = 0; index < GatewayState::kMaxAddressSlots; ++index) {
    lastNodeCanTxMs_[index] = 0;
  }
  for (uint8_t index = 0; index < kSlowObjectBucketCapacity; ++index) {
    objectFairnessStates_[index] = {};
  }
  for (uint8_t index = 0; index < DebugCounters::kSubnetCount; ++index) {
    lastSummaryIdentRxCountBySubnet_[index] = 0;
  }
}

void R70GatewayV2::clearSemanticCandidates() {
  for (uint8_t index = 0; index < kSemanticCandidateCapacity; ++index) {
    semanticCandidates_[index] = {};
  }
  semanticCandidateEvictions_ = 0;
  semanticCandidateLastSummaryMs_ = 0;
}

void R70GatewayV2::clearPresenceRounds() {
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    presenceRounds_[subnet] = {};
  }
}

void R70GatewayV2::startPresenceRound(const NormalizedRequest& request, uint32_t nowMs) {
  if (!request.valid || request.canId.subnet >= DebugCounters::kSubnetCount) {
    return;
  }
  PresenceRoundState& round = presenceRounds_[request.canId.subnet];
  // A scan queued by this recovery logic belongs to the current round and
  // must not erase the original seen/missing masks.
  if (round.active && round.retryQueued && !round.retrySent &&
      round.scanFrame.canId == request.frame.canId &&
      round.scanFrame.length == request.frame.length &&
      memcmp(round.scanFrame.data, request.frame.data, request.frame.length) == 0) {
    round.retrySent = true;
    round.retrySentMs = nowMs;
    round.evaluateAfterMs = nowMs + kPresenceRoundWaitMs;
    return;
  }

  // R1 already performs dense discovery scans while the initial object set is
  // still filling. Recovery is only useful after every active node completed
  // bootstrap; otherwise it amplifies the startup burst.
  if (bootstrapCompleteStableSinceMs_ == 0) {
    round = {};
    return;
  }

  round = {};
  round.scanFrame = request.frame;
  round.startedMs = nowMs;
  round.evaluateAfterMs = nowMs + kPresenceRoundWaitMs;
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* amp = state_.activeAmpByIndex(ampIndex);
    if (amp == 0 || !amp->used || amp->subnet != request.canId.subnet ||
        amp->device == 0 || amp->device > 64) {
      continue;
    }
    round.expectedMask |= (uint64_t(1) << (amp->device - 1u));
  }
  round.active = round.expectedMask != 0;
}

void R70GatewayV2::notePresenceReply(const NormalizedRequest& reply, uint32_t nowMs) {
  (void)nowMs;
  if (!isIdentPresenceReply(reply) || reply.canId.subnet >= DebugCounters::kSubnetCount ||
      reply.canId.device == 0 || reply.canId.device > 64) {
    return;
  }
  PresenceRoundState& round = presenceRounds_[reply.canId.subnet];
  if (!round.active) {
    return;
  }
  round.seenMask |= (uint64_t(1) << (reply.canId.device - 1u));
}

void R70GatewayV2::processPresenceRetries(uint32_t nowMs) {
  if (bootstrapCompleteStableSinceMs_ == 0) {
    clearPresenceRounds();
    return;
  }
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    PresenceRoundState& round = presenceRounds_[subnet];
    if (!round.active || static_cast<int32_t>(nowMs - round.evaluateAfterMs) < 0) {
      continue;
    }

    const uint64_t missing = round.expectedMask & ~round.seenMask;
    if (missing == 0) {
      if (round.retrySent) {
        char line[160] = {};
        snprintf(line,
                 sizeof(line),
                 "IDENT_RETRY_RECOVERED subnet=%u latency_ms=%lu",
                 static_cast<unsigned>(subnet),
                 static_cast<unsigned long>(nowMs - round.retrySentMs));
        debugCli_.logText(line);
      }
      round.active = false;
      continue;
    }

    if (round.retrySent) {
      for (uint8_t device = 1; device <= 64; ++device) {
        const uint64_t bit = uint64_t(1) << (device - 1u);
        if ((missing & bit) == 0) {
          continue;
        }
        char line[144] = {};
        snprintf(line,
                 sizeof(line),
                 "IDENT_RETRY_FAILED node=%u.%02u",
                 static_cast<unsigned>(subnet),
                 static_cast<unsigned>(device));
        debugCli_.logText(line);
      }
      round.active = false;
      continue;
    }

    if (round.retryQueued) {
      // enqueue() may coalesce an identical request while it is waiting. Do
      // not enqueue or log it again on every gateway loop iteration.
      if (nowMs - round.startedMs >= kPresenceRetryDecisionMaxMs) {
        char line[128] = {};
        snprintf(line,
                 sizeof(line),
                 "IDENT_RETRY_SKIPPED subnet=%u reason=queue_wait_timeout",
                 static_cast<unsigned>(subnet));
        debugCli_.logText(line);
        round.active = false;
      }
      continue;
    }

    uint64_t eligibleMissing = 0;
    for (uint8_t device = 1; device <= 64; ++device) {
      const uint64_t bit = uint64_t(1) << (device - 1u);
      if ((missing & bit) == 0) {
        continue;
      }
      const AmpState* amp = state_.amp(subnet, device);
      if (amp != 0 && amp->lastCanRxMs != 0 && nowMs >= amp->lastCanRxMs &&
          nowMs - amp->lastCanRxMs <= kPresenceRecentCanMaxAgeMs) {
        eligibleMissing |= bit;
      }
    }
    if (eligibleMissing == 0) {
      round.active = false;
      continue;
    }

    const bool pressure = scheduler_.queueDepth() > kPresenceRetryMaxQueueDepth ||
                          currentBusUtilAvgPercent(nowMs) >= kPresenceRetryMaxBusUtilPercent;
    if (pressure) {
      if (nowMs - round.startedMs < kPresenceRetryDecisionMaxMs) {
        round.evaluateAfterMs = nowMs + 100u;
        continue;
      }
      char line[144] = {};
      snprintf(line,
               sizeof(line),
               "IDENT_RETRY_SKIPPED subnet=%u reason=pressure queue=%u bus_util=%lu",
               static_cast<unsigned>(subnet),
               static_cast<unsigned>(scheduler_.queueDepth()),
               static_cast<unsigned long>(currentBusUtilAvgPercent(nowMs)));
      debugCli_.logText(line);
      round.active = false;
      continue;
    }

    NormalizedRequest retry = {};
    if (!codec::normalizeCanFrame(round.scanFrame, RequestOrigin::Tcp, retry)) {
      round.active = false;
      continue;
    }
    const char* failureReason = 0;
    if (!scheduler_.enqueue(retry, 0, false, nowMs, counters_, failureReason)) {
      round.evaluateAfterMs = nowMs + 100u;
      continue;
    }
    round.retryQueued = true;
    round.evaluateAfterMs = nowMs + kPresenceRetryDecisionMaxMs;
    round.missingAtRetryMask = eligibleMissing;
    for (uint8_t device = 1; device <= 64; ++device) {
      const uint64_t bit = uint64_t(1) << (device - 1u);
      if ((eligibleMissing & bit) == 0) {
        continue;
      }
      const AmpState* amp = state_.amp(subnet, device);
      const uint32_t ageMs = amp != 0 && nowMs >= amp->lastCanRxMs ? nowMs - amp->lastCanRxMs : 0u;
      char line[160] = {};
      snprintf(line,
               sizeof(line),
               "IDENT_RETRY node=%u.%02u recent_can_age_ms=%lu",
               static_cast<unsigned>(subnet),
               static_cast<unsigned>(device),
               static_cast<unsigned long>(ageMs));
      debugCli_.logText(line);
    }
  }
}

void R70GatewayV2::noteSemanticCandidate(uint8_t subnet,
                                         uint8_t device,
                                         uint8_t channel,
                                         uint16_t objectKey,
                                         const uint8_t* payload,
                                         uint8_t payloadLength,
                                         uint32_t nowMs,
                                         bool r1Requested,
                                         const char* reason) {
  if (objectKey == 0x0111 || payload == 0 || payloadLength == 0) {
    return;
  }

  uint32_t hash = 2166136261u;
  for (uint8_t index = 0; index < payloadLength; ++index) {
    hash ^= payload[index];
    hash *= 16777619u;
  }

  SemanticCandidateEntry* entry = 0;
  SemanticCandidateEntry* reusable = 0;
  SemanticCandidateEntry* oldest = &semanticCandidates_[0];
  for (uint8_t index = 0; index < kSemanticCandidateCapacity; ++index) {
    SemanticCandidateEntry& candidate = semanticCandidates_[index];
    if (candidate.used && candidate.objectKey == objectKey && candidate.channel == channel) {
      entry = &candidate;
      break;
    }
    if (!candidate.used && reusable == 0) {
      reusable = &candidate;
    }
    if (candidate.used && candidate.lastSeenMs < oldest->lastSeenMs) {
      oldest = &candidate;
    }
  }
  const bool firstSeen = entry == 0;
  if (entry == 0) {
    entry = reusable != 0 ? reusable : oldest;
    if (entry->used) {
      ++semanticCandidateEvictions_;
    }
    *entry = {};
    entry->used = true;
    entry->channel = channel;
    entry->objectKey = objectKey;
    entry->payloadHash = hash;
    entry->firstSeenMs = nowMs;
  }

  const bool changed = !firstSeen && entry->payloadHash != hash;
  entry->lastSubnet = subnet;
  entry->lastDevice = device;
  entry->lastSeenMs = nowMs;
  if (entry->seenCount != 0xFFFFFFFFu) {
    ++entry->seenCount;
  }
  if (r1Requested && entry->r1RequestedCount != 0xFFFFu) {
    ++entry->r1RequestedCount;
  }
  if (changed) {
    entry->payloadHash = hash;
    if (entry->changeCount != 0xFFFFu) {
      ++entry->changeCount;
    }
  }

  if (!firstSeen && (!changed || nowMs - entry->lastLogMs < kSemanticCandidateLogIntervalMs)) {
    return;
  }
  entry->lastLogMs = nowMs;
  char line[224] = {};
  snprintf(line,
           sizeof(line),
           "SEMANTIC_CANDIDATE node=%u.%02u object=%04X channel=%u reason=%s r1_requested=%u changed=%u payload_hash=%08lX seen=%lu changes=%u",
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned>(objectKey),
           static_cast<unsigned>(channel),
           reason != 0 ? reason : "unknown",
           static_cast<unsigned>(r1Requested ? 1u : 0u),
           static_cast<unsigned>(changed ? 1u : 0u),
           static_cast<unsigned long>(hash),
           static_cast<unsigned long>(entry->seenCount),
           static_cast<unsigned>(entry->changeCount));
  debugCli_.logText(line);
}

void R70GatewayV2::logSemanticCandidateSummary(uint32_t nowMs) {
  if (semanticCandidateLastSummaryMs_ != 0 &&
      nowMs - semanticCandidateLastSummaryMs_ < kSemanticCandidateSummaryIntervalMs) {
    return;
  }
  semanticCandidateLastSummaryMs_ = nowMs;
  uint8_t active = 0;
  uint32_t seen = 0;
  uint32_t changes = 0;
  uint32_t r1Requested = 0;
  for (uint8_t index = 0; index < kSemanticCandidateCapacity; ++index) {
    const SemanticCandidateEntry& entry = semanticCandidates_[index];
    if (!entry.used) {
      continue;
    }
    ++active;
    seen += entry.seenCount;
    changes += entry.changeCount;
    r1Requested += entry.r1RequestedCount;
  }
  char line[192] = {};
  snprintf(line,
           sizeof(line),
           "SEMANTIC_CANDIDATE_SUMMARY active=%u capacity=%u seen=%lu changes=%lu r1_requested=%lu evictions=%lu ram_bytes=%u",
           static_cast<unsigned>(active),
           static_cast<unsigned>(kSemanticCandidateCapacity),
           static_cast<unsigned long>(seen),
           static_cast<unsigned long>(changes),
           static_cast<unsigned long>(r1Requested),
           static_cast<unsigned long>(semanticCandidateEvictions_),
           static_cast<unsigned>(sizeof(semanticCandidates_)));
  debugCli_.logText(line);
}

void R70GatewayV2::resetObjectStats() {
  for (uint8_t index = 0; index < kObjectStatCapacity; ++index) {
    objectStats_[index] = {};
  }
}

void R70GatewayV2::resetRequestTraces() {
  recentRequestTraceHead_ = 0;
  nextRequestTraceId_ = 1u;
  aliveDiagnosticTraceHead_ = 0;
  timeoutWindowDumpActive_ = false;
  timeoutWindowDumpEventMs_ = 0;
  timeoutWindowDumpDueMs_ = 0;
  timeoutWindowDumpCount_ = 0;
  for (uint8_t index = 0; index < 8; ++index) {
    timeoutWindowDumpSubnets_[index] = 0;
    timeoutWindowDumpDevices_[index] = 0;
  }
  for (uint16_t index = 0; index < kRecentRequestTraceCapacity; ++index) {
    recentRequestTraces_[index] = {};
  }
  if (aliveDiagnosticTraces_ != 0) {
    for (uint16_t index = 0; index < kAliveDiagnosticTraceCapacity; ++index) {
      aliveDiagnosticTraces_[index] = {};
    }
  }
}

void R70GatewayV2::clearSoftInflightCollapseEntries() {
  for (uint8_t index = 0; index < kSoftInflightCollapseCapacity; ++index) {
    softInflightCollapse_[index] = {};
  }
}

void R70GatewayV2::clearDeferredSegmentedRequests() {
  for (uint8_t index = 0; index < kDeferredSegmentedCapacity; ++index) {
    deferredSegmented_[index] = {};
  }
}

void R70GatewayV2::clearDeferredIdentRequests() {
  for (uint8_t index = 0; index < kDeferredIdentCapacity; ++index) {
    deferredIdent_[index] = {};
  }
}

void R70GatewayV2::resetActiveShapingCadence() {
  for (uint16_t nodeIndex = 0; nodeIndex < GatewayState::kMaxAddressSlots; ++nodeIndex) {
    for (uint8_t groupIndex = 0; groupIndex < kActiveShapingCadenceGroupCount; ++groupIndex) {
      activeShapingCadence_[nodeIndex][groupIndex] = {};
    }
  }
}

uint32_t R70GatewayV2::activeFanoutWaiterCount() const {
  uint32_t count = 0;
  for (uint8_t index = 0; index < kSoftInflightCollapseCapacity; ++index) {
    const SoftInflightCollapseEntry& entry = softInflightCollapse_[index];
    if (!entry.used) {
      continue;
    }
    count += entry.waiterCount;
  }
  return count;
}

bool R70GatewayV2::fanoutAttachFeatureEnabled() const {
  return fanoutAttachRuntimeEnabled_ && config_.enableSoftInflightCollapse;
}

bool R70GatewayV2::segmentedDampeningFeatureEnabled() const {
  return segmentedDampeningRuntimeEnabled_;
}

bool R70GatewayV2::tryQueueDeferredSegmentedRequest(const NormalizedRequest& request, uint32_t nowMs) {
  if (!segmentedDampeningFeatureEnabled() || !isSegmentedReadRequest(request)) {
    return false;
  }
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  ++slowCanWindow_.segmentedGuardCandidates;
  const bool nameSegment = isNameSegmentReadRequest(request);
  const uint8_t queueThreshold = nameSegment ? kNameSegmentGuardQueueThreshold : kSegmentedGuardQueueThreshold;
  const uint8_t utilThreshold = nameSegment ? kNameSegmentGuardUtilThresholdPercent : kSegmentedGuardUtilThresholdPercent;
  const uint32_t queueDepth = scheduler_.normalQueueDepth();
  const uint32_t utilAvgPercent = currentBusUtilAvgPercent(nowMs);
  const bool queueBlocked = queueDepth >= queueThreshold;
  const bool utilBlocked = utilAvgPercent >= utilThreshold;
  const bool slowBlocked = slow500GuardActive(nowMs);
  if (!queueBlocked && !utilBlocked && !slowBlocked) {
    ++slowCanWindow_.segmentedGuardImmediate;
    return false;
  }

  if (queueBlocked) {
    ++slowCanWindow_.segmentedGuardBlockedByQueue;
  }
  if (utilBlocked) {
    ++slowCanWindow_.segmentedGuardBlockedByUtil;
  }
  if (slowBlocked) {
    ++slowCanWindow_.segmentedGuardBlockedBySlow500;
  }

  DeferredSegmentedRequestEntry* reusable = 0;
  for (uint8_t index = 0; index < kDeferredSegmentedCapacity; ++index) {
    DeferredSegmentedRequestEntry& entry = deferredSegmented_[index];
    if (entry.used && exactWrappedReadMatch(entry.request, request)) {
      if (entry.duplicateCount != 0xFFu) {
        ++entry.duplicateCount;
      }
      ++slowCanWindow_.segmentedGuardCoalesced;
      ++counters_.segmented_requests_suppressed;
      ++counters_.dropped_or_suppressed_requests;
      return true;
    }
    if (!entry.used && reusable == 0) {
      reusable = &entry;
    }
  }
  if (reusable == 0) {
    reusable = &deferredSegmented_[0];
  }
  const uint16_t hashJitterMs = static_cast<uint16_t>(((request.canId.subnet & 0x07u) * 11u +
                                                       (request.canId.device & 0x3Fu) * 5u +
                                                       (normalizedObjectKey(request) & 0x1Fu)) % 20u);
  reusable->used = true;
  reusable->request = request;
  reusable->firstDeferredMs = nowMs;
  reusable->releaseAfterMs = nowMs + (nameSegment ? kNameSegmentGuardBaseDeferMs : kSegmentedGuardBaseDeferMs) +
                             hashJitterMs;
  reusable->duplicateCount = 0;
  ++slowCanWindow_.segmentedGuardDeferred;
  ++counters_.segmented_requests_suppressed;
  ++counters_.dropped_or_suppressed_requests;
  uint8_t pendingCount = 0;
  for (uint8_t index = 0; index < kDeferredSegmentedCapacity; ++index) {
    if (deferredSegmented_[index].used) {
      ++pendingCount;
    }
  }
  if (pendingCount > slowCanWindow_.segmentedGuardPendingMax) {
    slowCanWindow_.segmentedGuardPendingMax = pendingCount;
  }
  return true;
}

bool R70GatewayV2::processDeferredSegmentedRequests(uint32_t nowMs) {
  DeferredSegmentedRequestEntry* best = 0;
  for (uint8_t index = 0; index < kDeferredSegmentedCapacity; ++index) {
    DeferredSegmentedRequestEntry& entry = deferredSegmented_[index];
    if (!entry.used) {
      continue;
    }
    if (best == 0 || entry.firstDeferredMs < best->firstDeferredMs) {
      best = &entry;
    }
  }
  if (best == 0) {
    return false;
  }
  const bool forcedRelease =
      nowMs >= best->firstDeferredMs &&
      static_cast<uint32_t>(nowMs - best->firstDeferredMs) >= kSegmentedGuardMaxDeferMs;
  const bool queueBlocked = scheduler_.normalQueueDepth() >= kSegmentedGuardQueueThreshold;
  const bool utilBlocked = currentBusUtilAvgPercent(nowMs) >= kSegmentedGuardUtilThresholdPercent;
  const bool slowBlocked = slow500GuardActive(nowMs);
  if (!forcedRelease) {
    if (static_cast<int32_t>(nowMs - best->releaseAfterMs) < 0) {
      return false;
    }
    if (queueBlocked || utilBlocked || slowBlocked) {
      return false;
    }
  }
  const char* failureReason = 0;
  if (!scheduler_.enqueue(best->request, 0, false, nowMs, counters_, failureReason)) {
    return false;
  }
  const uint32_t waitMs = nowMs >= best->firstDeferredMs ? nowMs - best->firstDeferredMs : 0u;
  slowCanWindow_.segmentedGuardWaitSumMs += waitMs;
  ++slowCanWindow_.segmentedGuardWaitCount;
  if (waitMs > slowCanWindow_.segmentedGuardWaitMaxMs) {
    slowCanWindow_.segmentedGuardWaitMaxMs = static_cast<uint16_t>(waitMs > 0xFFFFu ? 0xFFFFu : waitMs);
  }
  if (forcedRelease) {
    ++slowCanWindow_.segmentedGuardForcedRelease;
  } else {
    ++slowCanWindow_.segmentedGuardReleased;
  }
  best->used = false;
  return true;
}

bool R70GatewayV2::tryQueueDeferredIdentRequest(const NormalizedRequest& request, uint32_t nowMs) {
  if (!request.valid || !request.tcpWrappedCan || request.canId.device != 0 || request.canId.objectClass != 1 ||
      request.frame.length != 2 || request.frame.data[0] != 0x00 || request.frame.data[1] != 0x01) {
    return false;
  }
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  ++slowCanWindow_.identGuardCandidates;
  const bool queueBlocked = scheduler_.queueDepth() >= kIdentGuardQueueThreshold;
  const bool utilBlocked = currentBusUtilAvgPercent(nowMs) >= kIdentGuardUtilThresholdPercent;
  const bool slowBlocked = slow500GuardActive(nowMs);
  if (!queueBlocked && !utilBlocked && !slowBlocked) {
    ++slowCanWindow_.identGuardImmediate;
    return false;
  }
  if (queueBlocked) {
    ++slowCanWindow_.identGuardBlockedByQueue;
  }
  if (utilBlocked) {
    ++slowCanWindow_.identGuardBlockedByUtil;
  }
  if (slowBlocked) {
    ++slowCanWindow_.identGuardBlockedBySlow500;
  }

  DeferredIdentRequestEntry* reusable = 0;
  for (uint8_t index = 0; index < kDeferredIdentCapacity; ++index) {
    DeferredIdentRequestEntry& entry = deferredIdent_[index];
    if (entry.used &&
        entry.request.frame.canId == request.frame.canId &&
        entry.request.frame.length == request.frame.length &&
        entry.request.frame.data[0] == request.frame.data[0] &&
        entry.request.frame.data[1] == request.frame.data[1]) {
      if (entry.duplicateCount != 0xFFu) {
        ++entry.duplicateCount;
      }
      ++slowCanWindow_.identGuardCoalesced;
      ++counters_.dropped_or_suppressed_requests;
      return true;
    }
    if (!entry.used && reusable == 0) {
      reusable = &entry;
    }
  }
  if (reusable == 0) {
    reusable = &deferredIdent_[0];
  }
  reusable->used = true;
  reusable->request = request;
  reusable->firstDeferredMs = nowMs;
  reusable->releaseAfterMs = nowMs + kIdentGuardBaseDeferMs +
                             static_cast<uint32_t>((request.canId.subnet & 0x07u) * 15u);
  reusable->duplicateCount = 0;
  ++slowCanWindow_.identGuardDeferred;
  ++counters_.dropped_or_suppressed_requests;
  uint8_t pendingCount = 0;
  for (uint8_t index = 0; index < kDeferredIdentCapacity; ++index) {
    if (deferredIdent_[index].used) {
      ++pendingCount;
    }
  }
  if (pendingCount > slowCanWindow_.identGuardPendingMax) {
    slowCanWindow_.identGuardPendingMax = pendingCount;
  }
  return true;
}

bool R70GatewayV2::processDeferredIdentRequests(uint32_t nowMs) {
  DeferredIdentRequestEntry* best = 0;
  for (uint8_t index = 0; index < kDeferredIdentCapacity; ++index) {
    DeferredIdentRequestEntry& entry = deferredIdent_[index];
    if (!entry.used) {
      continue;
    }
    if (best == 0 || entry.firstDeferredMs < best->firstDeferredMs) {
      best = &entry;
    }
  }
  if (best == 0) {
    return false;
  }
  const bool forcedRelease =
      nowMs >= best->firstDeferredMs &&
      static_cast<uint32_t>(nowMs - best->firstDeferredMs) >= kIdentGuardMaxDeferMs;
  const bool queueBlocked = scheduler_.queueDepth() >= kIdentGuardQueueThreshold;
  const bool utilBlocked = currentBusUtilAvgPercent(nowMs) >= kIdentGuardUtilThresholdPercent;
  const bool slowBlocked = slow500GuardActive(nowMs);
  if (!forcedRelease) {
    if (static_cast<int32_t>(nowMs - best->releaseAfterMs) < 0) {
      return false;
    }
    if (queueBlocked || utilBlocked || slowBlocked) {
      return false;
    }
  }
  const char* failureReason = 0;
  if (!scheduler_.enqueue(best->request, 0, false, nowMs, counters_, failureReason)) {
    return false;
  }
  const uint32_t waitMs = nowMs >= best->firstDeferredMs ? nowMs - best->firstDeferredMs : 0u;
  slowCanWindow_.identGuardWaitSumMs += waitMs;
  ++slowCanWindow_.identGuardWaitCount;
  if (waitMs > slowCanWindow_.identGuardWaitMaxMs) {
    slowCanWindow_.identGuardWaitMaxMs = static_cast<uint16_t>(waitMs > 0xFFFFu ? 0xFFFFu : waitMs);
  }
  if (forcedRelease) {
    ++slowCanWindow_.identGuardForcedRelease;
  } else {
    ++slowCanWindow_.identGuardReleased;
  }
  best->used = false;
  return true;
}

R70GatewayV2::RequestTrace* R70GatewayV2::recordRequestTrace(const NormalizedRequest& request,
                                                             uint32_t nowMs,
                                                             uint16_t pendingTcpReplies,
                                                             uint16_t normalQueueDepth,
                                                             uint16_t canTxQueueDepth,
                                                             uint8_t duplicatePendingCount) {
  if (!config_.enablePerNodeRecentRequestRing) {
    return 0;
  }
  if (!request.valid || !isReadFunction(request.admin.functionCode) || request.canId.device == 0) {
    return 0;
  }
  RequestTrace& trace = recentRequestTraces_[recentRequestTraceHead_];
  recentRequestTraceHead_ =
      static_cast<uint16_t>((recentRequestTraceHead_ + 1u) % kRecentRequestTraceCapacity);
  trace = {};
  trace.used = true;
  trace.requestId = nextRequestTraceId_++;
  trace.subnet = request.canId.subnet;
  trace.device = request.canId.device;
  trace.channel = request.objectAddress.channelNumber;
  trace.objectKey = normalizedObjectKey(request);
  trace.fieldId = request.fieldId;
  trace.length = request.frame.length;
  for (uint8_t index = 0; index < request.frame.length && index < sizeof(trace.data); ++index) {
    trace.data[index] = request.frame.data[index];
  }
  trace.timestampMs = nowMs;
  trace.pendingTcpRepliesAtRequest = pendingTcpReplies;
  trace.normalQueueDepthAtRequest = normalQueueDepth;
  trace.canTxQueueDepthAtRequest = canTxQueueDepth;
  trace.duplicatePendingAtRequest = duplicatePendingCount;
  trace.waiterFanoutCount = duplicatePendingCount;
  trace.outstandingObjectCount = 1u;
  trace.outstandingSegmentCount = request.objectAddress.hasSegment ? 1u : 0u;
  trace.source = TraceSource::None;
  trace.mirrorDecision = TraceMirrorDecision::None;
  trace.backgroundRefreshState = TraceBackgroundRefreshState::None;
  trace.pendingState = TracePendingState::Created;
  trace.releaseReason = TraceReleaseReason::None;
  trace.category = classifySlowTraceCategory(request);
  trace.expensiveGroup = classifyExpensiveObjectGroup(request);
  logTcpPendingTrace(trace, "created");
  return &trace;
}

bool R70GatewayV2::exactWrappedReadMatch(const NormalizedRequest& lhs, const NormalizedRequest& rhs) {
  if (!lhs.valid || !rhs.valid || !lhs.admin.valid || !rhs.admin.valid) {
    return false;
  }
  if (!isReadFunction(lhs.admin.functionCode) || !isReadFunction(rhs.admin.functionCode)) {
    return false;
  }
  if (lhs.canId.subnet != rhs.canId.subnet ||
      lhs.canId.device != rhs.canId.device ||
      lhs.admin.telegramType != rhs.admin.telegramType ||
      lhs.objectAddress.channelNumber != rhs.objectAddress.channelNumber ||
      lhs.objectAddress.boxNumber != rhs.objectAddress.boxNumber ||
      lhs.objectAddress.objectNumber != rhs.objectAddress.objectNumber ||
      lhs.fieldId != rhs.fieldId ||
      lhs.frame.length != rhs.frame.length) {
    return false;
  }
  if (lhs.objectAddress.hasRecord != rhs.objectAddress.hasRecord ||
      lhs.objectAddress.hasSegment != rhs.objectAddress.hasSegment) {
    return false;
  }
  if (lhs.objectAddress.hasRecord &&
      lhs.objectAddress.recordNumber != rhs.objectAddress.recordNumber) {
    return false;
  }
  if (lhs.objectAddress.hasSegment &&
      lhs.objectAddress.segmentNumber != rhs.objectAddress.segmentNumber) {
    return false;
  }
  for (uint8_t index = 0; index < lhs.frame.length && index < sizeof(lhs.frame.data); ++index) {
    if (lhs.frame.data[index] != rhs.frame.data[index]) {
      return false;
    }
  }
  return true;
}

bool R70GatewayV2::hasRecentExactBurst(const NormalizedRequest& request,
                                       uint32_t nowMs,
                                       uint32_t windowMs) const {
  if (!config_.enablePerNodeRecentRequestRing) {
    return false;
  }
  for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
    const uint16_t index =
        static_cast<uint16_t>((recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) %
                              kRecentRequestTraceCapacity);
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used) {
      continue;
    }
    if (trace.timestampMs > nowMs ||
        static_cast<int32_t>(nowMs - (trace.timestampMs + windowMs)) >= 0) {
      continue;
    }
    if (trace.subnet != request.canId.subnet ||
        trace.device != request.canId.device ||
        trace.channel != request.objectAddress.channelNumber ||
        trace.objectKey != normalizedObjectKey(request) ||
        trace.fieldId != request.fieldId ||
        trace.length != request.frame.length) {
      continue;
    }
    bool exactBytes = true;
    for (uint8_t byteIndex = 0; byteIndex < trace.length && byteIndex < sizeof(trace.data); ++byteIndex) {
      if (trace.data[byteIndex] != request.frame.data[byteIndex]) {
        exactBytes = false;
        break;
      }
    }
    if (exactBytes) {
      return true;
    }
  }
  return false;
}

R70GatewayV2::RequestTrace* R70GatewayV2::findMatchingPendingTrace(const NormalizedRequest& request) {
  if (!config_.enablePerNodeRecentRequestRing) {
    return 0;
  }
  const uint16_t objectKey = normalizedObjectKey(request);
  for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
    const uint16_t index =
        static_cast<uint16_t>((recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) %
                              kRecentRequestTraceCapacity);
    RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used ||
        trace.subnet != request.canId.subnet ||
        trace.device != request.canId.device ||
        trace.objectKey != objectKey ||
        (trace.flags & kTraceFlagTcpTxSent) != 0) {
      continue;
    }
    if (trace.length > 3 && request.frame.length > 3 && trace.data[3] != request.frame.data[3]) {
      continue;
    }
    return &trace;
  }
  return 0;
}

void R70GatewayV2::markTraceForwardedToCan(RequestTrace* trace) {
  if (trace == 0) {
    return;
  }
  trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagForwardedToCan);
  trace->pendingState = TracePendingState::WaitingCan;
  logTcpPendingTrace(*trace, "waiting_can");
}

void R70GatewayV2::recordAliveDiagnosticTrace(const RequestTrace& trace) {
  if (aliveDiagnosticTraces_ == 0) {
    return;
  }
  AliveDiagnosticTrace& diagnostic = aliveDiagnosticTraces_[aliveDiagnosticTraceHead_];
  aliveDiagnosticTraceHead_ = static_cast<uint16_t>(
      (aliveDiagnosticTraceHead_ + 1u) % kAliveDiagnosticTraceCapacity);
  diagnostic.timestampMs = trace.timestampMs;
  diagnostic.replyTimestampMs = trace.replyTimestampMs;
  diagnostic.objectKey = trace.objectKey;
  diagnostic.latencyMs = trace.latencyMs;
  diagnostic.subnet = trace.subnet;
  diagnostic.device = trace.device;
  diagnostic.source = trace.source;
  diagnostic.channel = trace.channel;
}

void R70GatewayV2::recordAliveDiagnosticCanReply(const NormalizedRequest& reply,
                                                 uint32_t nowMs) {
  if (aliveDiagnosticTraces_ == 0 || !reply.valid ||
      normalizedObjectKey(reply) != 0x0100u) return;
  AliveDiagnosticTrace& diagnostic = aliveDiagnosticTraces_[aliveDiagnosticTraceHead_];
  aliveDiagnosticTraceHead_ = static_cast<uint16_t>(
      (aliveDiagnosticTraceHead_ + 1u) % kAliveDiagnosticTraceCapacity);
  diagnostic.timestampMs = nowMs;
  diagnostic.replyTimestampMs = nowMs;
  diagnostic.objectKey = 0x0100u;
  diagnostic.latencyMs = 0;
  diagnostic.subnet = reply.canId.subnet;
  diagnostic.device = reply.canId.device;
  diagnostic.source = TraceSource::RealCan;
  diagnostic.channel = reply.objectAddress.channelNumber;
}

void R70GatewayV2::markTraceCacheReply(RequestTrace* trace, uint32_t nowMs) {
  if (trace == 0) {
    return;
  }
  trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagCacheReplied | kTraceFlagTcpTxSent);
  trace->source = TraceSource::Cache;
  trace->replyTimestampMs = nowMs;
  trace->latencyMs = static_cast<uint16_t>(nowMs - trace->timestampMs);
  trace->postReplyMs = 0;
  trace->pendingState = TracePendingState::ReplySent;
  trace->releaseReason = TraceReleaseReason::CacheReply;
  trace->outstandingObjectCount = 0;
  trace->outstandingSegmentCount = 0;
  recordAliveDiagnosticTrace(*trace);
  logTcpPendingTrace(*trace, "reply_sent");
}

void R70GatewayV2::markTraceMirrorReply(RequestTrace* trace, uint32_t nowMs) {
  if (trace == 0) {
    return;
  }
  trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagCacheReplied | kTraceFlagTcpTxSent);
  trace->source = TraceSource::Mirror;
  trace->mirrorDecision = TraceMirrorDecision::MirrorServed;
  trace->replyTimestampMs = nowMs;
  trace->latencyMs = static_cast<uint16_t>(nowMs - trace->timestampMs);
  trace->postReplyMs = 0;
  trace->pendingState = TracePendingState::ReplySent;
  trace->releaseReason = TraceReleaseReason::MirrorReply;
  trace->outstandingObjectCount = 0;
  trace->outstandingSegmentCount = 0;
  recordAliveDiagnosticTrace(*trace);
  logTcpPendingTrace(*trace, "reply_sent");
}

void R70GatewayV2::markTraceStaleAtRequest(RequestTrace* trace) {
  if (trace == 0) {
    return;
  }
  trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagStaleAtRequest);
}

void R70GatewayV2::markTraceMirrorForceCan(RequestTrace* trace, TraceMirrorDecision decision) {
  if (trace == 0) {
    return;
  }
  trace->mirrorDecision = decision;
}

void R70GatewayV2::markTraceBackgroundRefresh(RequestTrace* trace, TraceBackgroundRefreshState state) {
  if (trace == 0) {
    return;
  }
  trace->backgroundRefreshState = state;
}

void R70GatewayV2::markTraceCanComplete(RequestTrace* trace, uint32_t nowMs, bool partialSegment) {
  if (trace == 0) {
    return;
  }
  trace->canRxTimestampMs = nowMs;
  trace->pendingState = partialSegment ? TracePendingState::PartialComplete : TracePendingState::CanComplete;
  trace->outstandingSegmentCount = partialSegment ? 1u : 0u;
  logTcpPendingTrace(*trace, partialSegment ? "partial_complete" : "can_complete");
}

void R70GatewayV2::markTraceTimeoutCleanup(RequestTrace* trace,
                                           uint32_t nowMs,
                                           TraceReleaseReason reason) {
  if (trace == 0 || trace->pendingState == TracePendingState::ReplySent ||
      trace->pendingState == TracePendingState::TimeoutCleanup) {
    return;
  }
  trace->cleanupTimestampMs = nowMs;
  trace->pendingState = TracePendingState::TimeoutCleanup;
  trace->releaseReason = reason;
  trace->outstandingObjectCount = 0;
  trace->outstandingSegmentCount = 0;
  logTcpPendingTrace(*trace, "timeout_cleanup");
}

void R70GatewayV2::processTraceTimeoutCleanup(uint32_t nowMs) {
  for (uint16_t index = 0; index < kRecentRequestTraceCapacity; ++index) {
    RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used || trace.pendingState == TracePendingState::ReplySent ||
        trace.pendingState == TracePendingState::TimeoutCleanup) {
      continue;
    }
    if (trace.timestampMs == 0 || nowMs < trace.timestampMs) {
      continue;
    }
    if (static_cast<uint32_t>(nowMs - trace.timestampMs) >= 5000u) {
      markTraceTimeoutCleanup(&trace, nowMs, TraceReleaseReason::AgedOutNoReply);
    }
  }
}

void R70GatewayV2::logTcpPendingTrace(const RequestTrace& trace, const char* stateText) const {
  if (!config_.enableTimeoutCorrelation) {
    return;
  }
  char line[256] = {};
  snprintf(line,
           sizeof(line),
           "TCP_PENDING_TRACE request_id=%lu node=%u.%02u object=%04X state=%s create_ms=%lu can_complete_ms=%lu tcp_send_ms=%lu outstanding_objects=%u outstanding_segments=%u waiter_fanout=%u completion=%s final_release=%s",
           static_cast<unsigned long>(trace.requestId),
           static_cast<unsigned>(trace.subnet),
           static_cast<unsigned>(trace.device),
           static_cast<unsigned>(trace.objectKey),
           stateText != 0 ? stateText : tracePendingStateName(trace.pendingState),
           static_cast<unsigned long>(trace.timestampMs),
           static_cast<unsigned long>(trace.canRxTimestampMs),
           static_cast<unsigned long>(trace.replyTimestampMs),
           static_cast<unsigned>(trace.outstandingObjectCount),
           static_cast<unsigned>(trace.outstandingSegmentCount),
           static_cast<unsigned>(trace.waiterFanoutCount),
           tracePendingStateName(trace.pendingState),
           traceReleaseReasonName(trace.releaseReason));
  debugCli_.logText(line);
}

void R70GatewayV2::markTraceRealCanRx(const NormalizedRequest& reply, uint32_t nowMs) {
  RequestTrace* trace = findMatchingPendingTrace(reply);
  if (trace == 0) {
    return;
  }
  if ((trace->flags & kTraceFlagRealCanRxSeen) != 0) {
    return;
  }
  trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagRealCanRxSeen);
  trace->canRxTimestampMs = nowMs;
  trace->channel = reply.objectAddress.channelNumber;
  const bool partialSegment = isSegmentedType(reply.admin.telegramType);
  trace->pendingState = partialSegment ? TracePendingState::PartialComplete : TracePendingState::CanComplete;
  trace->outstandingSegmentCount = partialSegment ? 1u : 0u;
  if (trace->replyTimestampMs == 0) {
    trace->replyTimestampMs = nowMs;
  }
  if (trace->canTxTimestampMs != 0 && nowMs >= trace->canTxTimestampMs) {
    trace->canRoundtripMs = static_cast<uint16_t>(nowMs - trace->canTxTimestampMs);
  }
  const uint32_t latencyMs = nowMs - trace->timestampMs;
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  slowCanWindow_.canResponseLatencySumMs += latencyMs;
  ++slowCanWindow_.canResponseCount;
  canResponseLatencySumMs_ += latencyMs;
  ++canResponseLatencyCount_;
  if (latencyMs > counters_.max_can_response_latency_ms) {
    counters_.max_can_response_latency_ms = latencyMs;
  }
  counters_.avg_can_response_latency_ms =
      canResponseLatencyCount_ != 0
          ? static_cast<uint32_t>(canResponseLatencySumMs_ / canResponseLatencyCount_)
          : 0;
  if (latencyMs > 500) {
    ++counters_.can_response_over_500ms;
  }
  if (latencyMs > 1000) {
    ++counters_.can_response_over_1000ms;
  }
  noteExpensiveObjectCanRx(*trace);
  logTcpPendingTrace(*trace, partialSegment ? "partial_complete" : "can_complete");
}

void R70GatewayV2::markTraceTcpTx(const NormalizedRequest& reply, TraceSource source, uint32_t nowMs) {
  RequestTrace* trace = findMatchingPendingTrace(reply);
  if (trace == 0) {
    return;
  }
  trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagTcpTxSent);
  trace->source = source;
  trace->replyTimestampMs = nowMs;
  trace->latencyMs = static_cast<uint16_t>(nowMs - trace->timestampMs);
  trace->pendingState = TracePendingState::ReplySent;
  trace->outstandingObjectCount = 0;
  trace->outstandingSegmentCount = 0;
  if (source == TraceSource::RealCan) {
    trace->releaseReason = TraceReleaseReason::RealCanReply;
  } else if (source == TraceSource::Cache) {
    trace->releaseReason = TraceReleaseReason::CacheReply;
  } else if (source == TraceSource::Mirror) {
    trace->releaseReason = TraceReleaseReason::MirrorReply;
  }
  if (trace->canRxTimestampMs != 0 && nowMs >= trace->canRxTimestampMs) {
    trace->postReplyMs = static_cast<uint16_t>(nowMs - trace->canRxTimestampMs);
  }
  AmpState* ampState = state_.amp(reply.canId.subnet, reply.canId.device, false);
  if (ampState != 0 && trace->latencyMs > ampState->maxRequestToTcpTxLatencyMs) {
    ampState->maxRequestToTcpTxLatencyMs = trace->latencyMs;
  }
  if (isUnknown0111CanReply(reply)) {
    ampState = state_.amp(reply.canId.subnet, reply.canId.device, false);
    if (ampState != 0 && trace->latencyMs > ampState->unknown0111MaxLatencyMs) {
      ampState->unknown0111MaxLatencyMs = trace->latencyMs;
    }
  }
  if (source == TraceSource::RealCan &&
      (trace->flags & kTraceFlagRealCanRxSeen) != 0 &&
      (trace->flags & kTraceFlagSlowAttributed) == 0) {
    noteSlowCanResponse(*trace);
    trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagSlowAttributed);
  }
  recordAliveDiagnosticTrace(*trace);
  logTcpPendingTrace(*trace, "reply_sent");
}

void R70GatewayV2::handleSchedulerCanTx(void* context,
                                        const NormalizedRequest& request,
                                        QueueClass queueClass,
                                        uint32_t enqueuedMs,
                                        uint32_t nowMs) {
  if (context == 0) {
    return;
  }
  static_cast<R70GatewayV2*>(context)->noteSchedulerCanTx(request, queueClass, enqueuedMs, nowMs);
}

R70GatewayV2::SlowTraceCategory R70GatewayV2::classifySlowTraceCategory(const NormalizedRequest& request) {
  if (isUnknown0111ReadRequest(request)) {
    return SlowTraceCategory::Unknown0111;
  }
  if (isNameSegmentReadRequest(request)) {
    return SlowTraceCategory::NameSegment;
  }
  if (isFirmwareSegmentReadRequest(request)) {
    return SlowTraceCategory::FirmwareSegment;
  }
  if (isStableSetupReadRequest(request)) {
    return SlowTraceCategory::StableSetup;
  }
  if (request.admin.valid && request.admin.telegramType == TelegramType::Segmented) {
    return SlowTraceCategory::SegmentedRead;
  }
  if (request.admin.valid && request.admin.telegramType == TelegramType::Unstructured) {
    return SlowTraceCategory::UnstructuredRead;
  }
  return SlowTraceCategory::Unknown;
}

const char* R70GatewayV2::slowTraceCategoryName(SlowTraceCategory category) {
  switch (category) {
    case SlowTraceCategory::Unknown0111:
      return "0111";
    case SlowTraceCategory::NameSegment:
      return "name_segment";
    case SlowTraceCategory::FirmwareSegment:
      return "firmware_segment";
    case SlowTraceCategory::StableSetup:
      return "stable_setup";
    case SlowTraceCategory::SegmentedRead:
      return "segmented";
    case SlowTraceCategory::UnstructuredRead:
      return "unstructured";
    case SlowTraceCategory::Unknown:
    default:
      return "unknown";
  }
}

R70GatewayV2::ExpensiveObjectGroup R70GatewayV2::classifyExpensiveObjectGroup(const NormalizedRequest& request) {
  if (!request.valid || !request.admin.valid || !isReadFunction(request.admin.functionCode)) {
    return ExpensiveObjectGroup::None;
  }
  if (isNameSegmentReadRequest(request)) {
    return ExpensiveObjectGroup::NameSegment;
  }
  switch (normalizedObjectKey(request)) {
    case 0x0100:
      return ExpensiveObjectGroup::Object0100;
    case 0x2400:
    case 0x4400:
      return ExpensiveObjectGroup::Object2400_4400;
    case 0x4205:
    case 0x4206:
      return ExpensiveObjectGroup::Object4205_4206;
    default:
      return ExpensiveObjectGroup::None;
  }
}

const char* R70GatewayV2::expensiveObjectGroupName(ExpensiveObjectGroup group) {
  switch (group) {
    case ExpensiveObjectGroup::Object0100:
      return "0100";
    case ExpensiveObjectGroup::Object2400_4400:
      return "2400_4400";
    case ExpensiveObjectGroup::Object4205_4206:
      return "4205_4206";
    case ExpensiveObjectGroup::NameSegment:
      return "name_segment";
    case ExpensiveObjectGroup::None:
    default:
      return "none";
  }
}

R70GatewayV2::ShapingSimGroup R70GatewayV2::classifyShapingSimGroup(const NormalizedRequest& request) {
  if (!request.valid || !request.admin.valid || !isReadFunction(request.admin.functionCode) ||
      request.canId.device == 0) {
    return ShapingSimGroup::None;
  }
  const uint8_t box = request.objectAddress.boxNumber;
  const uint8_t object = request.objectAddress.objectNumber;
  const uint8_t channel = request.objectAddress.channelNumber;
  if (box == 0x04 && object == 0x00 && (channel == 1 || channel == 2)) {
    return ShapingSimGroup::Object2400_4400;
  }
  if (box == 0x02 && (object == 0x05 || object == 0x06) && (channel == 1 || channel == 2)) {
    return object == 0x05 || object == 0x06
               ? (channel == 2 ? ShapingSimGroup::Object4205_4206 : ShapingSimGroup::Object2205_2206)
               : ShapingSimGroup::None;
  }
  if (box == 0x01 && object == 0x00) {
    return ShapingSimGroup::Object0100;
  }
  if (box == 0x04 && (object == 0x1D || object == 0x1E) && (channel == 1 || channel == 2)) {
    return ShapingSimGroup::Object241D_241E_441D_441E;
  }
  if (((box == 0x05 && object == 0x00 && channel == 0) ||
       (box == 0x06 && object == 0x00 && (channel == 1 || channel == 2)))) {
    return ShapingSimGroup::Object0500_2600_4600;
  }
  switch (normalizedObjectKey(request)) {
    case 0x2400:
    case 0x4400:
      return ShapingSimGroup::Object2400_4400;
    case 0x4205:
    case 0x4206:
      return ShapingSimGroup::Object4205_4206;
    case 0x0100:
      return ShapingSimGroup::Object0100;
    case 0x2205:
    case 0x2206:
      return ShapingSimGroup::Object2205_2206;
    case 0x241D:
    case 0x241E:
    case 0x441D:
    case 0x441E:
      return ShapingSimGroup::Object241D_241E_441D_441E;
    case 0x0500:
    case 0x2600:
    case 0x4600:
      return ShapingSimGroup::Object0500_2600_4600;
    default:
      return ShapingSimGroup::None;
  }
}

const char* R70GatewayV2::shapingSimGroupName(ShapingSimGroup group) {
  switch (group) {
    case ShapingSimGroup::Object2400_4400:
      return "2400_4400";
    case ShapingSimGroup::Object4205_4206:
      return "4205_4206";
    case ShapingSimGroup::Object0100:
      return "0100";
    case ShapingSimGroup::Object2205_2206:
      return "2205_2206";
    case ShapingSimGroup::Object241D_241E_441D_441E:
      return "241D_241E_441D_441E";
    case ShapingSimGroup::Object0500_2600_4600:
      return "0500_2600_4600";
    case ShapingSimGroup::None:
    default:
      return "none";
  }
}

uint16_t R70GatewayV2::shapingSimIntervalMs(ShapingSimGroup group) {
  switch (group) {
    case ShapingSimGroup::Object2400_4400:
      return 150u;
    case ShapingSimGroup::Object4205_4206:
      return 150u;
    case ShapingSimGroup::Object0100:
      return 250u;
    case ShapingSimGroup::Object2205_2206:
      return 100u;
    case ShapingSimGroup::Object241D_241E_441D_441E:
      return 150u;
    case ShapingSimGroup::Object0500_2600_4600:
      return 250u;
    case ShapingSimGroup::None:
    default:
      return 0u;
  }
}

uint16_t R70GatewayV2::gentleStatusShapingWindowMs(ShapingSimGroup group) {
  switch (group) {
    case ShapingSimGroup::Object241D_241E_441D_441E:
      return 125u;
    case ShapingSimGroup::Object0500_2600_4600:
      return 150u;
    case ShapingSimGroup::None:
    default:
      return 0u;
  }
}

uint16_t R70GatewayV2::gentleStatusShapingMaxWaitMs(ShapingSimGroup group) {
  return gentleStatusShapingWindowMs(group);
}

uint8_t R70GatewayV2::classifyStartupBlockerFamily(const NormalizedRequest& request) {
  const uint16_t objectKey = normalizedObjectKey(request);
  switch (objectKey) {
    case 0x0100:
    case 0x010D:
    case 0x0111:
    case 0x000B:
      return kStartupBlockerPowerStatus;
    case 0x0205:
    case 0x0206:
    case 0x2205:
    case 0x4205:
    case 0x2206:
    case 0x4206:
      return kStartupBlockerMuteGain;
    case 0x0300:
      return kStartupBlockerErrorStatus;
    case 0x0400:
    case 0x041D:
    case 0x041E:
    case 0x0500:
    case 0x0600:
    case 0x2400:
    case 0x241D:
    case 0x241E:
    case 0x4400:
    case 0x441D:
    case 0x441E:
    case 0x2600:
    case 0x4600:
      return kStartupBlockerChannelStatus;
    case 0x0214:
    case 0x0215:
      return kStartupBlockerSpeakerAndNameBase;
    case 0x2214:
    case 0x2215:
    case 0x4214:
    case 0x4215:
      return kStartupBlockerSpeakerAndNameChannel;
    default:
      return kStartupBlockerFamilyCount;
  }
}

const char* R70GatewayV2::startupBlockerFamilyName(uint8_t family) {
  switch (family) {
    case kStartupBlockerPowerStatus:
      return "0100_010D_0111_000B";
    case kStartupBlockerMuteGain:
      return "0205_0206";
    case kStartupBlockerErrorStatus:
      return "0300";
    case kStartupBlockerChannelStatus:
      return "0400_041D_041E_0500_0600";
    case kStartupBlockerSpeakerAndNameBase:
      return "speaker_id_channel_name_base_0214_0215";
    case kStartupBlockerSpeakerAndNameChannel:
      return "speaker_id_channel_name_2214_2215_4214_4215";
    default:
      return "none";
  }
}

bool R70GatewayV2::isSemanticMirrorRequest(const NormalizedRequest& request) {
  if (!request.valid ||
      !request.tcpWrappedCan ||
      request.canId.device == 0 ||
      !request.admin.valid ||
      !isReadFunction(request.admin.functionCode) ||
      request.admin.telegramType != TelegramType::Unstructured ||
      request.objectAddress.hasSegment ||
      request.objectAddress.hasRecord) {
    return false;
  }

  // After bootstrap every decoded, non-segmented read may use its own cached
  // payload with the amp-wide CAN freshness.  The value is still refreshed
  // independently in the background; only the R1-facing reply age is shared.
  if (request.mapping != 0 && request.fieldId != FieldId::Unknown) {
    return true;
  }

  switch (normalizedObjectKey(request)) {
    case 0x0100:
    case 0x010D:
    case 0x0111:
    case 0x0205:
    case 0x0206:
    case 0x0300:
    case 0x0400:
    case 0x041D:
    case 0x041E:
    case 0x0500:
    case 0x0600:
    case 0x2205:
    case 0x2206:
    case 0x2400:
    case 0x2600:
    case 0x4205:
    case 0x4206:
    case 0x4400:
    case 0x4600:
    case 0x241D:
    case 0x241E:
    case 0x441D:
    case 0x441E:
      return true;
    default:
      return false;
  }
}

R70GatewayV2::SemanticMirrorGroup R70GatewayV2::classifySemanticMirrorGroup(
    const NormalizedRequest& request) {
  switch (normalizedObjectKey(request)) {
    case 0x0100:
    case 0x010D:
    case 0x0111:
      return SemanticMirrorGroup::Object0100_010D_0111;
    case 0x0205:
    case 0x0206:
    case 0x2205:
    case 0x2206:
      return SemanticMirrorGroup::Object0205_0206;
    case 0x0300:
      return SemanticMirrorGroup::Object0300;
    case 0x0400:
    case 0x041D:
    case 0x041E:
      return SemanticMirrorGroup::Object0400_041D_041E;
    case 0x0500:
    case 0x0600:
      return SemanticMirrorGroup::Object0500_0600;
    case 0x2400:
    case 0x2600:
      return SemanticMirrorGroup::Object2400_2600;
    case 0x4205:
    case 0x4206:
      return SemanticMirrorGroup::Object4205_4206;
    case 0x4400:
    case 0x4600:
      return SemanticMirrorGroup::Object4400_4600;
    case 0x241D:
    case 0x241E:
    case 0x441D:
    case 0x441E:
      return SemanticMirrorGroup::Object241D_241E_441D_441E;
    default:
      return SemanticMirrorGroup::None;
  }
}

const char* R70GatewayV2::semanticMirrorGroupName(SemanticMirrorGroup group) {
  switch (group) {
    case SemanticMirrorGroup::Object0100_010D_0111:
      return "0100_010D_0111";
    case SemanticMirrorGroup::Object0205_0206:
      return "0205_0206";
    case SemanticMirrorGroup::Object0300:
      return "0300";
    case SemanticMirrorGroup::Object0400_041D_041E:
      return "0400_041D_041E";
    case SemanticMirrorGroup::Object0500_0600:
      return "0500_0600";
    case SemanticMirrorGroup::Object2400_2600:
      return "2400_2600";
    case SemanticMirrorGroup::Object4205_4206:
      return "4205_4206";
    case SemanticMirrorGroup::Object4400_4600:
      return "4400_4600";
    case SemanticMirrorGroup::Object241D_241E_441D_441E:
      return "241D_241E_441D_441E";
    case SemanticMirrorGroup::None:
    default:
      return "none";
  }
}

bool R70GatewayV2::isProtectedSemanticMirrorGroup(SemanticMirrorGroup group) {
  switch (group) {
    case SemanticMirrorGroup::Object0100_010D_0111:
    case SemanticMirrorGroup::Object0205_0206:
    case SemanticMirrorGroup::Object0300:
    case SemanticMirrorGroup::Object2400_2600:
    case SemanticMirrorGroup::Object4205_4206:
    case SemanticMirrorGroup::Object4400_4600:
    case SemanticMirrorGroup::Object241D_241E_441D_441E:
      return true;
    case SemanticMirrorGroup::Object0400_041D_041E:
    case SemanticMirrorGroup::Object0500_0600:
    case SemanticMirrorGroup::None:
    default:
      return false;
  }
}

uint32_t R70GatewayV2::semanticMirrorHardMaxAgeMs() {
  return 1500u;
}

uint32_t R70GatewayV2::semanticMirrorGroupTargetAgeMs(SemanticMirrorGroup group) {
  switch (group) {
    case SemanticMirrorGroup::Object0100_010D_0111:
    case SemanticMirrorGroup::Object0300:
      return 500u;
    case SemanticMirrorGroup::Object0205_0206:
    case SemanticMirrorGroup::Object4205_4206:
      return 650u;
    case SemanticMirrorGroup::Object2400_2600:
    case SemanticMirrorGroup::Object4400_4600:
    case SemanticMirrorGroup::Object241D_241E_441D_441E:
      return 700u;
    case SemanticMirrorGroup::Object0400_041D_041E:
    case SemanticMirrorGroup::Object0500_0600:
      return 1000u;
    case SemanticMirrorGroup::None:
    default:
      return semanticMirrorHardMaxAgeMs();
  }
}

uint32_t R70GatewayV2::semanticMirrorGroupHardMaxAgeMs(SemanticMirrorGroup group) {
  switch (group) {
    case SemanticMirrorGroup::Object0100_010D_0111:
    case SemanticMirrorGroup::Object0300:
      return 900u;
    case SemanticMirrorGroup::Object0205_0206:
    case SemanticMirrorGroup::Object4205_4206:
      return 900u;
    case SemanticMirrorGroup::Object2400_2600:
    case SemanticMirrorGroup::Object4400_4600:
    case SemanticMirrorGroup::Object241D_241E_441D_441E:
      return 900u;
    case SemanticMirrorGroup::Object0400_041D_041E:
    case SemanticMirrorGroup::Object0500_0600:
      return 1500u;
    case SemanticMirrorGroup::None:
    default:
      return semanticMirrorHardMaxAgeMs();
  }
}

uint32_t R70GatewayV2::semanticMirrorFreshTtlMs(const NormalizedRequest& request) const {
  const uint32_t ttlMs = confirmedReadCacheTtlMs(config_, request);
  if (ttlMs != 0) {
    return ttlMs;
  }
  return postBootstrapStatusCacheTtlMs(request);
}

uint8_t R70GatewayV2::activeShapingCadenceGroupIndex(ShapingSimGroup group) {
  switch (group) {
    case ShapingSimGroup::Object241D_241E_441D_441E:
      return 0u;
    case ShapingSimGroup::Object0500_2600_4600:
      return 1u;
    case ShapingSimGroup::None:
    default:
      return 0xFFu;
  }
}

const char* R70GatewayV2::shapingDisabledReasonName(uint8_t code) {
  switch (code) {
    case 1:
      return "config_off";
    case 2:
      return "bootstrap_incomplete";
    case 3:
      return "allowed_immediate";
    case 4:
      return "deferred";
    case 5:
      return "bypass_age";
    case 6:
      return "classifier_none";
    default:
      return "none";
  }
}

uint16_t R70GatewayV2::estimateCanFrameBits(uint8_t dlc) {
  const uint16_t payloadBits = static_cast<uint16_t>(8u * dlc);
  const uint16_t baseBits = static_cast<uint16_t>(80u + payloadBits);
  return static_cast<uint16_t>(baseBits + 16u);
}

void R70GatewayV2::resetSlowCanWindow(uint32_t nowMs) const {
  slowCanWindow_ = {};
  slowCanWindow_.initialized = true;
  slowCanWindow_.startedMs = nowMs;
  for (uint8_t index = 0; index < kExpensiveObjectGroupCount; ++index) {
    slowCanWindow_.expensiveGroups[index].group =
        static_cast<ExpensiveObjectGroup>(static_cast<uint8_t>(index) + 1u);
  }
  for (uint8_t index = 0; index < kShapingSimGroupCount; ++index) {
    slowCanWindow_.shapingGroups[index].group =
        static_cast<ShapingSimGroup>(static_cast<uint8_t>(index) + 1u);
  }
}

R70GatewayV2::SlowObjectBucket* R70GatewayV2::findOrCreateSlowObjectBucket(uint16_t objectKey) {
  SlowObjectBucket* empty = 0;
  for (uint8_t index = 0; index < kSlowObjectBucketCapacity; ++index) {
    SlowObjectBucket& bucket = slowCanWindow_.objectBuckets[index];
    if (bucket.used && bucket.objectKey == objectKey) {
      return &bucket;
    }
    if (!bucket.used && empty == 0) {
      empty = &bucket;
    }
  }
  if (empty == 0) {
    return 0;
  }
  empty->used = true;
  empty->objectKey = objectKey;
  return empty;
}

R70GatewayV2::SlowNodeBucket* R70GatewayV2::findOrCreateSlowNodeBucket(uint8_t subnet, uint8_t device) {
  SlowNodeBucket* empty = 0;
  for (uint8_t index = 0; index < kSlowNodeBucketCapacity; ++index) {
    SlowNodeBucket& bucket = slowCanWindow_.nodeBuckets[index];
    if (bucket.used && bucket.subnet == subnet && bucket.device == device) {
      return &bucket;
    }
    if (!bucket.used && empty == 0) {
      empty = &bucket;
    }
  }
  if (empty == 0) {
    return 0;
  }
  empty->used = true;
  empty->subnet = subnet;
  empty->device = device;
  return empty;
}

R70GatewayV2::SlowObjectFairnessState* R70GatewayV2::findOrCreateObjectFairnessState(uint16_t objectKey) {
  SlowObjectFairnessState* empty = 0;
  for (uint8_t index = 0; index < kSlowObjectBucketCapacity; ++index) {
    SlowObjectFairnessState& state = objectFairnessStates_[index];
    if (state.used && state.objectKey == objectKey) {
      return &state;
    }
    if (!state.used && empty == 0) {
      empty = &state;
    }
  }
  if (empty == 0) {
    return 0;
  }
  empty->used = true;
  empty->objectKey = objectKey;
  return empty;
}

R70GatewayV2::ExpensiveObjectWindow* R70GatewayV2::expensiveObjectWindow(ExpensiveObjectGroup group) {
  if (group == ExpensiveObjectGroup::None) {
    return 0;
  }
  const uint8_t index = static_cast<uint8_t>(group) - 1u;
  if (index >= kExpensiveObjectGroupCount) {
    return 0;
  }
  ExpensiveObjectWindow& bucket = slowCanWindow_.expensiveGroups[index];
  bucket.group = group;
  return &bucket;
}

const R70GatewayV2::ExpensiveObjectWindow* R70GatewayV2::expensiveObjectWindow(ExpensiveObjectGroup group) const {
  if (group == ExpensiveObjectGroup::None) {
    return 0;
  }
  const uint8_t index = static_cast<uint8_t>(group) - 1u;
  if (index >= kExpensiveObjectGroupCount) {
    return 0;
  }
  return &slowCanWindow_.expensiveGroups[index];
}

R70GatewayV2::ShapingSimGroupWindow* R70GatewayV2::shapingSimGroupWindow(ShapingSimGroup group) {
  if (group == ShapingSimGroup::None) {
    return 0;
  }
  const uint8_t index = static_cast<uint8_t>(group) - 1u;
  if (index >= kShapingSimGroupCount) {
    return 0;
  }
  ShapingSimGroupWindow& bucket = slowCanWindow_.shapingGroups[index];
  bucket.group = group;
  return &bucket;
}

const R70GatewayV2::ShapingSimGroupWindow* R70GatewayV2::shapingSimGroupWindow(ShapingSimGroup group) const {
  if (group == ShapingSimGroup::None) {
    return 0;
  }
  const uint8_t index = static_cast<uint8_t>(group) - 1u;
  if (index >= kShapingSimGroupCount) {
    return 0;
  }
  return &slowCanWindow_.shapingGroups[index];
}

uint8_t R70GatewayV2::countPendingExactDuplicates(const NormalizedRequest& request) const {
  if (!config_.enablePerNodeRecentRequestRing || classifyExpensiveObjectGroup(request) == ExpensiveObjectGroup::None) {
    return 0;
  }
  const uint16_t objectKey = normalizedObjectKey(request);
  uint8_t duplicateCount = 0;
  for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
    const uint16_t index =
        static_cast<uint16_t>((recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) %
                              kRecentRequestTraceCapacity);
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used ||
        trace.subnet != request.canId.subnet ||
        trace.device != request.canId.device ||
        trace.objectKey != objectKey ||
        trace.fieldId != request.fieldId ||
        trace.channel != request.objectAddress.channelNumber ||
        trace.length != request.frame.length ||
        (trace.flags & kTraceFlagTcpTxSent) != 0) {
      continue;
    }
    bool exactMatch = true;
    for (uint8_t byteIndex = 0; byteIndex < trace.length && byteIndex < sizeof(trace.data); ++byteIndex) {
      if (trace.data[byteIndex] != request.frame.data[byteIndex]) {
        exactMatch = false;
        break;
      }
    }
    if (!exactMatch) {
      continue;
    }
    if (duplicateCount != 0xFFu) {
      ++duplicateCount;
    }
  }
  return duplicateCount;
}

void R70GatewayV2::noteExpensiveObjectRequest(const NormalizedRequest& request,
                                              uint32_t nowMs,
                                              uint8_t duplicatePendingCount,
                                              bool burstRepeat,
                                              bool segmentedActiveRepeat) {
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  ExpensiveObjectWindow* bucket = expensiveObjectWindow(classifyExpensiveObjectGroup(request));
  if (bucket == 0) {
    return;
  }

  const uint32_t elapsedMs = nowMs >= slowCanWindow_.startedMs ? nowMs - slowCanWindow_.startedMs : 0;
  uint8_t bucket100ms = static_cast<uint8_t>(elapsedMs / 100u);
  if (bucket100ms >= 100u) {
    bucket100ms = 99u;
  }
  uint8_t bucket1s = static_cast<uint8_t>(elapsedMs / 1000u);
  if (bucket1s >= 10u) {
    bucket1s = 9u;
  }

  ++bucket->tcpReadCount;
  ++bucket->requestPer100msBins[bucket100ms];
  ++bucket->requestPer1sBins[bucket1s];
  if (bucket->requestPer100msBins[bucket100ms] > bucket->peakRequestPer100ms) {
    bucket->peakRequestPer100ms = bucket->requestPer100msBins[bucket100ms];
  }
  if (bucket->requestPer1sBins[bucket1s] > bucket->peakRequestPer1s) {
    bucket->peakRequestPer1s = bucket->requestPer1sBins[bucket1s];
  }
  if (duplicatePendingCount != 0) {
    ++slowCanWindow_.duplicateInflightReadCount;
    ++bucket->duplicateInflightCount;
    if (duplicatePendingCount > bucket->duplicatePendingMax) {
      bucket->duplicatePendingMax = duplicatePendingCount;
    }
  }
  if (burstRepeat) {
    ++slowCanWindow_.burstRepeatReadCount;
    ++bucket->burstRepeatCount;
  }
  if (segmentedActiveRepeat) {
    ++slowCanWindow_.segmentedActiveRepeatCount;
    ++bucket->segmentedActiveRepeatCount;
  }
}

void R70GatewayV2::noteExpensiveObjectCanTx(const NormalizedRequest& request) {
  ExpensiveObjectWindow* bucket = expensiveObjectWindow(classifyExpensiveObjectGroup(request));
  if (bucket != 0) {
    ++bucket->canTxCount;
  }
}

void R70GatewayV2::noteExpensiveObjectCanRx(const RequestTrace& trace) {
  ExpensiveObjectWindow* bucket = expensiveObjectWindow(trace.expensiveGroup);
  if (bucket == 0) {
    return;
  }
  ++bucket->canRxCount;
  if (trace.canRoundtripMs > bucket->maxCanRoundtripMs) {
    bucket->maxCanRoundtripMs = trace.canRoundtripMs;
  }
}

void R70GatewayV2::noteExpensiveObjectCoalesced(const NormalizedRequest& request) {
  ExpensiveObjectWindow* bucket = expensiveObjectWindow(classifyExpensiveObjectGroup(request));
  if (bucket != 0) {
    ++bucket->coalescedCount;
  }
}

void R70GatewayV2::noteShapingSimCanTx(const NormalizedRequest& request, uint32_t nowMs) {
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  const ShapingSimGroup group = classifyShapingSimGroup(request);
  ShapingSimGroupWindow* bucket = shapingSimGroupWindow(group);
  if (bucket == 0) {
    return;
  }

  ++bucket->actualCanTxCount;
  const uint16_t nodeKey =
      static_cast<uint16_t>(((static_cast<uint16_t>(request.canId.subnet) & 0x07u) << 6) |
                            (static_cast<uint16_t>(request.canId.device) & 0x3Fu));
  if (nodeKey >= GatewayState::kMaxAddressSlots) {
    ++bucket->simulatedCanTxCount;
    return;
  }

  const uint16_t minIntervalMs = shapingSimIntervalMs(group);
  const uint32_t lastSimulatedTxMs = bucket->lastSimulatedTxMsByNode[nodeKey];
  if (lastSimulatedTxMs == 0 ||
      nowMs <= lastSimulatedTxMs ||
      static_cast<int32_t>(nowMs - (lastSimulatedTxMs + minIntervalMs)) >= 0) {
    bucket->lastSimulatedTxMsByNode[nodeKey] = nowMs;
    ++bucket->simulatedCanTxCount;
    return;
  }

  ++bucket->simulatedSavedTx;
  const uint16_t extraWaitMs = static_cast<uint16_t>((lastSimulatedTxMs + minIntervalMs) - nowMs);
  if (extraWaitMs > bucket->maxExtraWaitMs) {
    bucket->maxExtraWaitMs = extraWaitMs;
  }
  if (bucket->affectedNodeFlags[nodeKey] == 0) {
    bucket->affectedNodeFlags[nodeKey] = 1;
    ++bucket->affectedNodeCount;
  }
}

bool R70GatewayV2::isGentleStatusShapingGroup(ShapingSimGroup group) {
  return group == ShapingSimGroup::Object241D_241E_441D_441E ||
         group == ShapingSimGroup::Object0500_2600_4600;
}

void R70GatewayV2::noteActualShapingSavedCanTx(ShapingSimGroup group, bool softCollapse) {
  ShapingSimGroupWindow* bucket = shapingSimGroupWindow(group);
  if (bucket == 0) {
    return;
  }
  if (softCollapse) {
    ++bucket->softCollapseSavedTx;
  } else {
    ++bucket->shapedSavedTx;
  }
}

uint32_t R70GatewayV2::r1ReplyAgeMs(const AmpState& ampState, uint32_t nowMs) const {
  if (ampState.lastTcpTxToR1Ms != 0 && nowMs >= ampState.lastTcpTxToR1Ms) {
    return nowMs - ampState.lastTcpTxToR1Ms;
  }
  if (ampState.lastSeenMs != 0 && nowMs >= ampState.lastSeenMs) {
    return nowMs - ampState.lastSeenMs;
  }
  if (firstAmpDiscoveryMs_ != 0 && nowMs >= firstAmpDiscoveryMs_) {
    return nowMs - firstAmpDiscoveryMs_;
  }
  return 0;
}

uint32_t R70GatewayV2::maxR1ReplyAgeMs(uint32_t nowMs, uint8_t* devicesOver5s) const {
  uint32_t maxAgeMs = 0;
  uint8_t over5s = 0;
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    const uint32_t ageMs = r1ReplyAgeMs(*ampState, nowMs);
    if (ageMs > maxAgeMs) {
      maxAgeMs = ageMs;
    }
    if (ageMs > kR1FreshnessCriticalMs) {
      ++over5s;
    }
  }
  if (devicesOver5s != 0) {
    *devicesOver5s = over5s;
  }
  return maxAgeMs;
}

uint16_t R70GatewayV2::pendingTcpReplyCount() const {
  uint16_t count = 0;
  for (uint16_t index = 0; index < kRecentRequestTraceCapacity; ++index) {
    const RequestTrace& trace = recentRequestTraces_[index];
    if (trace.used &&
        (trace.flags & kTraceFlagTcpTxSent) == 0 &&
        trace.pendingState != TracePendingState::TimeoutCleanup) {
      ++count;
    }
  }
  return count;
}

bool R70GatewayV2::startupFreshnessProtectionActive(uint32_t nowMs) const {
  if (!tcpServer_.connected() || state_.activeAmpCount() == 0 || firstAmpDiscoveryMs_ == 0) {
    return false;
  }
  const uint32_t startMs =
      r1ConnectedSinceMs_ != 0 && r1ConnectedSinceMs_ > firstAmpDiscoveryMs_
          ? r1ConnectedSinceMs_
          : firstAmpDiscoveryMs_;
  return nowMs >= startMs && static_cast<int32_t>(nowMs - (startMs + kStartupFreshnessProtectionMs)) < 0;
}

bool R70GatewayV2::isStartupFreshnessPriorityRequest(const NormalizedRequest& request, uint32_t nowMs) const {
  if (!startupFreshnessProtectionActive(nowMs) ||
      !request.valid ||
      !request.tcpWrappedCan ||
      request.canId.device == 0 ||
      !isPostBootstrapStatusCacheRequest(request)) {
    return false;
  }
  const AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device);
  if (ampState == 0 || !ampState->used) {
    return false;
  }
  return r1ReplyAgeMs(*ampState, nowMs) >= kStartupFreshnessPriorityMs;
}

R70GatewayV2::GentleStatusShapingEntry* R70GatewayV2::findGentleStatusShapingEntry(const NormalizedRequest& request) {
  for (uint8_t index = 0; index < kGentleStatusShapingCapacity; ++index) {
    GentleStatusShapingEntry& entry = gentleStatusShaping_[index];
    if (!entry.used ||
        entry.subnet != request.canId.subnet ||
        entry.device != request.canId.device ||
        entry.channel != request.objectAddress.channelNumber ||
        entry.objectKey != normalizedObjectKey(request) ||
        entry.fieldId != request.fieldId ||
        entry.telegramType != request.admin.telegramType ||
        entry.hasRecord != request.objectAddress.hasRecord ||
        entry.hasSegment != request.objectAddress.hasSegment ||
        entry.length != request.frame.length) {
      continue;
    }
    if (entry.hasRecord && entry.recordNumber != request.objectAddress.recordNumber) {
      continue;
    }
    if (entry.hasSegment && entry.segmentNumber != request.objectAddress.segmentNumber) {
      continue;
    }
    bool exactBytes = true;
    for (uint8_t byteIndex = 0; byteIndex < entry.length && byteIndex < sizeof(entry.data); ++byteIndex) {
      if (entry.data[byteIndex] != request.frame.data[byteIndex]) {
        exactBytes = false;
        break;
      }
    }
    if (exactBytes) {
      return &entry;
    }
  }
  return 0;
}

const R70GatewayV2::GentleStatusShapingEntry* R70GatewayV2::findGentleStatusShapingEntry(
    const NormalizedRequest& request) const {
  return const_cast<R70GatewayV2*>(this)->findGentleStatusShapingEntry(request);
}

R70GatewayV2::GentleStatusShapingEntry* R70GatewayV2::reserveGentleStatusShapingEntry(const NormalizedRequest& request) {
  GentleStatusShapingEntry* reusable = 0;
  for (uint8_t index = 0; index < kGentleStatusShapingCapacity; ++index) {
    GentleStatusShapingEntry& entry = gentleStatusShaping_[index];
    if (!entry.used) {
      reusable = &entry;
      break;
    }
    if (!entry.pendingActive && reusable == 0) {
      reusable = &entry;
    }
  }
  if (reusable == 0) {
    reusable = &gentleStatusShaping_[0];
  }
  *reusable = {};
  reusable->used = true;
  reusable->subnet = request.canId.subnet;
  reusable->device = request.canId.device;
  reusable->channel = request.objectAddress.channelNumber;
  reusable->objectKey = normalizedObjectKey(request);
  reusable->fieldId = request.fieldId;
  reusable->telegramType = request.admin.telegramType;
  reusable->hasRecord = request.objectAddress.hasRecord;
  reusable->recordNumber = request.objectAddress.recordNumber;
  reusable->hasSegment = request.objectAddress.hasSegment;
  reusable->segmentNumber = request.objectAddress.segmentNumber;
  reusable->length = request.frame.length;
  for (uint8_t index = 0; index < request.frame.length && index < sizeof(reusable->data); ++index) {
    reusable->data[index] = request.frame.data[index];
  }
  reusable->group = classifyShapingSimGroup(request);
  return reusable;
}

void R70GatewayV2::clearGentleStatusShapingEntries() {
  for (uint8_t index = 0; index < kGentleStatusShapingCapacity; ++index) {
    gentleStatusShaping_[index] = {};
  }
  resetActiveShapingCadence();
}

bool R70GatewayV2::isPostBootstrapStatusCacheField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::status_pwr_ok:
    case FieldId::error_general:
    case FieldId::chstatus_isp_a:
    case FieldId::chstatus_isp_b:
    case FieldId::cherr_channel_err_a:
    case FieldId::cherr_channel_err_b:
    case FieldId::chstatus_remhold_gr_a:
    case FieldId::chstatus_remhold_gr_b:
    case FieldId::chstatus_remhold_ovl_a:
    case FieldId::chstatus_remhold_ovl_b:
      return true;
    default:
      return false;
  }
}

bool R70GatewayV2::isPostBootstrapStatusCacheRequest(const NormalizedRequest& request) {
  return request.valid &&
         request.canId.device != 0 &&
         request.admin.valid &&
         isReadFunction(request.admin.functionCode) &&
         request.admin.telegramType == TelegramType::Unstructured &&
         !request.objectAddress.hasRecord &&
         !request.objectAddress.hasSegment &&
         isSemanticMirrorRequest(request) &&
         (isPostBootstrapStatusCacheObjectKey(normalizedObjectKey(request)) ||
          classifySemanticMirrorGroup(request) != SemanticMirrorGroup::None);
}

R70GatewayV2::StatefulStatusCacheGroup R70GatewayV2::classifyStatefulStatusCacheGroup(
    const NormalizedRequest& request) {
  switch (normalizedObjectKey(request)) {
    case 0x241D:
    case 0x241E:
    case 0x441D:
    case 0x441E:
      return StatefulStatusCacheGroup::Object241D_241E_441D_441E;
    case 0x0500:
    case 0x2600:
    case 0x4600:
      return StatefulStatusCacheGroup::Object0500_2600_4600;
    case 0x2400:
    case 0x4400:
      return StatefulStatusCacheGroup::Object2400_4400;
    case 0x0300:
      return StatefulStatusCacheGroup::Object0300;
    default:
      return StatefulStatusCacheGroup::None;
  }
}

const char* R70GatewayV2::statefulStatusCacheGroupName(StatefulStatusCacheGroup group) {
  switch (group) {
    case StatefulStatusCacheGroup::Object241D_241E_441D_441E:
      return "241D_241E_441D_441E";
    case StatefulStatusCacheGroup::Object0500_2600_4600:
      return "0500_2600_4600";
    case StatefulStatusCacheGroup::Object2400_4400:
      return "2400_4400";
    case StatefulStatusCacheGroup::Object0300:
      return "0300";
    case StatefulStatusCacheGroup::None:
    default:
      return "none";
  }
}

bool R70GatewayV2::semanticMirrorActive(uint32_t nowMs) const {
  return statefulStatusCacheDisabledReason(true,
                                           tcpServer_.connected(),
                                           state_.activeAmpCount(),
                                           firstAmpDiscoveryMs_,
                                           lastAmpDiscoveryMs_,
                                           lastActiveAmpCountChangeMs_,
                                           bootstrapCompleteStableSinceMs_,
                                           counters_.bootstrap_nodes_incomplete,
                                           nowMs) == kStatefulStatusCacheActive;
}

R70GatewayV2::SemanticMirrorEntry* R70GatewayV2::findSemanticMirrorEntry(const NormalizedRequest& request) {
  if (!request.valid) {
    return 0;
  }
  const uint16_t objectKey = normalizedObjectKey(request);
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    SemanticMirrorEntry& entry = semanticMirror_[index];
    if (!entry.used ||
        entry.subnet != request.canId.subnet ||
        entry.device != request.canId.device ||
        entry.channel != request.objectAddress.channelNumber ||
        entry.objectKey != objectKey ||
        entry.fieldId != request.fieldId ||
        entry.telegramType != request.admin.telegramType) {
      continue;
    }
    return &entry;
  }
  return 0;
}

const R70GatewayV2::SemanticMirrorEntry* R70GatewayV2::findSemanticMirrorEntry(
    const NormalizedRequest& request) const {
  return const_cast<R70GatewayV2*>(this)->findSemanticMirrorEntry(request);
}

R70GatewayV2::SemanticMirrorEntry* R70GatewayV2::reserveSemanticMirrorEntry(const NormalizedRequest& request) {
  SemanticMirrorEntry* reusable = 0;
  bool preserveRequestTemplate = false;
  uint8_t preservedRequestLength = 0;
  uint8_t preservedRequestData[8] = {};
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    SemanticMirrorEntry& entry = semanticMirror_[index];
    if (entry.used &&
        entry.subnet == request.canId.subnet &&
        entry.device == request.canId.device &&
        entry.channel == request.objectAddress.channelNumber &&
        entry.objectKey == normalizedObjectKey(request) &&
        entry.fieldId == request.fieldId &&
        entry.telegramType == request.admin.telegramType) {
      reusable = &entry;
      preserveRequestTemplate = entry.hasRequestTemplate;
      preservedRequestLength = entry.requestLength;
      for (uint8_t dataIndex = 0; dataIndex < sizeof(preservedRequestData); ++dataIndex) {
        preservedRequestData[dataIndex] = entry.requestData[dataIndex];
      }
      break;
    }
    if (!entry.used && reusable == 0) {
      reusable = &entry;
    }
  }
  if (reusable == 0) {
    uint16_t oldestIndex = 0;
    uint32_t oldestMs = 0xFFFFFFFFu;
    for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
      const SemanticMirrorEntry& entry = semanticMirror_[index];
      const uint32_t candidateMs = entry.lastUpdateMs != 0 ? entry.lastUpdateMs : entry.lastServedMs;
      if (candidateMs <= oldestMs) {
        oldestMs = candidateMs;
        oldestIndex = index;
      }
    }
    reusable = &semanticMirror_[oldestIndex];
  }
  *reusable = {};
  reusable->used = true;
  reusable->subnet = request.canId.subnet;
  reusable->device = request.canId.device;
  reusable->channel = request.objectAddress.channelNumber;
  reusable->objectKey = normalizedObjectKey(request);
  reusable->fieldId = request.fieldId;
  reusable->telegramType = request.admin.telegramType;
  reusable->hasRequestTemplate = preserveRequestTemplate;
  reusable->requestLength = preservedRequestLength;
  for (uint8_t index = 0; index < sizeof(reusable->requestData); ++index) {
    reusable->requestData[index] = preservedRequestData[index];
  }
  return reusable;
}

void R70GatewayV2::clearSemanticMirror() {
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    semanticMirror_[index] = {};
  }
  resetSemanticMirrorSummaryWindow();
  resetSemanticMirrorBackgroundRefreshControl();
}

void R70GatewayV2::resetSemanticMirrorSummaryWindow() {
  semanticMirrorWindow_ = {};
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    semanticMirror_[index].staleServedCountWindow = 0;
  }
  semanticMirrorGroups_[0] = {SemanticMirrorGroup::Object0100_010D_0111, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[1] = {SemanticMirrorGroup::Object0205_0206, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[2] = {SemanticMirrorGroup::Object0300, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[3] = {SemanticMirrorGroup::Object0400_041D_041E, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[4] = {SemanticMirrorGroup::Object0500_0600, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[5] = {SemanticMirrorGroup::Object2400_2600, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[6] = {SemanticMirrorGroup::Object4205_4206, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[7] = {SemanticMirrorGroup::Object4400_4600, 0, 0, 0, 0, 0, 0, 0, 0};
  semanticMirrorGroups_[8] = {SemanticMirrorGroup::Object241D_241E_441D_441E, 0, 0, 0, 0, 0, 0, 0, 0};
}

void R70GatewayV2::resetSemanticMirrorBackgroundRefreshControl() {
  semanticMirrorBgRefreshPauseUntilMs_ = 0;
  semanticMirrorBgRefreshWindow100MsStartedMs_ = 0;
  semanticMirrorBgRefreshWindow1sStartedMs_ = 0;
  semanticMirrorBgRefreshCount100Ms_ = 0;
  semanticMirrorBgRefreshCount1s_ = 0;
  semanticMirrorRefreshCursor_ = 0;
  semanticMirrorProtectedRefreshWindow100MsStartedMs_ = 0;
  semanticMirrorProtectedRefreshWindow1sStartedMs_ = 0;
  semanticMirrorProtectedRefreshCount100Ms_ = 0;
  semanticMirrorProtectedRefreshCount1s_ = 0;
  semanticMirrorProtectedRefreshCursor_ = 0;
  for (uint16_t index = 0; index < GatewayState::kMaxAddressSlots; ++index) {
    semanticMirrorBgRefreshLastNodeMs_[index] = 0;
    semanticMirrorProtectedRefreshLastNodeMs_[index] = 0;
  }
  for (uint8_t index = 0; index < DebugCounters::kSubnetCount; ++index) {
    semanticMirrorBgRefreshLastSubnetMs_[index] = 0;
    semanticMirrorProtectedRefreshLastSubnetMs_[index] = 0;
  }
}

bool R70GatewayV2::semanticMirrorBackgroundRefreshPaused(uint32_t nowMs) const {
  return semanticMirrorBgRefreshPauseUntilMs_ != 0 &&
         static_cast<int32_t>(nowMs - semanticMirrorBgRefreshPauseUntilMs_) < 0;
}

bool R70GatewayV2::semanticMirrorBackgroundRefreshAllowed(const NormalizedRequest& request,
                                                          uint32_t nowMs,
                                                          const char*& reason,
                                                          bool& rateLimited) {
  reason = "none";
  rateLimited = false;

  if (semanticMirrorBackgroundRefreshPaused(nowMs)) {
    reason = "diagnostic_hold";
    return false;
  }
  if (pendingTcpReplyCount() >= kSemanticMirrorBackgroundRefreshMaxPendingTcpReplies) {
    reason = "tcp_pressure";
    return false;
  }
  if (scheduler_.queueDepth() != 0 || counters_.can_tx_queue_depth > kSemanticMirrorBackgroundRefreshMaxCanTxQueueDepth) {
    reason = "can_pending";
    return false;
  }
  if (slowCanWindow_.worstDeviceReplyGapMs >= kSemanticMirrorBackgroundRefreshMaxServiceGapMs) {
    reason = "service_gap";
    return false;
  }

  if (semanticMirrorBgRefreshWindow100MsStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorBgRefreshWindow100MsStartedMs_) >= 100u) {
    semanticMirrorBgRefreshWindow100MsStartedMs_ = nowMs;
    semanticMirrorBgRefreshCount100Ms_ = 0;
  }
  if (semanticMirrorBgRefreshWindow1sStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorBgRefreshWindow1sStartedMs_) >= 1000u) {
    semanticMirrorBgRefreshWindow1sStartedMs_ = nowMs;
    semanticMirrorBgRefreshCount1s_ = 0;
  }

  const uint16_t nodeIndex = semanticMirrorNodeIndex(request.canId.subnet, request.canId.device);
  if (semanticMirrorBgRefreshCount100Ms_ >= kSemanticMirrorBackgroundRefreshMaxPer100Ms ||
      semanticMirrorBgRefreshCount1s_ >= kSemanticMirrorBackgroundRefreshMaxPer1s ||
      (nodeIndex < GatewayState::kMaxAddressSlots &&
       semanticMirrorBgRefreshLastNodeMs_[nodeIndex] != 0 &&
       static_cast<uint32_t>(nowMs - semanticMirrorBgRefreshLastNodeMs_[nodeIndex]) <
           kSemanticMirrorBackgroundRefreshNodeCooldownMs) ||
      (request.canId.subnet < DebugCounters::kSubnetCount &&
       semanticMirrorBgRefreshLastSubnetMs_[request.canId.subnet] != 0 &&
       static_cast<uint32_t>(nowMs - semanticMirrorBgRefreshLastSubnetMs_[request.canId.subnet]) <
           kSemanticMirrorBackgroundRefreshSubnetCooldownMs)) {
    reason = "rate_limit";
    rateLimited = true;
    return false;
  }

  return true;
}

void R70GatewayV2::noteSemanticMirrorBackgroundRefreshSent(const NormalizedRequest& request,
                                                           uint32_t nowMs) {
  if (semanticMirrorBgRefreshWindow100MsStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorBgRefreshWindow100MsStartedMs_) >= 100u) {
    semanticMirrorBgRefreshWindow100MsStartedMs_ = nowMs;
    semanticMirrorBgRefreshCount100Ms_ = 0;
  }
  if (semanticMirrorBgRefreshWindow1sStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorBgRefreshWindow1sStartedMs_) >= 1000u) {
    semanticMirrorBgRefreshWindow1sStartedMs_ = nowMs;
    semanticMirrorBgRefreshCount1s_ = 0;
  }

  if (semanticMirrorBgRefreshCount100Ms_ < 0xFFu) {
    ++semanticMirrorBgRefreshCount100Ms_;
  }
  if (semanticMirrorBgRefreshCount1s_ < 0xFFu) {
    ++semanticMirrorBgRefreshCount1s_;
  }
  const uint16_t nodeIndex = semanticMirrorNodeIndex(request.canId.subnet, request.canId.device);
  if (nodeIndex < GatewayState::kMaxAddressSlots) {
    semanticMirrorBgRefreshLastNodeMs_[nodeIndex] = nowMs;
  }
  if (request.canId.subnet < DebugCounters::kSubnetCount) {
    semanticMirrorBgRefreshLastSubnetMs_[request.canId.subnet] = nowMs;
  }
}

void R70GatewayV2::pauseSemanticMirrorBackgroundRefresh(uint32_t nowMs, const char* reason) {
  if (semanticMirrorBackgroundRefreshPaused(nowMs) &&
      static_cast<uint32_t>(semanticMirrorBgRefreshPauseUntilMs_ - nowMs) > 30000u) {
    return;
  }
  semanticMirrorBgRefreshPauseUntilMs_ = nowMs + kSemanticMirrorBackgroundRefreshPauseMs;
  char line[192] = {};
  snprintf(line,
           sizeof(line),
           "BG_REFRESH_PAUSED_AFTER_TIMEOUT reason=%s pause_ms=%lu worst_device_reply_gap_ms=%lu r1_timeout_events=%lu",
           reason != 0 ? reason : "timeout",
           static_cast<unsigned long>(kSemanticMirrorBackgroundRefreshPauseMs),
           static_cast<unsigned long>(slowCanWindow_.worstDeviceReplyGapMs),
           static_cast<unsigned long>(counters_.r1_timeout_events));
  debugCli_.logText(line);
}

bool R70GatewayV2::semanticMirrorProtectedRefreshAllowed(const NormalizedRequest& request,
                                                         uint32_t nowMs,
                                                         const char*& reason) {
  reason = "none";
  if (semanticMirrorProtectedRefreshWindow100MsStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorProtectedRefreshWindow100MsStartedMs_) >= 100u) {
    semanticMirrorProtectedRefreshWindow100MsStartedMs_ = nowMs;
    semanticMirrorProtectedRefreshCount100Ms_ = 0;
  }
  if (semanticMirrorProtectedRefreshWindow1sStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorProtectedRefreshWindow1sStartedMs_) >= 1000u) {
    semanticMirrorProtectedRefreshWindow1sStartedMs_ = nowMs;
    semanticMirrorProtectedRefreshCount1s_ = 0;
  }

  const uint16_t nodeIndex = semanticMirrorNodeIndex(request.canId.subnet, request.canId.device);
  if (semanticMirrorProtectedRefreshCount100Ms_ >= kSemanticMirrorProtectedRefreshMaxPer100Ms ||
      semanticMirrorProtectedRefreshCount1s_ >= kSemanticMirrorProtectedRefreshMaxPer1s) {
    reason = "rate_limit";
    return false;
  }
  if (nodeIndex < GatewayState::kMaxAddressSlots &&
      semanticMirrorProtectedRefreshLastNodeMs_[nodeIndex] != 0 &&
      static_cast<uint32_t>(nowMs - semanticMirrorProtectedRefreshLastNodeMs_[nodeIndex]) <
          kSemanticMirrorProtectedRefreshNodeCooldownMs) {
    reason = "node_cooldown";
    return false;
  }
  if (request.canId.subnet < DebugCounters::kSubnetCount &&
      semanticMirrorProtectedRefreshLastSubnetMs_[request.canId.subnet] != 0 &&
      static_cast<uint32_t>(nowMs - semanticMirrorProtectedRefreshLastSubnetMs_[request.canId.subnet]) <
          kSemanticMirrorProtectedRefreshSubnetCooldownMs) {
    reason = "subnet_cooldown";
    return false;
  }
  return true;
}

void R70GatewayV2::noteSemanticMirrorProtectedRefreshSent(const NormalizedRequest& request, uint32_t nowMs) {
  if (semanticMirrorProtectedRefreshWindow100MsStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorProtectedRefreshWindow100MsStartedMs_) >= 100u) {
    semanticMirrorProtectedRefreshWindow100MsStartedMs_ = nowMs;
    semanticMirrorProtectedRefreshCount100Ms_ = 0;
  }
  if (semanticMirrorProtectedRefreshWindow1sStartedMs_ == 0 ||
      static_cast<uint32_t>(nowMs - semanticMirrorProtectedRefreshWindow1sStartedMs_) >= 1000u) {
    semanticMirrorProtectedRefreshWindow1sStartedMs_ = nowMs;
    semanticMirrorProtectedRefreshCount1s_ = 0;
  }
  if (semanticMirrorProtectedRefreshCount100Ms_ < 0xFFu) {
    ++semanticMirrorProtectedRefreshCount100Ms_;
  }
  if (semanticMirrorProtectedRefreshCount1s_ < 0xFFu) {
    ++semanticMirrorProtectedRefreshCount1s_;
  }
  const uint16_t nodeIndex = semanticMirrorNodeIndex(request.canId.subnet, request.canId.device);
  if (nodeIndex < GatewayState::kMaxAddressSlots) {
    semanticMirrorProtectedRefreshLastNodeMs_[nodeIndex] = nowMs;
  }
  if (request.canId.subnet < DebugCounters::kSubnetCount) {
    semanticMirrorProtectedRefreshLastSubnetMs_[request.canId.subnet] = nowMs;
  }
}

void R70GatewayV2::rememberSemanticMirrorRequestTemplate(const NormalizedRequest& request) {
  if (!request.valid || !isSemanticMirrorRequest(request)) {
    return;
  }
  SemanticMirrorEntry* entry = findSemanticMirrorEntry(request);
  if (entry == 0) {
    entry = reserveSemanticMirrorEntry(request);
  }
  if (entry == 0) {
    return;
  }
  entry->hasRequestTemplate = true;
  entry->requestLength = request.frame.length;
  for (uint8_t index = 0; index < sizeof(entry->requestData); ++index) {
    entry->requestData[index] = index < request.frame.length ? request.frame.data[index] : 0u;
  }
}

bool R70GatewayV2::buildSemanticMirrorRefreshRequest(const SemanticMirrorEntry& entry, NormalizedRequest& request) const {
  if (!entry.used || !entry.learned || !entry.hasRequestTemplate || entry.requestLength == 0) {
    return false;
  }

  request = {};
  request.valid = true;
  request.tcpWrappedCan = true;
  request.canId.valid = true;
  request.canId.subnet = entry.subnet;
  request.canId.device = entry.device;
  request.admin.valid = true;
  request.admin.functionCode = FunctionCode::ReadObject;
  request.admin.telegramType = entry.telegramType;
  request.objectAddress.channelNumber = entry.channel;
  request.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
  request.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
  request.fieldId = entry.fieldId;
  request.frame.canId =
      protocol::makeCanId(protocol::kObjectClassPrimaryMaster, Address{entry.subnet, entry.device});
  request.frame.length = entry.requestLength;
  for (uint8_t index = 0; index < entry.requestLength && index < sizeof(request.frame.data); ++index) {
    request.frame.data[index] = entry.requestData[index];
  }
  return true;
}

void R70GatewayV2::processSemanticMirrorFreshness(uint32_t nowMs) {
  // Verify optimistic power/mute values without attaching an R1 waiter.  At
  // most one request is queued per pass; the common maintenance limits then
  // spread a group write over the CAN bus instead of creating a burst.
  for (uint16_t offset = 0; offset < kSemanticMirrorCapacity; ++offset) {
    const uint16_t index = static_cast<uint16_t>(
        (semanticMirrorRefreshCursor_ + offset) % kSemanticMirrorCapacity);
    SemanticMirrorEntry& entry = semanticMirror_[index];
    if (!entry.used || !entry.learned || !entry.invalidatedByWrite ||
        entry.lastUpdateMs == 0 || entry.lastUpdateMs > nowMs) {
      continue;
    }
    // A powered-off amplifier cannot confirm object 0100.  Polling it only
    // creates unanswered CAN reads and delays R1's offline transition.  Keep
    // the explicit OFF command as the authoritative R1-facing value.
    if (entry.fieldId == FieldId::settings_power &&
        entry.replyFrame.length >= 4 && entry.replyFrame.data[3] == 0u) {
      continue;
    }
    const uint32_t verifyDelayMs = entry.fieldId == FieldId::settings_power
                                       ? kPowerWriteThroughVerifyDelayMs
                                       : kMuteWriteThroughVerifyDelayMs;
    if (nowMs - entry.lastUpdateMs < verifyDelayMs) {
      continue;
    }
    if (entry.refreshInflight) {
      if (entry.refreshEnqueuedMs != 0 &&
          nowMs - entry.refreshEnqueuedMs >= kWriteThroughRefreshRetryMs) {
        entry.refreshInflight = false;
        entry.refreshEnqueuedMs = 0;
      } else {
        continue;
      }
    }

    NormalizedRequest refresh = {};
    if (!buildSemanticMirrorRefreshRequest(entry, refresh)) {
      continue;
    }
    const char* reason = 0;
    if (currentBusUtilAvgPercent(nowMs) >= 70u ||
        !semanticMirrorProtectedRefreshAllowed(refresh, nowMs, reason) ||
        !scheduler_.enqueueMaintenance(refresh, nowMs, counters_, reason)) {
      continue;
    }
    entry.refreshInflight = true;
    entry.refreshEnqueuedMs = nowMs;
    noteSemanticMirrorProtectedRefreshSent(refresh, nowMs);
    semanticMirrorRefreshCursor_ = static_cast<uint16_t>((index + 1u) % kSemanticMirrorCapacity);
    char line[176] = {};
    snprintf(line, sizeof(line),
             "SEMANTIC_WRITE_VERIFY_QUEUED node=%u.%02u object=%04X optimistic_age_ms=%lu",
             static_cast<unsigned>(entry.subnet), static_cast<unsigned>(entry.device),
             static_cast<unsigned>(entry.objectKey),
             static_cast<unsigned long>(nowMs - entry.lastUpdateMs));
    debugCli_.logText(line);
    return;
  }
}

void R70GatewayV2::invalidateSemanticMirror(uint8_t subnet, uint8_t device, uint8_t channel) {
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    SemanticMirrorEntry& entry = semanticMirror_[index];
    if (!entry.used || entry.subnet != subnet || entry.device != device) {
      continue;
    }
    if (channel != 0 && entry.channel != 0 && entry.channel != channel) {
      continue;
    }
    entry.learned = false;
    entry.refreshInflight = false;
    entry.invalidatedByWrite = false;
    entry.lastServedMs = 0;
    entry.refreshEnqueuedMs = 0;
  }
}

bool R70GatewayV2::updateSemanticMirrorOptimisticallyForWrite(
    const NormalizedRequest& request,
    uint32_t nowMs) {
  if (!request.valid ||
      request.canId.device == 0 ||
      request.frame.length < 4 ||
      !request.admin.valid ||
      !isWriteFunction(request.admin.functionCode) ||
      request.admin.telegramType != TelegramType::Unstructured) {
    return false;
  }
  if (request.fieldId != FieldId::settings_power &&
      request.fieldId != FieldId::config_mute_a &&
      request.fieldId != FieldId::config_mute_b) {
    return false;
  }

  const uint16_t objectKey = normalizedObjectKey(request);
  SemanticMirrorEntry* target = findSemanticMirrorEntry(request);
  if (target == 0) {
    target = reserveSemanticMirrorEntry(request);
  }
  if (target == 0) {
    return false;
  }
  SemanticMirrorEntry& entry = *target;
  const bool wasConfirmedPowerOff =
      request.fieldId == FieldId::settings_power &&
      entry.learned && entry.replyFrame.length >= 4 && entry.replyFrame.data[3] == 0u;
  if (!entry.learned || entry.replyFrame.length < 4) {
    // A group write can arrive before R1 has read this field.  For these
    // scalar booleans the write already contains everything needed to build
    // the corresponding read reply and its future verification request.
    entry.replyFrame = request.frame;
    entry.replyFrame.data[0] = static_cast<uint8_t>(
        (entry.replyFrame.data[0] & 0xF0u) | static_cast<uint8_t>(FunctionCode::ReadObject));
    entry.replyFrame.length = 4;
  }
    // Power and mute writes carry the complete boolean value in byte 3. Keep
    // the proven read-reply envelope and only replace that value.
    entry.replyFrame.data[3] = request.frame.data[3];
    entry.lastUpdateMs = nowMs;
    entry.lastServedMs = 0;
    entry.refreshInflight = false;
    // Reuse this flag as "optimistic, verification pending".  The read path
    // continues serving the value while a cache-only CAN read verifies it.
    entry.invalidatedByWrite = true;
    entry.refreshEnqueuedMs = 0;
    entry.lastRefreshBlockedReason =
        request.fieldId == FieldId::settings_power && request.frame.data[3] == 0u
            ? kSemanticPowerOffWriteAckConfirmed
            : kSemanticRefreshBlockedNone;
    entry.learned = true;
    entry.hasRequestTemplate = true;
    entry.requestLength = 3;
    entry.requestData[0] = static_cast<uint8_t>(
        (request.frame.data[0] & 0xF0u) | static_cast<uint8_t>(FunctionCode::ReadObject));
    entry.requestData[1] = request.frame.data[1];
    entry.requestData[2] = request.frame.data[2];
    for (uint8_t index = 3; index < sizeof(entry.requestData); ++index) {
      entry.requestData[index] = 0;
    }

  if (request.fieldId == FieldId::settings_power && request.frame.data[3] != 0u &&
      wasConfirmedPowerOff) {
    armPowerOnBootstrap(request.canId.subnet, request.canId.device, nowMs);
    // Power OK is a separate R1-visible object. Do not let its previous ON
    // value survive a new power-on cycle; only a genuine 0300 CAN reply may
    // make it valid again.
    state_.invalidateFieldCache(request.canId.subnet,
                                request.canId.device,
                                FieldId::status_pwr_ok);
    for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
      SemanticMirrorEntry& candidate = semanticMirror_[index];
      if (!candidate.used || candidate.subnet != request.canId.subnet ||
          candidate.device != request.canId.device ||
          candidate.fieldId != FieldId::status_pwr_ok) {
        continue;
      }
      // Keep reporting the known OFF value while the amplifier wakes, but do
      // not accept it as the final readback.  A low-rate maintenance read will
      // keep checking 0300 until the amplifier itself reports Power OK.
      candidate.learned = candidate.replyFrame.length >= 4;
      candidate.invalidatedByWrite = true;
      candidate.refreshInflight = false;
      candidate.refreshEnqueuedMs = 0;
      candidate.lastServedMs = 0;
      candidate.lastUpdateMs = nowMs;
      candidate.lastRefreshBlockedReason = kSemanticRefreshBlockedNone;
      candidate.lastRefreshBlockedMs = 0;
      break;
    }
  }

  uint16_t activeEntries = 0;
  bool powerOkMirroredOff = false;
  if (request.fieldId == FieldId::settings_power && request.frame.data[3] == 0u) {
    AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device, false);
    if (ampState != 0) {
      ampState->powerOnBootstrapPendingConfirmation = false;
      ampState->powerOnBootstrapUntilMs = 0;
    }
    // Power OFF is standby: non-power objects remain reachable on CAN. Keep
    // every learned non-power value active and let normal polling refresh it.
    // Only the power entry above uses protected write-through semantics.
    for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
      SemanticMirrorEntry& candidate = semanticMirror_[index];
      if (!candidate.used || &candidate == &entry ||
          candidate.subnet != request.canId.subnet ||
          candidate.device != request.canId.device) {
        continue;
      }
      if (candidate.fieldId == FieldId::status_pwr_ok && candidate.learned &&
          candidate.replyFrame.length >= 4) {
        candidate.replyFrame.data[3] = 0u;
        candidate.lastUpdateMs = nowMs;
        candidate.lastServedMs = 0;
        candidate.invalidatedByWrite = true;
        candidate.refreshInflight = false;
        candidate.refreshEnqueuedMs = 0;
        candidate.lastRefreshBlockedReason = kSemanticRefreshBlockedNone;
        candidate.lastRefreshBlockedMs = 0;
        powerOkMirroredOff = true;
      }
      ++activeEntries;
    }
  }

  char line[192] = {};
  snprintf(line,
           sizeof(line),
           "SEMANTIC_MIRROR_WRITE_THROUGH node=%u.%02u object=%04X value=%02X entries=%u verify_after_ms=%lu",
           static_cast<unsigned>(request.canId.subnet),
           static_cast<unsigned>(request.canId.device),
           static_cast<unsigned>(objectKey),
           static_cast<unsigned>(request.frame.data[3]),
           1u,
           static_cast<unsigned long>(request.fieldId == FieldId::settings_power
                                          ? kPowerWriteThroughVerifyDelayMs
                                          : kMuteWriteThroughVerifyDelayMs));
  debugCli_.logText(line);
  if (activeEntries != 0) {
    snprintf(line,
             sizeof(line),
             "POWER_OFF_NODE_CACHE_ACTIVE node=%u.%02u entries=%u standby_polling=1 power_ok_off=%u",
             static_cast<unsigned>(request.canId.subnet),
             static_cast<unsigned>(request.canId.device),
             static_cast<unsigned>(activeEntries),
             static_cast<unsigned>(powerOkMirroredOff ? 1u : 0u));
    debugCli_.logText(line);
  }
  return true;
}

void R70GatewayV2::confirmSemanticMirrorWriteAck(const NormalizedRequest& reply,
                                                  uint32_t nowMs) {
  if (!reply.valid || !reply.admin.valid || !isWriteFunction(reply.admin.functionCode) ||
      reply.fieldId != FieldId::settings_power) {
    return;
  }
  SemanticMirrorEntry* entry = findSemanticMirrorEntry(reply);
  if (entry == 0 || !entry->invalidatedByWrite || entry->replyFrame.length < 4 ||
      entry->replyFrame.data[3] != 0u ||
      entry->lastRefreshBlockedReason != kSemanticPowerOffAwaitingWriteAck) {
    return;
  }
  entry->lastRefreshBlockedReason = kSemanticPowerOffWriteAckConfirmed;
  entry->lastUpdateMs = nowMs;
  char line[160] = {};
  snprintf(line, sizeof(line),
           "SEMANTIC_POWER_OFF_ACK_CONFIRMED node=%u.%02u object=%04X",
           static_cast<unsigned>(reply.canId.subnet),
           static_cast<unsigned>(reply.canId.device),
           static_cast<unsigned>(normalizedObjectKey(reply)));
  debugCli_.logText(line);
}

void R70GatewayV2::confirmPowerOnFromNodeActivity(const NormalizedRequest& reply,
                                                   uint32_t nowMs) {
  if (!reply.valid || reply.canId.device == 0 || !reply.admin.valid ||
      !isReadFunction(reply.admin.functionCode)) {
    return;
  }
  const uint16_t sourceObject = normalizedObjectKey(reply);
  // Once the post-write guard has elapsed, every genuine node-specific CAN
  // read proves that this amplifier is responding again. Restricting release
  // to five presence objects left otherwise active nodes waiting until one of
  // those slower polling cycles happened to arrive.
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    SemanticMirrorEntry& entry = semanticMirror_[index];
    if (!entry.used || !entry.learned || !entry.invalidatedByWrite ||
        entry.subnet != reply.canId.subnet || entry.device != reply.canId.device ||
        entry.fieldId != FieldId::settings_power || entry.replyFrame.length < 4 ||
        entry.replyFrame.data[3] == 0u) {
      continue;
    }
    if (entry.lastUpdateMs > nowMs ||
        static_cast<uint32_t>(nowMs - entry.lastUpdateMs) < kPowerOnNodeActivityGuardMs) {
      continue;
    }
    entry.invalidatedByWrite = false;
    entry.refreshInflight = false;
    entry.refreshEnqueuedMs = 0;
    entry.lastRefreshBlockedReason = kSemanticRefreshBlockedNone;
    entry.lastUpdateMs = nowMs;
    bool targetedBootstrap = false;
    AmpState* ampState = state_.amp(reply.canId.subnet, reply.canId.device, false);
    if (ampState != 0 && ampState->powerOnBootstrapPendingConfirmation) {
      targetedBootstrap = true;
      ampState->powerOnBootstrapPendingConfirmation = false;
      uint8_t bootstrapIndex = 0;
      if (classifyBootstrapReply(reply, bootstrapIndex)) {
        ampState->bootstrapSeenMask |= bootstrapMaskFor(bootstrapIndex);
      }
    }
    char line[160] = {};
    snprintf(line, sizeof(line),
             "SEMANTIC_POWER_ON_NODE_CONFIRMED node=%u.%02u source_object=%04X targeted_bootstrap=%u",
             static_cast<unsigned>(reply.canId.subnet),
             static_cast<unsigned>(reply.canId.device),
             static_cast<unsigned>(sourceObject),
             static_cast<unsigned>(targetedBootstrap ? 1u : 0u));
    debugCli_.logText(line);
    return;
  }
}

void R70GatewayV2::invalidateSemanticMirrorForWrite(const NormalizedRequest& request, const char* reason) {
  if (!request.valid || request.canId.device == 0) {
    return;
  }
  const uint16_t objectKey = normalizedObjectKey(request);
  bool invalidated = false;
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    SemanticMirrorEntry& entry = semanticMirror_[index];
    if (!entry.used ||
        entry.subnet != request.canId.subnet ||
        entry.device != request.canId.device ||
        entry.channel != request.objectAddress.channelNumber ||
        entry.objectKey != objectKey) {
      continue;
    }
    entry.learned = false;
    entry.refreshInflight = false;
    entry.invalidatedByWrite = true;
    entry.refreshEnqueuedMs = 0;
    invalidated = true;
  }
  if (!invalidated) {
    return;
  }
  ++semanticMirrorWindow_.writeInvalidations;
  char line[160] = {};
  snprintf(line,
           sizeof(line),
           "SEMANTIC_MIRROR_INVALIDATE node=%u.%02u object=%04X reason=%s",
           static_cast<unsigned>(request.canId.subnet),
           static_cast<unsigned>(request.canId.device),
           static_cast<unsigned>(objectKey),
           reason != 0 ? reason : "write");
  debugCli_.logText(line);
}

bool R70GatewayV2::isSemanticMirrorBackgroundRefreshReply(const NormalizedRequest& reply) const {
  const SemanticMirrorEntry* entry = findSemanticMirrorEntry(reply);
  if (entry == 0 || !entry->refreshInflight || entry->refreshEnqueuedMs == 0) {
    return false;
  }
  return true;
}

uint8_t R70GatewayV2::statefulStatusCacheDisabledReason(bool featureEnabled,
                                                        bool tcpConnected,
                                                        uint8_t activeAmpCount,
                                                        uint32_t firstAmpDiscoveryMs,
                                                        uint32_t lastAmpDiscoveryMs,
                                                        uint32_t lastActiveAmpCountChangeMs,
                                                        uint32_t bootstrapCompleteStableSinceMs,
                                                        uint32_t bootstrapNodesIncomplete,
                                                        uint32_t nowMs) {
  (void)lastActiveAmpCountChangeMs;
  (void)bootstrapCompleteStableSinceMs;
  if (!featureEnabled) {
    return kStatefulStatusCacheFlagOff;
  }
  if (!tcpConnected) {
    return kStatefulStatusCacheTcpDisconnected;
  }
  if (activeAmpCount == 0) {
    return kStatefulStatusCacheNoActiveAmps;
  }
  if (firstAmpDiscoveryMs == 0 || nowMs < firstAmpDiscoveryMs) {
    return kStatefulStatusCacheNoDiscoveryYet;
  }
  if (static_cast<int32_t>(nowMs - (firstAmpDiscoveryMs + statefulStatusCacheWarmupMs())) < 0) {
    return kStatefulStatusCacheWarmup;
  }
  if (lastAmpDiscoveryMs == 0 || nowMs < lastAmpDiscoveryMs ||
      static_cast<int32_t>(nowMs - (lastAmpDiscoveryMs + ampCountStableRequiredMs())) < 0) {
    return kStatefulStatusCacheAmpCountUnstable;
  }
  if (bootstrapNodesIncomplete != 0) {
    return kStatefulStatusCacheBootstrapIncomplete;
  }
  return kStatefulStatusCacheActive;
}

const char* R70GatewayV2::statefulStatusCacheDisabledReasonName(uint8_t reasonCode) {
  switch (reasonCode) {
    case kStatefulStatusCacheActive:
      return "active";
    case kStatefulStatusCacheFlagOff:
      return "feature_flag_off";
    case kStatefulStatusCacheTcpDisconnected:
      return "r1_disconnected";
    case kStatefulStatusCacheNoActiveAmps:
      return "no_active_amps";
    case kStatefulStatusCacheNoDiscoveryYet:
      return "no_discovery";
    case kStatefulStatusCacheWarmup:
      return "warmup";
    case kStatefulStatusCacheAmpCountUnstable:
      return "amp_count_stability";
    case kStatefulStatusCacheBootstrapIncomplete:
      return "bootstrap_unsafe";
    case kStatefulStatusCacheBootstrapSteadyDelay:
      return "bootstrap_steady_delay";
    case kStatefulStatusCacheAutoDisabled:
      return "auto_disabled_timeout";
    case kStatefulStatusCacheAutoDisabledChannelIncrease:
      return "auto_disabled_channel_increase";
    case kStatefulStatusCacheAutoDisabledAge:
      return "auto_disabled_cache_age";
    case kStatefulStatusCacheAutoDisabledServiceGap:
      return "auto_disabled_service_gap";
    default:
      return "unknown";
  }
}

void R70GatewayV2::clearStatefulStatusCache() {
  for (uint16_t index = 0; index < kStatefulStatusCacheCapacity; ++index) {
    statefulStatusCache_[index] = {};
  }
}

void R70GatewayV2::resetStatefulStatusCacheGroupWindows() {
  statefulStatusCacheGroups_[0] = {StatefulStatusCacheGroup::Object241D_241E_441D_441E, 0, 0, 0, 0, 0, 0};
  statefulStatusCacheGroups_[1] = {StatefulStatusCacheGroup::Object0500_2600_4600, 0, 0, 0, 0, 0, 0};
  statefulStatusCacheGroups_[2] = {StatefulStatusCacheGroup::Object2400_4400, 0, 0, 0, 0, 0, 0};
  statefulStatusCacheGroups_[3] = {StatefulStatusCacheGroup::Object0300, 0, 0, 0, 0, 0, 0};
}

R70GatewayV2::StatefulStatusCacheGroupWindow* R70GatewayV2::statefulStatusCacheGroupWindow(
    StatefulStatusCacheGroup group) {
  if (group == StatefulStatusCacheGroup::None) {
    return 0;
  }
  for (uint8_t index = 0; index < kStatefulStatusCacheGroupCount; ++index) {
    if (statefulStatusCacheGroups_[index].group == group) {
      return &statefulStatusCacheGroups_[index];
    }
  }
  return 0;
}

const R70GatewayV2::StatefulStatusCacheGroupWindow* R70GatewayV2::statefulStatusCacheGroupWindow(
    StatefulStatusCacheGroup group) const {
  return const_cast<R70GatewayV2*>(this)->statefulStatusCacheGroupWindow(group);
}

void R70GatewayV2::autoDisableStatefulStatusCache(uint8_t reasonCode, const char* reason) {
  if (!statefulStatusCacheHasEverBeenActive_) {
    return;
  }
  if (!postBootstrapStatusCacheRuntimeEnabled_) {
    statefulStatusCacheDisabledReasonCode_ = reasonCode;
    return;
  }
  postBootstrapStatusCacheRuntimeEnabled_ = false;
  statefulStatusCacheLastEnabled_ = false;
  clearStatefulStatusCache();
  statefulStatusCacheDisabledReasonCode_ = reasonCode;
  char line[128] = {};
  snprintf(line,
           sizeof(line),
           "FEATURE_AUTO_DISABLED feature=%s reason=%s",
           regressionFeatureName(RegressionFeature::PostBootstrapStatusCache),
           reason != 0 ? reason : "unspecified");
  debugCli_.logText(line);
}

void R70GatewayV2::invalidateStatefulStatusCache(uint8_t subnet, uint8_t device, uint8_t channel) {
  for (uint16_t index = 0; index < kStatefulStatusCacheCapacity; ++index) {
    StatefulStatusCacheEntry& entry = statefulStatusCache_[index];
    if (!entry.used || entry.subnet != subnet || entry.device != device) {
      continue;
    }
    if (channel != 0 && entry.channel != 0 && entry.channel != channel) {
      continue;
    }
    entry.valid = false;
    entry.inflight = false;
    entry.waiterCount = 0;
    entry.inflightExpiresMs = 0;
  }
}

R70GatewayV2::StatefulStatusCacheEntry* R70GatewayV2::findStatefulStatusCacheEntry(const NormalizedRequest& request) {
  for (uint16_t index = 0; index < kStatefulStatusCacheCapacity; ++index) {
    StatefulStatusCacheEntry& entry = statefulStatusCache_[index];
    if (!entry.used ||
        entry.subnet != request.canId.subnet ||
        entry.device != request.canId.device ||
        entry.channel != request.objectAddress.channelNumber ||
        entry.objectKey != normalizedObjectKey(request) ||
        entry.fieldId != request.fieldId) {
      continue;
    }
    if (request.tcpWrappedCan || request.origin == RequestOrigin::Tcp) {
      if (entry.requestLength != request.frame.length) {
        continue;
      }
    bool exactBytes = true;
      for (uint8_t byteIndex = 0; byteIndex < entry.requestLength && byteIndex < sizeof(entry.requestData); ++byteIndex) {
        if (entry.requestData[byteIndex] != request.frame.data[byteIndex]) {
          exactBytes = false;
          break;
        }
      }
      if (!exactBytes) {
        continue;
      }
    }
    return &entry;
  }
  return 0;
}

const R70GatewayV2::StatefulStatusCacheEntry* R70GatewayV2::findStatefulStatusCacheEntry(
    const NormalizedRequest& request) const {
  return const_cast<R70GatewayV2*>(this)->findStatefulStatusCacheEntry(request);
}

R70GatewayV2::StatefulStatusCacheEntry* R70GatewayV2::reserveStatefulStatusCacheEntry(
    const NormalizedRequest& request) {
  StatefulStatusCacheEntry* reusable = 0;
  for (uint16_t index = 0; index < kStatefulStatusCacheCapacity; ++index) {
    StatefulStatusCacheEntry& entry = statefulStatusCache_[index];
    if (entry.used &&
        entry.subnet == request.canId.subnet &&
        entry.device == request.canId.device &&
        entry.channel == request.objectAddress.channelNumber &&
        entry.objectKey == normalizedObjectKey(request) &&
        entry.fieldId == request.fieldId) {
      reusable = &entry;
      break;
    }
    if (!entry.used && reusable == 0) {
      reusable = &entry;
    }
  }
  if (reusable == 0) {
    reusable = &statefulStatusCache_[0];
  }
  const Frame cachedReplyFrame = reusable->replyFrame;
  const uint32_t cachedUpdateMs = reusable->lastUpdateMs;
  const bool cachedValid = reusable->valid;
  *reusable = {};
  reusable->used = true;
  reusable->valid = cachedValid;
  reusable->replyFrame = cachedReplyFrame;
  reusable->lastUpdateMs = cachedUpdateMs;
  reusable->subnet = request.canId.subnet;
  reusable->device = request.canId.device;
  reusable->channel = request.objectAddress.channelNumber;
  reusable->objectKey = normalizedObjectKey(request);
  reusable->fieldId = request.fieldId;
  reusable->requestLength = request.frame.length;
  for (uint8_t index = 0; index < request.frame.length && index < sizeof(reusable->requestData); ++index) {
    reusable->requestData[index] = request.frame.data[index];
  }
  return reusable;
}

bool R70GatewayV2::postBootstrapStatusCacheFeatureEnabled() const {
  return postBootstrapStatusCacheRuntimeEnabled_;
}

bool R70GatewayV2::postBootstrapStatusCacheActive(uint32_t nowMs) const {
  return statefulStatusCacheDisabledReason(postBootstrapStatusCacheFeatureEnabled(),
                                           tcpServer_.connected(),
                                           state_.activeAmpCount(),
                                           firstAmpDiscoveryMs_,
                                           lastAmpDiscoveryMs_,
                                           lastActiveAmpCountChangeMs_,
                                           bootstrapCompleteStableSinceMs_,
                                           counters_.bootstrap_nodes_incomplete,
                                           nowMs) == kStatefulStatusCacheActive;
}

void R70GatewayV2::printStatefulStatusCacheActivationStatus(Print& out, uint32_t nowMs) const {
  const uint32_t firstAmpSeenAgeMs =
      firstAmpDiscoveryMs_ != 0 && nowMs >= firstAmpDiscoveryMs_ ? nowMs - firstAmpDiscoveryMs_ : 0u;
  const uint32_t activeAmpCountStableMs =
      lastActiveAmpCountChangeMs_ != 0 && nowMs >= lastActiveAmpCountChangeMs_
          ? nowMs - lastActiveAmpCountChangeMs_
          : 0u;
  const uint32_t lastNewAmpAgeMs =
      lastAmpDiscoveryMs_ != 0 && nowMs >= lastAmpDiscoveryMs_ ? nowMs - lastAmpDiscoveryMs_ : 0u;
  const bool r1ConnectedOk = tcpServer_.connected();
  const bool activeAmpCountOk = state_.activeAmpCount() > 0;
  const bool warmupOk =
      firstAmpDiscoveryMs_ != 0 &&
      nowMs >= firstAmpDiscoveryMs_ &&
      static_cast<int32_t>(nowMs - (firstAmpDiscoveryMs_ + statefulStatusCacheWarmupMs())) >= 0;
  const bool ampCountStabilityOk =
      lastAmpDiscoveryMs_ != 0 &&
      nowMs >= lastAmpDiscoveryMs_ &&
      static_cast<int32_t>(nowMs - (lastAmpDiscoveryMs_ + ampCountStableRequiredMs())) >= 0;
  const bool bootstrapSafeOk = counters_.bootstrap_nodes_incomplete == 0;
  out.print("STATEFUL_CACHE_ACTIVATION_STATUS feature_flag=");
  out.print(static_cast<unsigned>(ENABLE_POST_BOOTSTRAP_STATUS_CACHE != 0 ? 1u : 0u));
  out.print(" enabled=");
  out.print(static_cast<unsigned>(counters_.stateful_cache_enabled));
  out.print(" has_ever_been_active=");
  out.print(static_cast<unsigned>(statefulStatusCacheHasEverBeenActive_ ? 1u : 0u));
  out.print(" first_amp_seen_age_ms=");
  out.print(firstAmpSeenAgeMs);
  out.print(" warmup_required_ms=");
  out.print(statefulStatusCacheWarmupMs());
  out.print(" amp_count_stability_required_ms=");
  out.print(ampCountStableRequiredMs());
  out.print(" active_amp_count=");
  out.print(state_.activeAmpCount());
  out.print(" active_amp_count_stable_ms=");
  out.print(activeAmpCountStableMs);
  out.print(" last_new_amp_age_ms=");
  out.print(lastNewAmpAgeMs);
  out.print(" r1_connected=");
  out.print(static_cast<unsigned>(r1ConnectedOk ? 1u : 0u));
  out.print(" r1_connected_ok=");
  out.print(static_cast<unsigned>(r1ConnectedOk ? 1u : 0u));
  out.print(" active_amp_count_ok=");
  out.print(static_cast<unsigned>(activeAmpCountOk ? 1u : 0u));
  out.print(" warmup_ok=");
  out.print(static_cast<unsigned>(warmupOk ? 1u : 0u));
  out.print(" amp_count_stability_ok=");
  out.print(static_cast<unsigned>(ampCountStabilityOk ? 1u : 0u));
  out.print(" bootstrap_safe_ok=");
  out.print(static_cast<unsigned>(bootstrapSafeOk ? 1u : 0u));
  out.print(" disabled_reason=");
  out.print(statefulStatusCacheDisabledReasonName(statefulStatusCacheDisabledReasonCode_));
  out.print(" activation_blocker=");
  out.println(statefulStatusCacheDisabledReasonCode_ == kStatefulStatusCacheActive
                  ? "none"
                  : statefulStatusCacheDisabledReasonName(statefulStatusCacheDisabledReasonCode_));
}

void R70GatewayV2::printStatefulStatusCacheGroupSummaries(Print& out) const {
  for (uint8_t index = 0; index < kStatefulStatusCacheGroupCount; ++index) {
    const StatefulStatusCacheGroupWindow& window = statefulStatusCacheGroups_[index];
    out.print("STATEFUL_CACHE_GROUP group=");
    out.print(statefulStatusCacheGroupName(window.group));
    out.print(" cache_hits_10s=");
    out.print(window.cacheHits);
    out.print(" cache_misses_10s=");
    out.print(window.cacheMisses);
    out.print(" can_reads_prevented_10s=");
    out.print(window.canReadsPrevented);
    out.print(" forced_can_reads_10s=");
    out.print(window.forcedCanReads);
    out.print(" inflight_joined_10s=");
    out.print(window.inflightJoined);
    out.print(" cache_age_max_ms=");
    out.println(window.cacheAgeMaxMs);
  }
}

void R70GatewayV2::printR1FreshnessSummary(Print& out, uint32_t nowMs) const {
  uint32_t maxAgeMs = 0;
  uint8_t devicesOver2s = 0;
  uint8_t devicesOver5s = 0;
  uint8_t worstSubnet = 0;
  uint8_t worstDevice = 0;
  uint8_t subnetOver2s[DebugCounters::kSubnetCount] = {};
  uint8_t subnetOver5s[DebugCounters::kSubnetCount] = {};
  uint32_t subnetMaxAgeMs[DebugCounters::kSubnetCount] = {};

  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    const uint32_t ageMs = r1ReplyAgeMs(*ampState, nowMs);
    if (ageMs > maxAgeMs) {
      maxAgeMs = ageMs;
      worstSubnet = ampState->subnet;
      worstDevice = ampState->device;
    }
    if (ampState->subnet < DebugCounters::kSubnetCount && ageMs > subnetMaxAgeMs[ampState->subnet]) {
      subnetMaxAgeMs[ampState->subnet] = ageMs;
    }
    if (ageMs > kR1FreshnessWarnMs) {
      ++devicesOver2s;
      if (ampState->subnet < DebugCounters::kSubnetCount) {
        ++subnetOver2s[ampState->subnet];
      }
    }
    if (ageMs > kR1FreshnessCriticalMs) {
      ++devicesOver5s;
      if (ampState->subnet < DebugCounters::kSubnetCount) {
        ++subnetOver5s[ampState->subnet];
      }
    }
  }

  char line[256] = {};
  snprintf(line,
           sizeof(line),
           "R1_FRESHNESS_SUMMARY active_amp_count=%u max_r1_reply_age_ms=%lu devices_over_2s_without_reply=%u devices_over_5s_without_reply=%u worst_r1_stale_device=%u.%02u pending_tcp_replies=%u pending_can_reads=%u startup_freshness_active=%u",
           static_cast<unsigned>(state_.activeAmpCount()),
           static_cast<unsigned long>(maxAgeMs),
           static_cast<unsigned>(devicesOver2s),
           static_cast<unsigned>(devicesOver5s),
           static_cast<unsigned>(worstSubnet),
           static_cast<unsigned>(worstDevice),
           static_cast<unsigned>(pendingTcpReplyCount()),
           static_cast<unsigned>(scheduler_.queueDepth()),
           static_cast<unsigned>(startupFreshnessProtectionActive(nowMs) ? 1u : 0u));
  out.println(line);

  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    if (subnetOver2s[subnet] == 0 && subnetOver5s[subnet] == 0 && subnetMaxAgeMs[subnet] == 0) {
      continue;
    }
    snprintf(line,
             sizeof(line),
             "R1_FRESHNESS_SUBNET subnet=%u devices_over_2s_without_reply=%u devices_over_5s_without_reply=%u max_r1_reply_age_ms=%lu",
             static_cast<unsigned>(subnet),
             static_cast<unsigned>(subnetOver2s[subnet]),
             static_cast<unsigned>(subnetOver5s[subnet]),
             static_cast<unsigned long>(subnetMaxAgeMs[subnet]));
    out.println(line);
  }

  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    const uint32_t ageMs = r1ReplyAgeMs(*ampState, nowMs);
    if (ageMs <= kR1FreshnessWarnMs && ampState->lastTcpTxToR1Ms != 0) {
      continue;
    }
    const uint32_t lastCanRxAgeMs = ampState->lastCanRxMs != 0 && nowMs >= ampState->lastCanRxMs
                                        ? nowMs - ampState->lastCanRxMs
                                        : 0u;
    const uint32_t lastCacheReplyAgeMs =
        ampState->lastCacheReplyMs != 0 && nowMs >= ampState->lastCacheReplyMs
            ? nowMs - ampState->lastCacheReplyMs
            : 0u;
    snprintf(line,
             sizeof(line),
             "R1_FRESHNESS_DEVICE node=%u.%02u last_tcp_reply_to_r1_ms=%lu r1_reply_age_ms=%lu last_can_rx_age_ms=%lu last_cache_reply_age_ms=%lu",
             static_cast<unsigned>(ampState->subnet),
             static_cast<unsigned>(ampState->device),
             static_cast<unsigned long>(ampState->lastTcpTxToR1Ms),
             static_cast<unsigned long>(ageMs),
             static_cast<unsigned long>(lastCanRxAgeMs),
             static_cast<unsigned long>(lastCacheReplyAgeMs));
    out.println(line);
  }
}

void R70GatewayV2::logR1FreshnessSummary(uint32_t nowMs, const char* prefix) const {
  uint8_t devicesOver5s = 0;
  const uint32_t maxAgeMs = maxR1ReplyAgeMs(nowMs, &devicesOver5s);
  char line[192] = {};
  snprintf(line,
           sizeof(line),
           "%s pending_tcp_replies=%u pending_can_reads=%u max_r1_reply_age_ms=%lu devices_over_5s=%u",
           prefix != 0 ? prefix : "R1_FRESHNESS_EVENT",
           static_cast<unsigned>(pendingTcpReplyCount()),
           static_cast<unsigned>(scheduler_.queueDepth()),
           static_cast<unsigned long>(maxAgeMs),
           static_cast<unsigned>(devicesOver5s));
  debugCli_.logText(line);
}

void R70GatewayV2::noteSemanticMirrorCanReply(const NormalizedRequest& reply,
                                              uint32_t nowMs,
                                              bool forwardedToR1) {
  const uint16_t rawObjectKey = normalizedObjectKey(reply);
  const uint16_t displayObjectKey = semanticMirrorDisplayObjectKey(rawObjectKey);
  const bool focusObject = isSemanticMirrorFocusObjectKey(rawObjectKey);
  if (focusObject) {
    ++semanticMirrorWindow_.learnAttempts;
    if (shouldLogLowRateCounter(semanticMirrorWindow_.learnAttempts)) {
      char line[192] = {};
      snprintf(line,
               sizeof(line),
               "MIRROR_LEARN_ATTEMPT node=%u.%02u object=%04X raw_object=%04X field=%s forwarded=%u",
               static_cast<unsigned>(reply.canId.subnet),
               static_cast<unsigned>(reply.canId.device),
               static_cast<unsigned>(displayObjectKey),
               static_cast<unsigned>(rawObjectKey),
               fieldIdName(reply.fieldId),
               static_cast<unsigned>(forwardedToR1 ? 1u : 0u));
      debugCli_.logText(line);
    }
  }

  const char* skipReason = 0;
  if (!reply.valid) {
    skipReason = "not_normalized";
  } else if (reply.canId.device == 0) {
    skipReason = "unsupported";
  } else if (!reply.admin.valid) {
    skipReason = "parser_mismatch";
  } else if (!isReadFunction(reply.admin.functionCode)) {
    skipReason = "write_sensitive";
  } else if (isSegmentedType(reply.admin.telegramType)) {
    skipReason = "segmented";
  } else if (reply.admin.telegramType != TelegramType::Unstructured) {
    skipReason = "unsupported";
  } else if (reply.objectAddress.hasRecord || reply.objectAddress.hasSegment) {
    skipReason = "segmented";
  } else if (reply.mapping == 0 || reply.fieldId == FieldId::Unknown) {
    skipReason = "parser_mismatch";
  } else if (reply.frame.length <= 3) {
    skipReason = "invalid_payload";
  }

  if (skipReason != 0) {
    if (focusObject) {
      ++semanticMirrorWindow_.learnSkips;
      if (shouldLogLowRateCounter(semanticMirrorWindow_.learnSkips)) {
        char line[192] = {};
        snprintf(line,
                 sizeof(line),
                 "MIRROR_LEARN_SKIP node=%u.%02u object=%04X raw_object=%04X reason=%s",
                 static_cast<unsigned>(reply.canId.subnet),
                 static_cast<unsigned>(reply.canId.device),
                 static_cast<unsigned>(displayObjectKey),
                 static_cast<unsigned>(rawObjectKey),
                 skipReason);
        debugCli_.logText(line);
      }
    }
    return;
  }

  SemanticMirrorEntry* entry = findSemanticMirrorEntry(reply);
  if (entry == 0) {
    entry = reserveSemanticMirrorEntry(reply);
  }
  if (entry == 0) {
    return;
  }

  if (entry->fieldId == FieldId::status_pwr_ok && entry->invalidatedByWrite) {
    const uint8_t powerOkValue = reply.frame.data[3];
    entry->replyFrame = reply.frame;
    entry->lastUpdateMs = nowMs;
    entry->learned = true;
    entry->refreshInflight = false;
    entry->refreshEnqueuedMs = 0;
    entry->lastRefreshBlockedReason = kSemanticRefreshBlockedNone;
    entry->lastRefreshBlockedMs = 0;

    const bool powerOkConfirmed = powerOkValue != 0u;
    entry->invalidatedByWrite = !powerOkConfirmed;
    if (!powerOkConfirmed) {
      // Do not let the generic field cache bypass the guarded semantic entry
      // with this still-transitional OFF reply.
      state_.invalidateFieldCache(reply.canId.subnet,
                                  reply.canId.device,
                                  FieldId::status_pwr_ok);
    }

    char line[192] = {};
    snprintf(line,
             sizeof(line),
             "SEMANTIC_POWER_OK_READBACK node=%u.%02u object=%04X value=%02X accepted=%u retry=%u",
             static_cast<unsigned>(reply.canId.subnet),
             static_cast<unsigned>(reply.canId.device),
             static_cast<unsigned>(rawObjectKey),
             static_cast<unsigned>(powerOkValue),
             static_cast<unsigned>(powerOkConfirmed ? 1u : 0u),
             static_cast<unsigned>(powerOkConfirmed ? 0u : 1u));
    debugCli_.logText(line);
    return;
  }

  if (entry->invalidatedByWrite) {
    if (entry->fieldId == FieldId::settings_power &&
        entry->replyFrame.length >= 4 && entry->replyFrame.data[3] == 0u) {
      // An OFF write is authoritative.  A late pre-write read may still say
      // ON, while later reads receive no answer because the node is already
      // down; neither case should undo the immediate OFF indication to R1.
      entry->refreshInflight = false;
      entry->refreshEnqueuedMs = 0;
      return;
    }
    const uint32_t verifyDelayMs = entry->fieldId == FieldId::settings_power
                                       ? kPowerWriteThroughVerifyDelayMs
                                       : kMuteWriteThroughVerifyDelayMs;
    // A read already in flight can legitimately contain the old value.  Keep
    // it only as the latest genuine readback candidate while the R1-facing
    // cache remains blocked.
    if (entry->lastUpdateMs <= nowMs && nowMs - entry->lastUpdateMs < verifyDelayMs) {
      entry->replyFrame = reply.frame;
      entry->refreshInflight = false;
      entry->refreshEnqueuedMs = 0;
      entry->lastRefreshBlockedReason = kSemanticRefreshWriteReadbackCaptured;
      return;
    }
  }

  const bool wasLearned = entry->learned;
  entry->replyFrame = reply.frame;
  entry->lastUpdateMs = nowMs;
  if (forwardedToR1 || entry->invalidatedByWrite) {
    entry->learned = true;
    entry->invalidatedByWrite = false;
  }
  entry->refreshInflight = false;
  entry->refreshEnqueuedMs = 0;
  entry->lastRefreshBlockedReason = kSemanticRefreshBlockedNone;
  entry->lastRefreshBlockedMs = 0;
  if (focusObject && forwardedToR1 && !wasLearned) {
    ++semanticMirrorWindow_.learnSuccesses;
    char line[192] = {};
    snprintf(line,
             sizeof(line),
             "MIRROR_LEARN_SUCCESS node=%u.%02u object=%04X raw_object=%04X field=%s",
             static_cast<unsigned>(reply.canId.subnet),
             static_cast<unsigned>(reply.canId.device),
             static_cast<unsigned>(displayObjectKey),
             static_cast<unsigned>(rawObjectKey),
             fieldIdName(reply.fieldId));
    debugCli_.logText(line);
  }
}

bool R70GatewayV2::tryHandleSemanticMirrorRequest(const NormalizedRequest& request,
                                                  RequestTrace* trace,
                                                  uint32_t nowMs) {
  const uint16_t rawObjectKey = normalizedObjectKey(request);
  const uint16_t displayObjectKey = semanticMirrorDisplayObjectKey(rawObjectKey);
  rememberSemanticMirrorRequestTemplate(request);
  if (!semanticMirrorActive(nowMs)) {
    return false;
  }
  if (!isSemanticMirrorRequest(request)) {
    markTraceMirrorForceCan(trace, TraceMirrorDecision::ForcedCanNotSafe);
    if (request.valid &&
        request.tcpWrappedCan &&
        request.canId.device != 0 &&
        request.admin.valid &&
        isReadFunction(request.admin.functionCode) &&
        request.admin.telegramType == TelegramType::Unstructured &&
        shouldLogLowRateCounter(semanticMirrorWindow_.cacheMisses + 1u)) {
      char line[160] = {};
      snprintf(line,
               sizeof(line),
               "SEMANTIC_MIRROR_FORCE_CAN node=%u.%02u object=%04X reason=not_safe",
               static_cast<unsigned>(request.canId.subnet),
               static_cast<unsigned>(request.canId.device),
               static_cast<unsigned>(displayObjectKey));
      debugCli_.logText(line);
    }
    return false;
  }

  SemanticMirrorGroupWindow* groupWindow = 0;
  const SemanticMirrorGroup group = classifySemanticMirrorGroup(request);
  const uint32_t targetAgeMs = semanticMirrorGroupTargetAgeMs(group);
  const uint32_t hardMaxAgeMs = semanticMirrorGroupHardMaxAgeMs(group);
  for (uint8_t index = 0; index < kSemanticMirrorGroupCount; ++index) {
    if (semanticMirrorGroups_[index].group == group) {
      groupWindow = &semanticMirrorGroups_[index];
      break;
    }
  }

  SemanticMirrorEntry* entry = findSemanticMirrorEntry(request);
  if (entry == 0 || !entry->learned ||
      entry->lastUpdateMs == 0 || entry->lastUpdateMs > nowMs) {
    markTraceMirrorForceCan(trace,
                            entry != 0 && entry->invalidatedByWrite
                                ? TraceMirrorDecision::ForcedCanWriteInvalidated
                                : TraceMirrorDecision::ForcedCanMissing);
    ++semanticMirrorWindow_.cacheMisses;
    if (groupWindow != 0) {
      ++groupWindow->misses;
    }
    if (shouldLogLowRateCounter(semanticMirrorWindow_.cacheMisses)) {
      char line[160] = {};
      snprintf(line,
               sizeof(line),
               "MIRROR_FORCE_CAN node=%u.%02u object=%04X reason=%s",
               static_cast<unsigned>(request.canId.subnet),
               static_cast<unsigned>(request.canId.device),
               static_cast<unsigned>(displayObjectKey),
               static_cast<const char*>(
                   entry != 0 && entry->invalidatedByWrite ? "write_invalidated" : "missing"));
      debugCli_.logText(line);
    }
    return false;
  }

  if (entry->invalidatedByWrite) {
    const bool commandedPowerOff =
        entry->fieldId == FieldId::settings_power &&
        entry->replyFrame.length >= 4 && entry->replyFrame.data[3] == 0u;
    if (commandedPowerOff) {
      if (entry->lastRefreshBlockedReason != kSemanticPowerOffWriteAckConfirmed) {
        // Do not claim OFF until the amplifier acknowledged the write.
        markTraceMirrorForceCan(trace, TraceMirrorDecision::ForcedCanWriteInvalidated);
        return false;
      }
      // Once acknowledged, there is no useful readback window: the node is
      // intentionally going down.  Keep serving the confirmed OFF value.
    } else if (entry->fieldId == FieldId::settings_power) {
      // Release each amplifier independently on its first genuine CAN read
      // response after power-on.  Until then R1 correctly keeps that node in
      // its waiting state instead of releasing all nodes on one timer edge.
      markTraceMirrorForceCan(trace, TraceMirrorDecision::ForcedCanWriteInvalidated);
      return false;
    } else {
      // Mute stays responsive from the commanded value.  A single bounded
      // maintenance read verifies it after 250 ms without an R1 read flood.
    }
  }

  if (entry->refreshInflight &&
      entry->refreshEnqueuedMs != 0 &&
      static_cast<int32_t>(nowMs - (entry->refreshEnqueuedMs + hardMaxAgeMs)) >= 0) {
    entry->refreshInflight = false;
    entry->refreshEnqueuedMs = 0;
  }

  const uint32_t valueAgeMs = nowMs - entry->lastUpdateMs;
  const AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device);
  const uint32_t nodeCanAgeMs =
      ampState != 0 && ampState->lastCanRxMs != 0 && ampState->lastCanRxMs <= nowMs
          ? nowMs - ampState->lastCanRxMs
          : 0xFFFFFFFFu;
  const uint32_t ageMs = nodeCanAgeMs < valueAgeMs ? nodeCanAgeMs : valueAgeMs;
  const bool nodeRecentlyConfirmed = nodeCanAgeMs <= kNodeFreshMirrorServeMaxAgeMs;
  // During the bounded post-write hold, power/mute readback remains optimistic
  // and cannot turn an R1 read into a synchronous CAN wait.
  if (ageMs > hardMaxAgeMs && !entry->invalidatedByWrite) {
    markTraceMirrorForceCan(trace, TraceMirrorDecision::ForcedCanStale);
    ++semanticMirrorWindow_.cacheMisses;
    ++semanticMirrorWindow_.staleForcedCanReads;
    if (groupWindow != 0) {
      ++groupWindow->misses;
    }
    noteObjectCacheStale(request);
    if (shouldLogLowRateCounter(semanticMirrorWindow_.staleForcedCanReads)) {
      char line[160] = {};
      snprintf(line,
               sizeof(line),
               "MIRROR_FORCE_CAN node=%u.%02u object=%04X reason=stale_hardmax age_ms=%lu",
               static_cast<unsigned>(request.canId.subnet),
               static_cast<unsigned>(request.canId.device),
               static_cast<unsigned>(displayObjectKey),
               static_cast<unsigned long>(ageMs));
      debugCli_.logText(line);
      snprintf(line,
               sizeof(line),
               "SEMANTIC_FORCE_CAN_STALE_HARDMAX node=%u.%02u object=%04X age_ms=%lu",
               static_cast<unsigned>(request.canId.subnet),
               static_cast<unsigned>(request.canId.device),
               static_cast<unsigned>(displayObjectKey),
               static_cast<unsigned long>(ageMs));
      debugCli_.logText(line);
    }
    return false;
  }

  if (valueAgeMs > hardMaxAgeMs &&
      nodeRecentlyConfirmed &&
      shouldLogLowRateCounter(semanticMirrorWindow_.cacheHits + 1u)) {
    char line[192] = {};
    snprintf(line,
             sizeof(line),
             "NODE_FRESH_MIRROR_SERVE node=%u.%02u object=%04X value_age_ms=%lu node_can_age_ms=%lu",
             static_cast<unsigned>(request.canId.subnet),
             static_cast<unsigned>(request.canId.device),
             static_cast<unsigned>(displayObjectKey),
             static_cast<unsigned long>(valueAgeMs),
             static_cast<unsigned long>(nodeCanAgeMs));
    debugCli_.logText(line);
  }

  uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
  uint8_t tcpFrameLength = 0;
  if (!R70Frame::encodeWrappedCanFrame(entry->replyFrame, tcpFrame, sizeof(tcpFrame), tcpFrameLength) ||
      !tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
    return false;
  }

  ++semanticMirrorWindow_.cacheHits;
  ++counters_.cache_hits;
  ++counters_.cache_replies_sent;
  ++counters_.stateful_cache_hits_10s;
  ++counters_.stateful_can_reads_prevented_10s;
  ++counters_.allowlist_hits;
  if (groupWindow != 0) {
    ++groupWindow->hits;
    if (ageMs > groupWindow->maxAgeMs) {
      groupWindow->maxAgeMs = static_cast<uint16_t>(ageMs > 0xFFFFu ? 0xFFFFu : ageMs);
    }
  }
  noteCacheHitByField(request.fieldId, counters_);
  debugCli_.logTcpTxWrappedCan(entry->replyFrame, tcpFrame, tcpFrameLength);
  if (shouldLogLowRateCounter(semanticMirrorWindow_.cacheHits)) {
    char line[160] = {};
    snprintf(line,
             sizeof(line),
             "ACTIVE_MIRROR_SERVE object=%04X node=%u.%02u source=mirror age_ms=%lu",
             static_cast<unsigned>(displayObjectKey),
             static_cast<unsigned>(request.canId.subnet),
             static_cast<unsigned>(request.canId.device),
             static_cast<unsigned long>(ageMs));
    debugCli_.logText(line);
    snprintf(line,
             sizeof(line),
             "SEMANTIC_RESPONSE_REUSED node=%u.%02u object=%04X source=mirror prevented_can=1",
             static_cast<unsigned>(request.canId.subnet),
             static_cast<unsigned>(request.canId.device),
             static_cast<unsigned>(displayObjectKey));
    debugCli_.logText(line);
  }
  state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
  noteSemanticFamilyUpdated(request, nowMs, 1u, true);
  noteStartupBlockerReplyForwarded(request, nowMs);
  markTraceMirrorReply(trace, nowMs);
  noteObjectCacheReply(request, nowMs);
  entry->lastServedMs = nowMs;
  if (ageMs > 1000u) {
    ++semanticMirrorWindow_.staleServedOver1s;
  }
  if (ageMs > 1500u) {
    ++semanticMirrorWindow_.staleServedOver1500ms;
  }
  if (ageMs > targetAgeMs) {
    if (entry->staleServedCountWindow != 0xFFFFFFFFu) {
      ++entry->staleServedCountWindow;
    }
    char line[160] = {};
    snprintf(line,
             sizeof(line),
             "SEMANTIC_SERVE_STALE_BUT_ALLOWED node=%u.%02u object=%04X age_ms=%lu",
             static_cast<unsigned>(request.canId.subnet),
             static_cast<unsigned>(request.canId.device),
             static_cast<unsigned>(displayObjectKey),
             static_cast<unsigned long>(ageMs));
    debugCli_.logText(line);
  }

  if (valueAgeMs >= targetAgeMs) {
    const bool fifoPressure = scheduler_.normalQueueDepth() != 0 ||
                              counters_.normal_read_queue_oldest_age_ms != 0 ||
                              pendingTcpReplyCount() != 0 ||
                              scheduler_.queueDepth() != 0;
    if (fifoPressure) {
      markTraceBackgroundRefresh(trace, TraceBackgroundRefreshState::SkippedCanPending);
      ++semanticMirrorWindow_.refreshSkippedDueToFifoPressure;
    } else {
      markTraceBackgroundRefresh(trace, TraceBackgroundRefreshState::None);
      ++semanticMirrorWindow_.refreshIdleOnlyCount;
    }
  }

  return true;
}

void R70GatewayV2::logSemanticMirrorSummary() const {
  uint16_t activeEntries = 0;
  uint16_t learnedEntries = 0;
  uint16_t learnedByGroup[kSemanticMirrorGroupCount] = {};
  uint16_t ageByGroup[kSemanticMirrorGroupCount] = {};
  uint16_t overTargetByGroup[kSemanticMirrorGroupCount] = {};
  uint16_t overHardMaxByGroup[kSemanticMirrorGroupCount] = {};
  uint8_t activeGroups = 0;
  uint16_t entriesOverTarget = 0;
  uint16_t entriesOverHardMax = 0;

  const uint32_t nowMs = millis();
  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    const SemanticMirrorEntry& entry = semanticMirror_[index];
    if (!entry.used) {
      continue;
    }
    ++activeEntries;
    if (!entry.learned) {
      continue;
    }
    ++learnedEntries;
    NormalizedRequest request = {};
    request.valid = true;
    request.canId.subnet = entry.subnet;
    request.canId.device = entry.device;
    request.objectAddress.channelNumber = entry.channel;
    request.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
    request.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
    request.frame.length = 3;
    request.frame.data[1] = request.objectAddress.boxNumber;
    request.frame.data[2] = request.objectAddress.objectNumber;
    const SemanticMirrorGroup group = classifySemanticMirrorGroup(request);
    if (group == SemanticMirrorGroup::None) {
      continue;
    }
    const uint8_t groupIndex = static_cast<uint8_t>(group) - 1u;
    if (groupIndex >= kSemanticMirrorGroupCount) {
      continue;
    }
    ++learnedByGroup[groupIndex];
    if (learnedByGroup[groupIndex] == 1u) {
      ++activeGroups;
    }
    if (entry.lastUpdateMs != 0 && nowMs >= entry.lastUpdateMs) {
      const uint32_t ageMs = nowMs - entry.lastUpdateMs;
      if (ageMs > ageByGroup[groupIndex]) {
        ageByGroup[groupIndex] = static_cast<uint16_t>(ageMs > 0xFFFFu ? 0xFFFFu : ageMs);
      }
      const uint32_t targetAgeMs = semanticMirrorGroupTargetAgeMs(group);
      const uint32_t hardMaxAgeMs = semanticMirrorGroupHardMaxAgeMs(group);
      if (ageMs > targetAgeMs) {
        ++entriesOverTarget;
        ++overTargetByGroup[groupIndex];
      }
      if (ageMs > hardMaxAgeMs) {
        ++entriesOverHardMax;
        ++overHardMaxByGroup[groupIndex];
      }
    }
  }

  char line[224] = {};
  snprintf(line,
           sizeof(line),
           "SEMANTIC_MIRROR_SUMMARY active=%u learned_entries=%u mirror_served_10s=%lu mirror_forced_can_10s=%lu mirror_stale_10s=%lu mirror_write_invalidations_10s=%lu bg_refresh_sent_10s=%lu bg_refresh_skipped_10s=%lu bg_refresh_rate_limited_10s=%lu refresh_idle_only_count=%lu refresh_skipped_due_to_fifo_pressure=%lu",
           static_cast<unsigned>(activeEntries),
           static_cast<unsigned>(learnedEntries),
           static_cast<unsigned long>(semanticMirrorWindow_.cacheHits),
           static_cast<unsigned long>(semanticMirrorWindow_.cacheMisses),
           static_cast<unsigned long>(semanticMirrorWindow_.staleForcedCanReads),
           static_cast<unsigned long>(semanticMirrorWindow_.writeInvalidations),
           static_cast<unsigned long>(semanticMirrorWindow_.backgroundRefreshSent),
           static_cast<unsigned long>(semanticMirrorWindow_.backgroundRefreshSkipped),
           static_cast<unsigned long>(semanticMirrorWindow_.backgroundRefreshRateLimited),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshIdleOnlyCount),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshSkippedDueToFifoPressure));
  debugCli_.logText(line);

  snprintf(line,
           sizeof(line),
           "SEMANTIC_FRESHNESS_SUMMARY active_groups=%u entries_over_target=%u entries_over_hardmax=%u protected_refresh_sent=%lu protected_refresh_delayed=%lu protected_refresh_age_max=%u stale_served_over_1s=%lu stale_served_over_1500ms=%lu refresh_blocked_by_tcp_pressure=%lu refresh_blocked_by_queue=%lu protected_refresh_bypassed_tcp=%lu protected_refresh_bypassed_shaping=%lu protected_refresh_overdue=%lu priority_refresh_sent_10s=%lu proactive_refresh_sent_10s=%lu refresh_skipped_pressure_10s=%lu",
           static_cast<unsigned>(activeGroups),
           static_cast<unsigned>(entriesOverTarget),
           static_cast<unsigned>(entriesOverHardMax),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshSent),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshDelayed),
           static_cast<unsigned>(semanticMirrorWindow_.protectedRefreshAgeMax),
           static_cast<unsigned long>(semanticMirrorWindow_.staleServedOver1s),
           static_cast<unsigned long>(semanticMirrorWindow_.staleServedOver1500ms),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshBlockedByTcpPressure),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshBlockedByQueue),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshBypassedTcp),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshBypassedShaping),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshOverdue),
           static_cast<unsigned long>(semanticMirrorWindow_.priorityRefreshSent),
           static_cast<unsigned long>(semanticMirrorWindow_.proactiveRefreshSent),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshSkippedPressure));
  debugCli_.logText(line);

  for (uint8_t index = 0; index < kSemanticMirrorGroupCount; ++index) {
    const SemanticMirrorGroupWindow& groupWindow = semanticMirrorGroups_[index];
    snprintf(line,
             sizeof(line),
             "SEMANTIC_MIRROR_GROUP group=%s learned=%u hits=%lu misses=%lu refresh_sent=%lu max_age_ms=%u",
             semanticMirrorGroupName(groupWindow.group),
             static_cast<unsigned>(learnedByGroup[index]),
             static_cast<unsigned long>(groupWindow.hits),
             static_cast<unsigned long>(groupWindow.misses),
             static_cast<unsigned long>(groupWindow.refreshSent),
             static_cast<unsigned>(ageByGroup[index]));
    debugCli_.logText(line);
    snprintf(line,
             sizeof(line),
             "SEMANTIC_FRESHNESS_GROUP group=%s max_age_ms=%u over_target=%u over_hardmax=%u refresh_sent=%lu protected_refresh_sent=%lu protected_refresh_delayed=%lu",
             semanticMirrorGroupName(groupWindow.group),
             static_cast<unsigned>(ageByGroup[index]),
             static_cast<unsigned>(overTargetByGroup[index]),
             static_cast<unsigned>(overHardMaxByGroup[index]),
             static_cast<unsigned long>(groupWindow.refreshSent),
             static_cast<unsigned long>(groupWindow.protectedRefreshSent),
             static_cast<unsigned long>(groupWindow.protectedRefreshDelayed));
    debugCli_.logText(line);
  }
}

void R70GatewayV2::logSemanticHealth(uint32_t nowMs) const {
  uint32_t protectedMaxAgeMs = 0;
  uint16_t protectedOver500 = 0;
  uint16_t protectedOver900 = 0;
  uint16_t protectedOver1200 = 0;
  uint16_t maintenanceBacklog = 0;

  for (uint16_t index = 0; index < kSemanticMirrorCapacity; ++index) {
    const SemanticMirrorEntry& entry = semanticMirror_[index];
    if (!entry.used || !entry.learned || entry.lastUpdateMs == 0 || entry.lastUpdateMs > nowMs) {
      continue;
    }
    NormalizedRequest request = {};
    request.valid = true;
    request.canId.subnet = entry.subnet;
    request.canId.device = entry.device;
    request.objectAddress.channelNumber = entry.channel;
    request.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
    request.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
    request.fieldId = entry.fieldId;
    request.admin.valid = true;
    request.admin.telegramType = entry.telegramType;
    const SemanticMirrorGroup group = classifySemanticMirrorGroup(request);
    if (!isProtectedSemanticMirrorGroup(group)) {
      continue;
    }
    const uint32_t ageMs = nowMs - entry.lastUpdateMs;
    if (ageMs > protectedMaxAgeMs) {
      protectedMaxAgeMs = ageMs;
    }
    if (ageMs > 500u) {
      ++protectedOver500;
    }
    if (ageMs > 900u) {
      ++protectedOver900;
    }
    if (ageMs > 1200u) {
      ++protectedOver1200;
    }
    if (ageMs > semanticMirrorGroupTargetAgeMs(group)) {
      ++maintenanceBacklog;
    }
  }

  char line[256] = {};
  snprintf(line,
           sizeof(line),
           "SEMANTIC_HEALTH protected_max_age_ms=%lu protected_over_500=%u protected_over_900=%u protected_over_1200=%u refresh_skipped_tcp=%lu refresh_skipped_queue=%lu maintenance_budget_used=%lu maintenance_backlog=%u maintenance_queue_depth=%u scheduler_consecutive_reads_max=%u",
           static_cast<unsigned long>(protectedMaxAgeMs),
           static_cast<unsigned>(protectedOver500),
           static_cast<unsigned>(protectedOver900),
           static_cast<unsigned>(protectedOver1200),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshBlockedByTcpPressure),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshBlockedByQueue),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshSent),
           static_cast<unsigned>(maintenanceBacklog),
           static_cast<unsigned>(scheduler_.maintenanceQueueDepth()),
           static_cast<unsigned>(slowCanWindow_.maxConsecutiveNodeRequests));
  debugCli_.logText(line);

  snprintf(line,
           sizeof(line),
           "SEMANTIC_RESERVED_BUDGET reserved_slots=%u used_slots=%lu unused_slots=%lu backlog=%u protected_refresh_queue_depth=%u",
           static_cast<unsigned>(kSemanticMirrorReservedSlotsPerSecond),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshSent),
           static_cast<unsigned long>(semanticMirrorWindow_.protectedRefreshSent >= kSemanticMirrorReservedSlotsPerSecond
                                          ? 0u
                                          : static_cast<uint32_t>(kSemanticMirrorReservedSlotsPerSecond) -
                                                semanticMirrorWindow_.protectedRefreshSent),
           static_cast<unsigned>(maintenanceBacklog),
           static_cast<unsigned>(scheduler_.maintenanceQueueDepth()));
  debugCli_.logText(line);

  snprintf(line,
           sizeof(line),
           "FIFO_RUNTIME fifo_queue_oldest_ms=%lu fifo_dispatch_blocked_ms=%lu inflight_join_count=%lu mirror_served_count=%lu refresh_idle_only_count=%lu refresh_skipped_due_to_fifo_pressure=%lu max_pending_tcp_oldest_ms=%lu",
           static_cast<unsigned long>(counters_.normal_read_queue_oldest_age_ms),
           static_cast<unsigned long>(counters_.normal_read_queue_oldest_age_ms),
           static_cast<unsigned long>(counters_.stateful_inflight_joined_10s),
           static_cast<unsigned long>(semanticMirrorWindow_.cacheHits),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshIdleOnlyCount),
           static_cast<unsigned long>(semanticMirrorWindow_.refreshSkippedDueToFifoPressure),
           static_cast<unsigned long>(maxPendingTcpOldestMs(nowMs)));
  debugCli_.logText(line);
}

bool R70GatewayV2::tryHandlePostBootstrapStatusCacheRequest(const NormalizedRequest& request,
                                                            RequestTrace* trace,
                                                            uint32_t nowMs) {
  if (!request.tcpWrappedCan || !isPostBootstrapStatusCacheRequest(request) || !postBootstrapStatusCacheActive(nowMs)) {
    return false;
  }

  StatefulStatusCacheGroupWindow* groupWindow =
      statefulStatusCacheGroupWindow(classifyStatefulStatusCacheGroup(request));
  StatefulStatusCacheEntry* entry = findStatefulStatusCacheEntry(request);
  if (entry != 0 && entry->valid && entry->lastUpdateMs <= nowMs) {
    const uint32_t ageMs = nowMs - entry->lastUpdateMs;
    if (ageMs <= postBootstrapStatusCacheTtlMs(request) && ageMs <= postBootstrapStatusCacheHardMaxAgeMs()) {
      uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
      uint8_t tcpFrameLength = 0;
      if (R70Frame::encodeWrappedCanFrame(entry->replyFrame, tcpFrame, sizeof(tcpFrame), tcpFrameLength) &&
          tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
        ++counters_.cache_hits;
        ++counters_.cache_replies_sent;
        ++counters_.stateful_cache_hits_10s;
        ++counters_.stateful_can_reads_prevented_10s;
        if (groupWindow != 0) {
          ++groupWindow->cacheHits;
          ++groupWindow->canReadsPrevented;
          if (ageMs > groupWindow->cacheAgeMaxMs) {
            groupWindow->cacheAgeMaxMs = static_cast<uint16_t>(ageMs);
          }
        }
        if (ageMs > counters_.stateful_cache_age_max_ms) {
          counters_.stateful_cache_age_max_ms = ageMs;
        }
        noteCacheHitByField(request.fieldId, counters_);
        debugCli_.logTcpTxWrappedCan(entry->replyFrame, tcpFrame, tcpFrameLength);
        state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
        noteSemanticFamilyUpdated(request, nowMs, 3u, true);
        noteStartupBlockerReplyForwarded(request, nowMs);
        markTraceCacheReply(trace, nowMs);
        noteObjectCacheReply(request, nowMs);
        if (shouldLogLowRateCounter(counters_.stateful_cache_hits_10s)) {
          char line[192] = {};
          snprintf(line,
                   sizeof(line),
                   "CACHE_SERVE node=%u.%02u object=%04X age_ms=%lu",
                   static_cast<unsigned>(request.canId.subnet),
                   static_cast<unsigned>(request.canId.device),
                   static_cast<unsigned>(normalizedObjectKey(request)),
                   static_cast<unsigned long>(ageMs));
          debugCli_.logText(line);
          snprintf(line,
                   sizeof(line),
                   "SEMANTIC_RESPONSE_REUSED node=%u.%02u object=%04X source=cache prevented_can=1",
                   static_cast<unsigned>(request.canId.subnet),
                   static_cast<unsigned>(request.canId.device),
                   static_cast<unsigned>(normalizedObjectKey(request)));
          debugCli_.logText(line);
        }
        ++counters_.allowlist_hits;
        return true;
      }
    }
  }

  if (entry != 0 && entry->inflight &&
      static_cast<int32_t>(nowMs - entry->inflightExpiresMs) < 0) {
    if (entry->waiterCount != 0xFFu) {
      ++entry->waiterCount;
    }
    ++counters_.stateful_inflight_joined_10s;
    ++counters_.stateful_can_reads_prevented_10s;
    if (groupWindow != 0) {
      ++groupWindow->inflightJoined;
      ++groupWindow->canReadsPrevented;
    }
    if (shouldLogLowRateCounter(counters_.stateful_inflight_joined_10s)) {
      char line[192] = {};
      snprintf(line,
               sizeof(line),
               "INFLIGHT_JOIN_ATTACH node=%u.%02u object=%04X source=stateful_cache waiters=%u",
               static_cast<unsigned>(request.canId.subnet),
               static_cast<unsigned>(request.canId.device),
               static_cast<unsigned>(normalizedObjectKey(request)),
               static_cast<unsigned>(entry->waiterCount));
      debugCli_.logText(line);
    }
    ++counters_.allowlist_hits;
    return true;
  }

  if (entry == 0) {
    entry = reserveStatefulStatusCacheEntry(request);
  }
  if (entry == 0) {
    return false;
  }
  entry->inflight = true;
  entry->inflightExpiresMs = nowMs + postBootstrapStatusCacheHardMaxAgeMs();
  entry->waiterCount = 0;
  ++counters_.cache_misses;
  ++counters_.stateful_cache_misses_10s;
  if (groupWindow != 0) {
    ++groupWindow->cacheMisses;
  }
  const char* failureReason = 0;
  if (scheduler_.enqueue(request, 0, false, nowMs, counters_, failureReason)) {
    debugCli_.logTcpCanTx(request);
    markTraceForwardedToCan(trace);
    noteObjectCanTxForwarded(request);
    if (groupWindow != 0) {
      ++groupWindow->forcedCanReads;
    }
    ++counters_.allowlist_hits;
  } else {
    entry->inflight = false;
    entry->inflightExpiresMs = 0;
    entry->waiterCount = 0;
    debugCli_.logTcpCanTxFail(request, failureReason);
    if (failureReason != 0 && strcmp(failureReason, "queue_full") == 0) {
      noteObjectQueueFull(request);
    }
  }
  return true;
}

uint8_t R70GatewayV2::releaseStatefulStatusCacheWaiters(const NormalizedRequest& reply, uint32_t nowMs) {
  if (!isPostBootstrapStatusCacheRequest(reply)) {
    return 0;
  }
  StatefulStatusCacheEntry* entry = findStatefulStatusCacheEntry(reply);
  if (entry == 0 || !entry->inflight) {
    return 0;
  }
  const uint8_t waiters = entry->waiterCount;
  entry->inflight = false;
  entry->inflightExpiresMs = 0;
  entry->waiterCount = 0;
  if (entry->valid && entry->lastUpdateMs <= nowMs) {
    const uint32_t ageMs = nowMs - entry->lastUpdateMs;
    if (ageMs > counters_.stateful_cache_age_max_ms) {
      counters_.stateful_cache_age_max_ms = ageMs;
    }
  }
  return waiters;
}

void R70GatewayV2::noteStatefulStatusCacheCanReply(const NormalizedRequest& reply, uint32_t nowMs) {
  if (!isPostBootstrapStatusCacheRequest(reply) || !postBootstrapStatusCacheActive(nowMs)) {
    return;
  }
  StatefulStatusCacheEntry* entry = findStatefulStatusCacheEntry(reply);
  if (entry == 0) {
    entry = reserveStatefulStatusCacheEntry(reply);
  }
  if (entry == 0) {
    return;
  }
  entry->valid = true;
  entry->replyFrame = reply.frame;
  entry->lastUpdateMs = nowMs;
}

void R70GatewayV2::resetStatefulStatusCacheSummaryWindow() {
  counters_.stateful_cache_hits_10s = 0;
  counters_.stateful_cache_misses_10s = 0;
  counters_.stateful_can_reads_prevented_10s = 0;
  counters_.stateful_inflight_joined_10s = 0;
  counters_.stateful_cache_age_max_ms = 0;
  resetStatefulStatusCacheGroupWindows();
}

uint8_t R70GatewayV2::activeGentleDeferredCount() const {
  uint8_t count = 0;
  for (uint8_t index = 0; index < kGentleStatusShapingCapacity; ++index) {
    if (gentleStatusShaping_[index].used && gentleStatusShaping_[index].pendingActive) {
      ++count;
    }
  }
  return count;
}

bool R70GatewayV2::gentleStatusShapingFeatureEnabled() const {
  return gentleStatusShapingRuntimeEnabled_ && config_.enableGentleStatusShaping;
}

bool R70GatewayV2::gentleStatusShapingEnabled() const {
  if (!gentleStatusShapingFeatureEnabled()) {
    return false;
  }
  const uint32_t nowMs = millis();
  if (!postBootstrapStatusCacheActive(nowMs)) {
    return false;
  }
  if (bootstrapProtectionActive(nowMs)) {
    return false;
  }
  if (counters_.bootstrap_nodes_incomplete != 0) {
    return false;
  }
  return true;
}

bool R70GatewayV2::gentleStatusShapingAutoDisableEligible(const char* reason) const {
  if (!gentleStatusShapingHasEverBeenActive_) {
    return false;
  }
  if (reason != 0 &&
      strcmp(reason, "r1_timeout_events") == 0 &&
      counters_.r1_timeout_events <= r1TimeoutEventsAtGentleStatusShapingEnable_) {
    return false;
  }
  return true;
}

uint32_t R70GatewayV2::currentBusUtilAvgPercent(uint32_t nowMs) const {
  if (!slowCanWindow_.initialized || config_.canBitrate == 0) {
    return 0;
  }
  const uint32_t durationMs =
      slowCanWindow_.startedMs != 0 && nowMs > slowCanWindow_.startedMs ? nowMs - slowCanWindow_.startedMs : 1u;
  return static_cast<uint32_t>((slowCanWindow_.estimatedBusBitsTotal * 100000ull) /
                               (static_cast<uint64_t>(config_.canBitrate) * durationMs));
}

bool R70GatewayV2::slow500GuardActive(uint32_t nowMs) const {
  return lastSlowOver500Ms_ != 0 &&
         static_cast<int32_t>(nowMs - (lastSlowOver500Ms_ + kSlow500GuardHoldMs)) < 0;
}

bool R70GatewayV2::postBootstrapStatusCacheWouldServe(const NormalizedRequest& request, uint32_t nowMs) const {
  if (!request.tcpWrappedCan || !isPostBootstrapStatusCacheRequest(request) || !postBootstrapStatusCacheActive(nowMs)) {
    return false;
  }
  const StatefulStatusCacheEntry* entry = findStatefulStatusCacheEntry(request);
  if (entry == 0) {
    return false;
  }
  if (entry->valid && entry->lastUpdateMs <= nowMs) {
    const uint32_t ageMs = nowMs - entry->lastUpdateMs;
    if (ageMs <= postBootstrapStatusCacheTtlMs(request) && ageMs <= postBootstrapStatusCacheHardMaxAgeMs()) {
      return true;
    }
  }
  return entry->inflight && static_cast<int32_t>(nowMs - entry->inflightExpiresMs) < 0;
}

R70GatewayV2::ActiveShapingCadenceState* R70GatewayV2::activeShapingCadenceState(
    const NormalizedRequest& request) {
  if (!request.valid || request.canId.device == 0) {
    return 0;
  }
  const uint8_t groupIndex = activeShapingCadenceGroupIndex(classifyShapingSimGroup(request));
  if (groupIndex >= kActiveShapingCadenceGroupCount) {
    return 0;
  }
  const uint16_t nodeIndex =
      static_cast<uint16_t>((static_cast<uint16_t>(request.canId.subnet) << 6) | (request.canId.device & 0x3Fu));
  if (nodeIndex >= GatewayState::kMaxAddressSlots) {
    return 0;
  }
  return &activeShapingCadence_[nodeIndex][groupIndex];
}

const R70GatewayV2::ActiveShapingCadenceState* R70GatewayV2::activeShapingCadenceState(
    const NormalizedRequest& request) const {
  return const_cast<R70GatewayV2*>(this)->activeShapingCadenceState(request);
}

bool R70GatewayV2::tryDeferGentleStatusShaping(const NormalizedRequest& request, uint32_t nowMs) {
  const ShapingSimGroup group = classifyShapingSimGroup(request);
  ShapingSimGroupWindow* bucket = shapingSimGroupWindow(group);
  if (!isGentleStatusShapingGroup(group)) {
    if (!slowCanWindow_.initialized) {
      resetSlowCanWindow(nowMs);
    }
    slowCanWindow_.shapingDisabledReasonCode = 6u;
    if (bucket != 0) {
      ++bucket->bypassIneligibleCount;
    }
    return false;
  }
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  ++slowCanWindow_.shapingCandidatesCount;
  if (!gentleStatusShapingFeatureEnabled() || !gentleStatusShapingEnabled()) {
    slowCanWindow_.shapingDisabledReasonCode = 1u;
    if (bucket != 0) {
      ++bucket->bypassDisabledCount;
    }
    return false;
  }
  if (request.objectAddress.hasRecord || request.objectAddress.hasSegment) {
    slowCanWindow_.shapingDisabledReasonCode = 6u;
    if (bucket != 0) {
      ++bucket->bypassIneligibleCount;
    }
    return false;
  }
  if (postBootstrapStatusCacheWouldServe(request, nowMs)) {
    slowCanWindow_.shapingDisabledReasonCode = 3u;
    if (bucket != 0) {
      ++bucket->bypassCacheReadyCount;
    }
    return false;
  }
  if (counters_.bootstrap_nodes_incomplete != 0) {
    ++slowCanWindow_.r1MissingDataEvents;
    slowCanWindow_.shapingDisabledReasonCode = 2u;
    if (bucket != 0) {
      ++bucket->bypassDisabledCount;
    }
    return false;
  }
  const uint16_t shapeWindowMs = gentleStatusShapingWindowMs(group);
  if (shapeWindowMs == 0) {
    slowCanWindow_.shapingDisabledReasonCode = 6u;
    if (bucket != 0) {
      ++bucket->bypassIneligibleCount;
    }
    return false;
  }
  ActiveShapingCadenceState* cadence = activeShapingCadenceState(request);
  if (cadence == 0) {
    slowCanWindow_.shapingDisabledReasonCode = 6u;
    if (bucket != 0) {
      ++bucket->bypassIneligibleCount;
    }
    return false;
  }

  GentleStatusShapingEntry* entry = findGentleStatusShapingEntry(request);
  if (entry != 0 && entry->pendingActive) {
    if (entry->lastCanTxMs == 0 ||
        (nowMs >= entry->lastCanTxMs &&
         static_cast<uint32_t>(nowMs - entry->lastCanTxMs) <= shapeWindowMs)) {
      ++slowCanWindow_.shapingShapedRequestsCount;
      ++slowCanWindow_.shapedDeferredCount;
      ++slowCanWindow_.shapingAttachedToInflightCount;
      ++slowCanWindow_.shapingSavedCanTx;
      noteActualShapingSavedCanTx(group, false);
      if (bucket != 0) {
        ++bucket->shapedRequests;
        ++bucket->attachedToRecentCount;
      }
      slowCanWindow_.shapingDisabledReasonCode = 4u;
      if (entry->waiterCount != 0xFFu) {
        ++entry->waiterCount;
      }
      if (entry->oldestWaiterAttachedMs == 0 || nowMs < entry->oldestWaiterAttachedMs) {
        entry->oldestWaiterAttachedMs = nowMs;
      }
      return true;
    }
    ++slowCanWindow_.shapedBypassDueAgeCount;
    slowCanWindow_.shapingDisabledReasonCode = 5u;
    return false;
  }

  if (entry != 0 && entry->lastReplyValid && entry->lastReplyMs != 0 && nowMs >= entry->lastReplyMs) {
    const uint32_t ageMs = nowMs - entry->lastReplyMs;
    if (ageMs <= kGentleStatusShapingFreshnessMaxMs) {
      uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
      uint8_t tcpFrameLength = 0;
      if (R70Frame::encodeWrappedCanFrame(entry->lastReplyFrame, tcpFrame, sizeof(tcpFrame), tcpFrameLength) &&
          tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
        ++slowCanWindow_.shapingShapedRequestsCount;
        ++slowCanWindow_.shapingReusedRecentResponseCount;
        ++slowCanWindow_.shapingSavedCanTx;
        noteActualShapingSavedCanTx(group, false);
        if (bucket != 0) {
          ++bucket->shapedRequests;
          ++bucket->reusedRecentResponseCount;
        }
        slowCanWindow_.shapingDisabledReasonCode = 4u;
        debugCli_.logTcpTxWrappedCan(entry->lastReplyFrame, tcpFrame, tcpFrameLength);
        state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
        noteStartupBlockerReplyForwarded(request, nowMs);
        cadence->lastReplyMs = nowMs;
        return true;
      }
    } else {
      ++slowCanWindow_.shapedBypassDueAgeCount;
      slowCanWindow_.shapingDisabledReasonCode = 5u;
      if (bucket != 0) {
        ++bucket->bypassDueToAgeCount;
        ++bucket->bypassCacheTooOldCount;
      }
      return false;
    }
  }

  const uint8_t groupIndex = activeShapingCadenceGroupIndex(group);
  const uint16_t phaseMs = static_cast<uint16_t>(((request.canId.subnet & 0x07u) * 11u +
                                                  (request.canId.device & 0x3Fu) * 5u +
                                                  groupIndex * 13u) % 23u);
  const uint32_t cadenceAnchorMs = cadence->lastCanTxMs != 0 ? cadence->lastCanTxMs : cadence->windowStartedMs;
  if (cadenceAnchorMs == 0 || nowMs < cadenceAnchorMs ||
      static_cast<uint32_t>(nowMs - cadenceAnchorMs) > kGentleStatusShapingFreshnessMaxMs) {
    if (cadenceAnchorMs != 0 &&
        nowMs >= cadenceAnchorMs &&
        static_cast<uint32_t>(nowMs - cadenceAnchorMs) > kGentleStatusShapingFreshnessMaxMs) {
      ++slowCanWindow_.shapedBypassDueAgeCount;
      slowCanWindow_.shapingDisabledReasonCode = 5u;
      if (bucket != 0) {
        ++bucket->bypassDueToAgeCount;
      }
    } else {
      slowCanWindow_.shapingDisabledReasonCode = 3u;
      if (bucket != 0) {
        ++bucket->bypassFirstRequestCount;
      }
    }
    ++slowCanWindow_.shapingAllowedImmediateCount;
    cadence->windowStartedMs = nowMs;
    cadence->pendingActive = true;
    return false;
  }

  const uint32_t cadenceDeadlineMs = cadenceAnchorMs + (shapeWindowMs > phaseMs ? (shapeWindowMs - phaseMs) : shapeWindowMs);
  if (static_cast<int32_t>(nowMs - cadenceDeadlineMs) >= 0) {
    ++slowCanWindow_.shapingAllowedImmediateCount;
    ++slowCanWindow_.shapedBypassDueAgeCount;
    slowCanWindow_.shapingDisabledReasonCode = 5u;
    if (bucket != 0) {
      ++bucket->bypassDueToAgeCount;
    }
    cadence->windowStartedMs = nowMs;
    cadence->pendingActive = true;
    return false;
  }

  if (entry == 0) {
    entry = reserveGentleStatusShapingEntry(request);
  }
  if (entry == 0) {
    slowCanWindow_.shapingDisabledReasonCode = 3u;
    if (bucket != 0) {
      ++bucket->bypassDisabledCount;
    }
    return false;
  }
  entry->pendingActive = true;
  entry->lastReplyValid = false;
  entry->lastCanTxMs = 0;
  entry->lastReplyMs = 0;
  entry->deferredStartedMs = nowMs;
  entry->deferredReleaseMs = cadenceDeadlineMs;
  entry->waiterCount = 0;
  entry->oldestWaiterAttachedMs = nowMs;
  ++slowCanWindow_.shapingShapedRequestsCount;
  ++slowCanWindow_.shapedDeferredCount;
  slowCanWindow_.shapingDisabledReasonCode = 4u;
  if (bucket != 0) {
    ++bucket->shapedRequests;
    ++bucket->deferredRequests;
  }
  cadence->pendingActive = true;
  return true;
}

void R70GatewayV2::processGentleStatusShaping(uint32_t nowMs) {
  if (!gentleStatusShapingFeatureEnabled()) {
    clearGentleStatusShapingEntries();
    return;
  }
  auto noteShapeAddedWait = [&](const GentleStatusShapingEntry& shapedEntry, uint32_t releasedMs) {
    if (shapedEntry.deferredStartedMs == 0 || releasedMs < shapedEntry.deferredStartedMs) {
      return;
    }
    const uint32_t addedWaitMs = releasedMs - shapedEntry.deferredStartedMs;
    slowCanWindow_.shapingAddedWaitSumMs += addedWaitMs;
    ++slowCanWindow_.shapingAddedWaitCount;
    if (addedWaitMs > slowCanWindow_.shapingAddedWaitMaxMs) {
      slowCanWindow_.shapingAddedWaitMaxMs =
          static_cast<uint16_t>(addedWaitMs > 0xFFFFu ? 0xFFFFu : addedWaitMs);
    }
    ShapingSimGroupWindow* bucket = shapingSimGroupWindow(shapedEntry.group);
    if (bucket != 0) {
      ++bucket->shapedDispatchCount;
      bucket->shapedDispatchAddedWaitSumMs += addedWaitMs;
      if (addedWaitMs > bucket->shapedDispatchAddedWaitMaxMs) {
        bucket->shapedDispatchAddedWaitMaxMs =
            static_cast<uint16_t>(addedWaitMs > 0xFFFFu ? 0xFFFFu : addedWaitMs);
      }
    }
  };
  auto noteShapeReplyWait = [&](GentleStatusShapingEntry& shapedEntry, uint32_t releasedMs) {
    if (shapedEntry.oldestWaiterAttachedMs == 0 || releasedMs < shapedEntry.oldestWaiterAttachedMs) {
      return;
    }
    const uint32_t waitMs = releasedMs - shapedEntry.oldestWaiterAttachedMs;
    ++slowCanWindow_.shapingReleaseSentCount;
    slowCanWindow_.shapingExtraWaitSumMs += waitMs;
    ++slowCanWindow_.shapingExtraWaitCount;
    if (waitMs > slowCanWindow_.shapingExtraWaitMaxMs) {
      slowCanWindow_.shapingExtraWaitMaxMs =
          static_cast<uint16_t>(waitMs > 0xFFFFu ? 0xFFFFu : waitMs);
    }
  };
  auto sendRecentReplyCopies = [&](const GentleStatusShapingEntry& shapedEntry,
                                   const NormalizedRequest& request,
                                   const Frame& replyFrame,
                                   bool statefulCacheHit) {
    uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
    uint8_t tcpFrameLength = 0;
    if (!R70Frame::encodeWrappedCanFrame(replyFrame, tcpFrame, sizeof(tcpFrame), tcpFrameLength)) {
      return false;
    }
    const uint8_t copies = static_cast<uint8_t>(1u + shapedEntry.waiterCount);
    bool forwarded = false;
    for (uint8_t copyIndex = 0; copyIndex < copies; ++copyIndex) {
      if (!tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
        continue;
      }
      debugCli_.logTcpTxWrappedCan(replyFrame, tcpFrame, tcpFrameLength);
      state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
      noteSemanticFamilyUpdated(request, nowMs, 3u, true);
      noteStartupBlockerReplyForwarded(request, nowMs);
      forwarded = true;
    }
    if (!forwarded) {
      return false;
    }
    if (statefulCacheHit) {
      ++counters_.cache_hits;
      ++counters_.cache_replies_sent;
      ++counters_.stateful_cache_hits_10s;
      ++counters_.stateful_can_reads_prevented_10s;
      if (StatefulStatusCacheGroupWindow* groupWindow =
              statefulStatusCacheGroupWindow(classifyStatefulStatusCacheGroup(request))) {
        ++groupWindow->cacheHits;
        ++groupWindow->canReadsPrevented;
        StatefulStatusCacheEntry* cacheEntry = findStatefulStatusCacheEntry(request);
        if (cacheEntry != 0 && cacheEntry->lastUpdateMs <= nowMs) {
          const uint32_t ageMs = nowMs - cacheEntry->lastUpdateMs;
          if (ageMs > groupWindow->cacheAgeMaxMs) {
            groupWindow->cacheAgeMaxMs = static_cast<uint16_t>(ageMs);
          }
          if (ageMs > counters_.stateful_cache_age_max_ms) {
            counters_.stateful_cache_age_max_ms = ageMs;
          }
        }
      }
      noteCacheHitByField(request.fieldId, counters_);
      noteObjectCacheReply(request, nowMs);
      ++counters_.allowlist_hits;
    }
    return true;
  };
  for (uint8_t index = 0; index < kGentleStatusShapingCapacity; ++index) {
    GentleStatusShapingEntry& entry = gentleStatusShaping_[index];
    if (!entry.used) {
      continue;
    }
    const uint16_t maxWaitMs = gentleStatusShapingMaxWaitMs(entry.group);
    if (entry.pendingActive &&
        entry.lastCanTxMs == 0 &&
        entry.deferredReleaseMs != 0 &&
        static_cast<int32_t>(nowMs - entry.deferredReleaseMs) >= 0) {
      NormalizedRequest request = {};
      request.valid = true;
      request.tcpWrappedCan = true;
      request.admin.valid = true;
      request.admin.functionCode = FunctionCode::ReadObject;
      request.admin.telegramType = entry.telegramType;
      request.canId.subnet = entry.subnet;
      request.canId.device = entry.device;
      request.objectAddress.channelNumber = entry.channel;
      request.objectAddress.hasRecord = entry.hasRecord;
      request.objectAddress.recordNumber = entry.recordNumber;
      request.objectAddress.hasSegment = entry.hasSegment;
      request.objectAddress.segmentNumber = entry.segmentNumber;
      request.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
      request.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
      request.fieldId = entry.fieldId;
      request.frame.length = entry.length;
      request.frame.canId =
          protocol::makeCanId(protocol::kObjectClassPrimaryMaster, Address{entry.subnet, entry.device});
      for (uint8_t byteIndex = 0; byteIndex < entry.length && byteIndex < sizeof(entry.data); ++byteIndex) {
        request.frame.data[byteIndex] = entry.data[byteIndex];
      }
      ShapingSimGroupWindow* bucket = shapingSimGroupWindow(entry.group);
      bool cacheAvailable = false;
      bool cacheTooOld = false;
      if (entry.lastReplyValid && entry.lastReplyMs != 0 && nowMs >= entry.lastReplyMs) {
        const uint32_t ageMs = nowMs - entry.lastReplyMs;
        cacheAvailable = true;
        if (ageMs <= kGentleStatusShapingFreshnessMaxMs &&
            sendRecentReplyCopies(entry, request, entry.lastReplyFrame, false)) {
          ++slowCanWindow_.shapingReusedRecentResponseCount;
          ++slowCanWindow_.shapingSavedCanTx;
          noteActualShapingSavedCanTx(entry.group, false);
          if (bucket != 0) {
            ++bucket->reusedRecentResponseCount;
          }
          noteShapeAddedWait(entry, nowMs);
          noteShapeReplyWait(entry, nowMs);
          if (ActiveShapingCadenceState* cadence = activeShapingCadenceState(request)) {
            cadence->windowStartedMs = nowMs;
            cadence->lastReplyMs = nowMs;
            cadence->pendingActive = false;
          }
          entry = {};
          continue;
        }
        if (ageMs > kGentleStatusShapingFreshnessMaxMs) {
          cacheTooOld = true;
        }
      }
      StatefulStatusCacheEntry* cacheEntry =
          postBootstrapStatusCacheActive(nowMs) ? findStatefulStatusCacheEntry(request) : 0;
      if (cacheEntry != 0 && cacheEntry->valid && cacheEntry->lastUpdateMs <= nowMs) {
        const uint32_t ageMs = nowMs - cacheEntry->lastUpdateMs;
        cacheAvailable = true;
        if (ageMs <= postBootstrapStatusCacheTtlMs(request) && ageMs <= postBootstrapStatusCacheHardMaxAgeMs()) {
          if (sendRecentReplyCopies(entry, request, cacheEntry->replyFrame, true)) {
            ++slowCanWindow_.shapingReusedRecentResponseCount;
            ++slowCanWindow_.shapingSavedCanTx;
            noteActualShapingSavedCanTx(entry.group, false);
            if (bucket != 0) {
              ++bucket->reusedRecentResponseCount;
            }
            noteShapeAddedWait(entry, nowMs);
            noteShapeReplyWait(entry, nowMs);
            if (ActiveShapingCadenceState* cadence = activeShapingCadenceState(request)) {
              cadence->windowStartedMs = nowMs;
              cadence->lastReplyMs = nowMs;
              cadence->pendingActive = false;
            }
            entry = {};
            continue;
          }
        } else {
          cacheTooOld = true;
        }
      }
      if (bucket != 0) {
        if (!cacheAvailable) {
          ++bucket->bypassNoCacheCount;
        } else if (cacheTooOld) {
          ++bucket->bypassCacheTooOldCount;
        }
      }
      bool joinedInflight = false;
      if (cacheEntry != 0 && cacheEntry->inflight &&
          static_cast<int32_t>(nowMs - cacheEntry->inflightExpiresMs) < 0) {
        uint16_t totalWaiters = static_cast<uint16_t>(cacheEntry->waiterCount) +
                                static_cast<uint16_t>(1u + entry.waiterCount);
        cacheEntry->waiterCount = static_cast<uint8_t>(totalWaiters > 0xFFu ? 0xFFu : totalWaiters);
        ++counters_.stateful_inflight_joined_10s;
        ++counters_.stateful_can_reads_prevented_10s;
        ++counters_.allowlist_hits;
        ++slowCanWindow_.shapingAttachedToInflightCount;
        ++slowCanWindow_.shapingSavedCanTx;
        noteActualShapingSavedCanTx(entry.group, false);
        if (StatefulStatusCacheGroupWindow* groupWindow =
                statefulStatusCacheGroupWindow(classifyStatefulStatusCacheGroup(request))) {
          ++groupWindow->inflightJoined;
          ++groupWindow->canReadsPrevented;
        }
        if (ShapingSimGroupWindow* bucket = shapingSimGroupWindow(entry.group)) {
          ++bucket->attachedToRecentCount;
        }
        noteShapeAddedWait(entry, nowMs);
        if (ActiveShapingCadenceState* cadence = activeShapingCadenceState(request)) {
          cadence->windowStartedMs = nowMs;
          cadence->pendingActive = true;
        }
        joinedInflight = true;
      } else if (fanoutAttachFeatureEnabled()) {
        SoftInflightCollapseEntry* inflightEntry = findSoftInflightCollapseEntry(request);
        if (inflightEntry != 0 &&
            static_cast<int32_t>(nowMs - inflightEntry->expiresMs) < 0) {
          if (inflightEntry->waiterCount != 0xFFu) {
            uint16_t totalWaiters = static_cast<uint16_t>(inflightEntry->waiterCount) +
                                    static_cast<uint16_t>(1u + entry.waiterCount);
            inflightEntry->waiterCount = static_cast<uint8_t>(totalWaiters > 0xFFu ? 0xFFu : totalWaiters);
          }
          const uint16_t maxWaitMs = gentleStatusShapingMaxWaitMs(entry.group);
          inflightEntry->expiresMs = nowMs + (maxWaitMs != 0 ? maxWaitMs : kGentleStatusShapingFreshnessMaxMs);
          ++slowCanWindow_.shapingAttachedToInflightCount;
          ++slowCanWindow_.shapingSavedCanTx;
          noteActualShapingSavedCanTx(entry.group, false);
          if (bucket != 0) {
            ++bucket->attachedToRecentCount;
          }
          if (inflightEntry->waiterCount > slowCanWindow_.inflightWaitersMax) {
            slowCanWindow_.inflightWaitersMax = inflightEntry->waiterCount;
          }
          noteShapeAddedWait(entry, nowMs);
          if (ActiveShapingCadenceState* cadence = activeShapingCadenceState(request)) {
            cadence->windowStartedMs = nowMs;
            cadence->pendingActive = true;
          }
          joinedInflight = true;
        }
      }
      if (joinedInflight) {
        entry = {};
        continue;
      }
      if (bucket != 0) {
        ++bucket->bypassNoInflightCount;
      }
      if (bucket != 0) {
        ++bucket->bypassReleaseForcedCount;
      }
      const char* failureReason = 0;
      if (scheduler_.enqueue(request, 0, false, nowMs, counters_, failureReason)) {
        entry.lastCanTxMs = nowMs;
        ++slowCanWindow_.shapedSentCount;
        noteShapeAddedWait(entry, nowMs);
      } else {
        debugCli_.logTcpCanTxFail(request, failureReason);
        entry.deferredReleaseMs = nowMs + 10u;
      }
    } else if (entry.pendingActive &&
               entry.lastCanTxMs != 0 &&
               maxWaitMs != 0 &&
               static_cast<int32_t>(nowMs - (entry.lastCanTxMs + maxWaitMs)) >= 0) {
      if (entry.waiterCount == 0) {
        if (static_cast<int32_t>(nowMs - (entry.lastCanTxMs + postBootstrapStatusCacheHardMaxAgeMs())) >= 0) {
          entry.pendingActive = false;
          entry.oldestWaiterAttachedMs = 0;
        }
      } else {
        NormalizedRequest request = {};
        request.valid = true;
        request.tcpWrappedCan = true;
        request.admin.valid = true;
        request.admin.functionCode = FunctionCode::ReadObject;
        request.admin.telegramType = entry.telegramType;
        request.canId.subnet = entry.subnet;
        request.canId.device = entry.device;
        request.objectAddress.channelNumber = entry.channel;
        request.objectAddress.hasRecord = entry.hasRecord;
        request.objectAddress.recordNumber = entry.recordNumber;
        request.objectAddress.hasSegment = entry.hasSegment;
        request.objectAddress.segmentNumber = entry.segmentNumber;
        request.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
        request.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
        request.fieldId = entry.fieldId;
        request.frame.length = entry.length;
        request.frame.canId =
            protocol::makeCanId(protocol::kObjectClassPrimaryMaster, Address{entry.subnet, entry.device});
        for (uint8_t byteIndex = 0; byteIndex < entry.length && byteIndex < sizeof(entry.data); ++byteIndex) {
          request.frame.data[byteIndex] = entry.data[byteIndex];
        }
        const char* failureReason = 0;
        if (scheduler_.enqueue(request, 0, false, nowMs, counters_, failureReason)) {
          ++slowCanWindow_.shapedBypassDueAgeCount;
          entry.lastCanTxMs = nowMs;
        } else {
          debugCli_.logTcpCanTxFail(request, failureReason);
        }
      }
    }
    if (entry.lastReplyValid &&
        entry.lastReplyMs != 0 &&
        static_cast<int32_t>(nowMs - (entry.lastReplyMs + kGentleStatusShapingFreshnessMaxMs)) >= 0) {
      entry.lastReplyValid = false;
    }
  }
}

void R70GatewayV2::noteGentleStatusShapingCanTx(const NormalizedRequest& request, uint32_t nowMs) {
  if (!gentleStatusShapingFeatureEnabled()) {
    return;
  }
  const ShapingSimGroup group = classifyShapingSimGroup(request);
  if (!isGentleStatusShapingGroup(group)) {
    return;
  }
  GentleStatusShapingEntry* entry = findGentleStatusShapingEntry(request);
  if (entry == 0) {
    entry = reserveGentleStatusShapingEntry(request);
  }
  if (entry == 0) {
    return;
  }
  if (ActiveShapingCadenceState* cadence = activeShapingCadenceState(request)) {
    cadence->windowStartedMs = nowMs;
    cadence->lastCanTxMs = nowMs;
    cadence->pendingActive = true;
  }
  const bool preserveWaiters = entry->pendingActive && entry->lastCanTxMs == 0;
  entry->pendingActive = true;
  entry->lastReplyValid = false;
  entry->lastCanTxMs = nowMs;
  entry->deferredStartedMs = 0;
  entry->deferredReleaseMs = 0;
  if (!preserveWaiters) {
    entry->waiterCount = 0;
    entry->oldestWaiterAttachedMs = 0;
  }
}

uint8_t R70GatewayV2::releaseGentleStatusShapingWaiters(const NormalizedRequest& reply, uint32_t nowMs) {
  for (uint8_t index = 0; index < kGentleStatusShapingCapacity; ++index) {
    GentleStatusShapingEntry& entry = gentleStatusShaping_[index];
    if (!entry.used || !entry.pendingActive ||
        entry.subnet != reply.canId.subnet ||
        entry.device != reply.canId.device ||
        entry.channel != reply.objectAddress.channelNumber ||
        entry.objectKey != normalizedObjectKey(reply) ||
        entry.fieldId != reply.fieldId ||
        entry.telegramType != reply.admin.telegramType ||
        entry.hasRecord != reply.objectAddress.hasRecord ||
        entry.hasSegment != reply.objectAddress.hasSegment) {
      continue;
    }
    if (entry.hasRecord && entry.recordNumber != reply.objectAddress.recordNumber) {
      continue;
    }
    if (entry.hasSegment && entry.segmentNumber != reply.objectAddress.segmentNumber) {
      continue;
    }
    const uint8_t waiters = entry.waiterCount;
    entry.lastReplyValid = true;
    entry.lastReplyFrame = reply.frame;
    entry.lastReplyMs = nowMs;
    if (entry.oldestWaiterAttachedMs != 0 && nowMs >= entry.oldestWaiterAttachedMs) {
      const uint32_t waitMs = nowMs - entry.oldestWaiterAttachedMs;
      ++slowCanWindow_.shapingReleaseSentCount;
      slowCanWindow_.shapingExtraWaitSumMs += waitMs;
      ++slowCanWindow_.shapingExtraWaitCount;
      if (waitMs > slowCanWindow_.shapingExtraWaitMaxMs) {
        slowCanWindow_.shapingExtraWaitMaxMs =
            static_cast<uint16_t>(waitMs > 0xFFFFu ? 0xFFFFu : waitMs);
      }
    }
    entry.pendingActive = false;
    entry.waiterCount = 0;
    entry.oldestWaiterAttachedMs = 0;
    if (ActiveShapingCadenceState* cadence = activeShapingCadenceState(reply)) {
      cadence->windowStartedMs = nowMs;
      cadence->lastReplyMs = nowMs;
      cadence->pendingActive = false;
    }
    return waiters;
  }
  return 0;
}

uint32_t R70GatewayV2::softInflightCollapseTimeoutMs() const {
  if (config_.softInflightCollapseTimeoutMs == 0) {
    return 2000u;
  }
  return config_.softInflightCollapseTimeoutMs < 500u ? 500u : config_.softInflightCollapseTimeoutMs;
}

R70GatewayV2::SoftInflightCollapseEntry* R70GatewayV2::findSoftInflightCollapseEntry(const NormalizedRequest& request) {
  for (uint8_t index = 0; index < kSoftInflightCollapseCapacity; ++index) {
    SoftInflightCollapseEntry& entry = softInflightCollapse_[index];
    if (!entry.used) {
      continue;
    }
    if (entry.subnet != request.canId.subnet ||
        entry.device != request.canId.device ||
        entry.channel != request.objectAddress.channelNumber ||
        entry.objectKey != normalizedObjectKey(request) ||
        entry.fieldId != request.fieldId ||
        entry.telegramType != request.admin.telegramType ||
        entry.hasRecord != request.objectAddress.hasRecord ||
        entry.hasSegment != request.objectAddress.hasSegment ||
        entry.length != request.frame.length) {
      continue;
    }
    if (entry.hasRecord && entry.recordNumber != request.objectAddress.recordNumber) {
      continue;
    }
    if (entry.hasSegment && entry.segmentNumber != request.objectAddress.segmentNumber) {
      continue;
    }
    bool exactBytes = true;
    for (uint8_t byteIndex = 0; byteIndex < entry.length && byteIndex < sizeof(entry.data); ++byteIndex) {
      if (entry.data[byteIndex] != request.frame.data[byteIndex]) {
        exactBytes = false;
        break;
      }
    }
    if (exactBytes) {
      return &entry;
    }
  }
  return 0;
}

const R70GatewayV2::SoftInflightCollapseEntry* R70GatewayV2::findSoftInflightCollapseEntry(
    const NormalizedRequest& request) const {
  return const_cast<R70GatewayV2*>(this)->findSoftInflightCollapseEntry(request);
}

void R70GatewayV2::expireSoftInflightCollapse(uint32_t nowMs) {
  for (uint8_t index = 0; index < kSoftInflightCollapseCapacity; ++index) {
    SoftInflightCollapseEntry& entry = softInflightCollapse_[index];
    if (!entry.used || static_cast<int32_t>(nowMs - entry.expiresMs) < 0) {
      continue;
    }
    if (!slowCanWindow_.initialized) {
      resetSlowCanWindow(nowMs);
    }
    if (entry.waiterCount == 0) {
      entry = {};
      continue;
    }
    NormalizedRequest request = {};
    request.valid = true;
    request.tcpWrappedCan = true;
    request.admin.valid = true;
    request.admin.functionCode = FunctionCode::ReadObject;
    request.admin.telegramType = entry.telegramType;
    request.canId.subnet = entry.subnet;
    request.canId.device = entry.device;
    request.objectAddress.channelNumber = entry.channel;
    request.objectAddress.hasRecord = entry.hasRecord;
    request.objectAddress.recordNumber = entry.recordNumber;
    request.objectAddress.hasSegment = entry.hasSegment;
    request.objectAddress.segmentNumber = entry.segmentNumber;
    request.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
    request.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
    request.fieldId = entry.fieldId;
    request.frame.length = entry.length;
    request.frame.canId =
        protocol::makeCanId(protocol::kObjectClassPrimaryMaster, Address{entry.subnet, entry.device});
    for (uint8_t byteIndex = 0; byteIndex < entry.length && byteIndex < sizeof(entry.data); ++byteIndex) {
      request.frame.data[byteIndex] = entry.data[byteIndex];
    }
    const uint16_t maxWaitMs = gentleStatusShapingMaxWaitMs(entry.group);
    const char* failureReason = 0;
    if (scheduler_.enqueue(request, 0, false, nowMs, counters_, failureReason)) {
      ++slowCanWindow_.inflightTimeoutReleaseCount;
      entry.startedMs = nowMs;
      entry.expiresMs = nowMs + (maxWaitMs != 0 ? maxWaitMs : kGentleStatusShapingFreshnessMaxMs);
    } else {
      debugCli_.logTcpCanTxFail(request, failureReason);
      entry.expiresMs = nowMs + (maxWaitMs != 0 ? maxWaitMs : kGentleStatusShapingFreshnessMaxMs);
    }
  }
}

bool R70GatewayV2::tryAttachSoftInflightCollapse(const NormalizedRequest& request, uint32_t nowMs) {
  if (!fanoutAttachFeatureEnabled() || !gentleStatusShapingEnabled()) {
    return false;
  }
  if (request.objectAddress.hasRecord ||
      request.objectAddress.hasSegment ||
      isNameSegmentReadRequest(request) ||
      isFirmwareSegmentReadRequest(request)) {
    return false;
  }
  const ShapingSimGroup group = classifyShapingSimGroup(request);
  if (!isGentleStatusShapingGroup(group)) {
    return false;
  }
  SoftInflightCollapseEntry* entry = findSoftInflightCollapseEntry(request);
  if (entry == 0) {
    return false;
  }
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  if (entry->waiterCount != 0xFFu) {
    ++entry->waiterCount;
  }
  const uint16_t maxWaitMs = gentleStatusShapingMaxWaitMs(group);
  entry->expiresMs = nowMs + (maxWaitMs != 0 ? maxWaitMs : kGentleStatusShapingFreshnessMaxMs);
  ++slowCanWindow_.inflightCollapsedCount;
  ++slowCanWindow_.collapseSavedCanTx;
  noteActualShapingSavedCanTx(group, true);
  if (entry->waiterCount > slowCanWindow_.inflightWaitersMax) {
    slowCanWindow_.inflightWaitersMax = entry->waiterCount;
  }
  return true;
}

void R70GatewayV2::noteSoftInflightPrimary(const NormalizedRequest& request, uint32_t nowMs) {
  if (!fanoutAttachFeatureEnabled() || !gentleStatusShapingEnabled()) {
    return;
  }
  if (request.objectAddress.hasRecord ||
      request.objectAddress.hasSegment ||
      isNameSegmentReadRequest(request) ||
      isFirmwareSegmentReadRequest(request)) {
    return;
  }
  const ShapingSimGroup group = classifyShapingSimGroup(request);
  if (!isGentleStatusShapingGroup(group)) {
    return;
  }
  SoftInflightCollapseEntry* existing = findSoftInflightCollapseEntry(request);
  const uint16_t maxWaitMs = gentleStatusShapingMaxWaitMs(group);
  if (existing != 0) {
    existing->expiresMs = nowMs + (maxWaitMs != 0 ? maxWaitMs : kGentleStatusShapingFreshnessMaxMs);
    return;
  }
  SoftInflightCollapseEntry* empty = 0;
  for (uint8_t index = 0; index < kSoftInflightCollapseCapacity; ++index) {
    if (!softInflightCollapse_[index].used) {
      empty = &softInflightCollapse_[index];
      break;
    }
  }
  if (empty == 0) {
    empty = &softInflightCollapse_[0];
  }
  *empty = {};
  empty->used = true;
  empty->subnet = request.canId.subnet;
  empty->device = request.canId.device;
  empty->channel = request.objectAddress.channelNumber;
  empty->objectKey = normalizedObjectKey(request);
  empty->fieldId = request.fieldId;
  empty->telegramType = request.admin.telegramType;
  empty->hasRecord = request.objectAddress.hasRecord;
  empty->recordNumber = request.objectAddress.recordNumber;
  empty->hasSegment = request.objectAddress.hasSegment;
  empty->segmentNumber = request.objectAddress.segmentNumber;
  empty->length = request.frame.length;
  for (uint8_t index = 0; index < request.frame.length && index < sizeof(empty->data); ++index) {
    empty->data[index] = request.frame.data[index];
  }
  empty->startedMs = nowMs;
  empty->expiresMs = nowMs + (maxWaitMs != 0 ? maxWaitMs : kGentleStatusShapingFreshnessMaxMs);
  empty->group = group;
}

uint8_t R70GatewayV2::releaseSoftInflightCollapseWaiters(const NormalizedRequest& reply, uint32_t nowMs) {
  for (uint8_t index = 0; index < kSoftInflightCollapseCapacity; ++index) {
    SoftInflightCollapseEntry& entry = softInflightCollapse_[index];
    if (!entry.used ||
        entry.subnet != reply.canId.subnet ||
        entry.device != reply.canId.device ||
        entry.channel != reply.objectAddress.channelNumber ||
        entry.objectKey != normalizedObjectKey(reply) ||
        entry.fieldId != reply.fieldId ||
        entry.telegramType != reply.admin.telegramType ||
        entry.hasRecord != reply.objectAddress.hasRecord ||
        entry.hasSegment != reply.objectAddress.hasSegment) {
      continue;
    }
    if (entry.hasRecord && entry.recordNumber != reply.objectAddress.recordNumber) {
      continue;
    }
    if (entry.hasSegment && entry.segmentNumber != reply.objectAddress.segmentNumber) {
      continue;
    }
    const uint8_t waiters = entry.waiterCount;
    if (waiters != 0 && nowMs >= entry.startedMs) {
      const uint32_t waitMs = nowMs - entry.startedMs;
      slowCanWindow_.shapingExtraWaitSumMs += waitMs;
      ++slowCanWindow_.shapingExtraWaitCount;
      if (waitMs > slowCanWindow_.shapingExtraWaitMaxMs) {
        slowCanWindow_.shapingExtraWaitMaxMs =
            waitMs > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(waitMs);
      }
    }
    entry = {};
    return waiters;
  }
  return 0;
}

void R70GatewayV2::noteCanBusFrame(uint32_t nowMs, uint8_t subnet, bool txFrame, uint8_t dlc) {
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  const uint32_t elapsedMs = nowMs >= slowCanWindow_.startedMs ? nowMs - slowCanWindow_.startedMs : 0;
  uint8_t bucket100ms = static_cast<uint8_t>(elapsedMs / 100u);
  if (bucket100ms >= 100u) {
    bucket100ms = 99u;
  }
  uint8_t bucket1s = static_cast<uint8_t>(elapsedMs / 1000u);
  if (bucket1s >= 10u) {
    bucket1s = 9u;
  }

  const uint16_t estimatedBits = estimateCanFrameBits(dlc);
  slowCanWindow_.estimatedBusBitsTotal += estimatedBits;
  slowCanWindow_.estimatedBusBitsPer100msBins[bucket100ms] += estimatedBits;
  if (slowCanWindow_.estimatedBusBitsPer100msBins[bucket100ms] > slowCanWindow_.peakBusBitsPer100ms) {
    slowCanWindow_.peakBusBitsPer100ms = slowCanWindow_.estimatedBusBitsPer100msBins[bucket100ms];
  }

  if (txFrame) {
    ++slowCanWindow_.tx10sCount;
    ++slowCanWindow_.txFramesPer100msBins[bucket100ms];
    ++slowCanWindow_.txFramesPer1sBins[bucket1s];
    if (subnet < DebugCounters::kSubnetCount) {
      ++slowCanWindow_.txPerSubnet[subnet];
    }
    if (slowCanWindow_.txFramesPer100msBins[bucket100ms] > slowCanWindow_.peakTxFramesPer100ms) {
      slowCanWindow_.peakTxFramesPer100ms = slowCanWindow_.txFramesPer100msBins[bucket100ms];
    }
    if (slowCanWindow_.txFramesPer1sBins[bucket1s] > slowCanWindow_.peakTxFramesPer1s) {
      slowCanWindow_.peakTxFramesPer1s = slowCanWindow_.txFramesPer1sBins[bucket1s];
    }
    return;
  }

  ++slowCanWindow_.rx10sCount;
  ++slowCanWindow_.rxFramesPer100msBins[bucket100ms];
  ++slowCanWindow_.rxFramesPer1sBins[bucket1s];
  if (subnet < DebugCounters::kSubnetCount) {
    ++slowCanWindow_.rxPerSubnet[subnet];
  }
  if (slowCanWindow_.rxFramesPer100msBins[bucket100ms] > slowCanWindow_.peakRxFramesPer100ms) {
    slowCanWindow_.peakRxFramesPer100ms = slowCanWindow_.rxFramesPer100msBins[bucket100ms];
  }
  if (slowCanWindow_.rxFramesPer1sBins[bucket1s] > slowCanWindow_.peakRxFramesPer1s) {
    slowCanWindow_.peakRxFramesPer1s = slowCanWindow_.rxFramesPer1sBins[bucket1s];
  }
}

void R70GatewayV2::noteSchedulerCanTx(const NormalizedRequest& request,
                                      QueueClass queueClass,
                                      uint32_t enqueuedMs,
                                      uint32_t nowMs) {
  (void)enqueuedMs;
  if (queueClass == QueueClass::Maintenance) {
    AmpState* amp = state_.amp(request.canId.subnet, request.canId.device, false);
    if (amp != 0) {
      for (uint8_t i = 0; i < AmpState::kLiveRefreshCount; ++i) {
        AmpState::LiveRefresh& track = amp->liveRefresh[i];
        if (track.pending && track.request.canId == request.frame.canId &&
            track.request.length == request.frame.length &&
            memcmp(track.request.data, request.frame.data, request.frame.length) == 0) track.sent = true;
      }
    }
    for (uint8_t i = 0; i < kExtraLiveRefreshCount; ++i) {
      AmpState::LiveRefresh& track = extraLiveRefresh_[i].track;
      if (extraLiveRefresh_[i].used && track.pending &&
          track.request.canId == request.frame.canId && track.request.length == request.frame.length &&
          memcmp(track.request.data, request.frame.data, request.frame.length) == 0) track.sent = true;
    }
  }
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(nowMs);
  }
  noteCanBusFrame(nowMs, request.canId.subnet, true, request.frame.length);
  if (queueClass == QueueClass::Ident) {
    lastIdentTxMs_ = nowMs;
    ++slowCanWindow_.identTxCount;
    startPresenceRound(request, nowMs);
    return;
  }
  noteExpensiveObjectCanTx(request);
  noteShapingSimCanTx(request, nowMs);
  noteGentleStatusShapingCanTx(request, nowMs);

  RequestTrace* trace = findMatchingPendingTrace(request);
  if (trace != 0) {
    trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagCanTxSeen);
    trace->canTxTimestampMs = nowMs;
    if (nowMs >= trace->timestampMs) {
      trace->queueWaitMs = static_cast<uint16_t>(nowMs - trace->timestampMs);
    }
    if (lastIdentTxMs_ != 0 &&
        static_cast<int32_t>(nowMs - (lastIdentTxMs_ + config_.identWindowMs + config_.identQuietHoldMs)) < 0) {
      trace->flags = static_cast<uint8_t>(trace->flags | kTraceFlagNearIdentBurst);
    }
  }

  if (!request.valid || !request.admin.valid || !isReadFunction(request.admin.functionCode) ||
      request.canId.device == 0) {
    return;
  }
  if (queueClass == QueueClass::NormalRead && request.canId.subnet < DebugCounters::kSubnetCount) {
    ++slowCanWindow_.normalReadTxPerSubnet[request.canId.subnet];
  }

  const uint16_t nodeKey =
      static_cast<uint16_t>(((static_cast<uint16_t>(request.canId.subnet) & 0x07u) << 6) |
                            (static_cast<uint16_t>(request.canId.device) & 0x3Fu));
  if (nodeKey == fairnessLastNodeKey_) {
    ++fairnessCurrentNodeStreak_;
  } else {
    fairnessLastNodeKey_ = nodeKey;
    fairnessCurrentNodeStreak_ = 1;
  }
  if (fairnessCurrentNodeStreak_ > slowCanWindow_.maxConsecutiveNodeRequests) {
    slowCanWindow_.maxConsecutiveNodeRequests = fairnessCurrentNodeStreak_;
  }
  if (nodeKey < GatewayState::kMaxAddressSlots) {
    const uint32_t lastNodeMs = lastNodeCanTxMs_[nodeKey];
    if (lastNodeMs != 0 && nowMs > lastNodeMs) {
      const uint32_t gapMs = nowMs - lastNodeMs;
      if (gapMs > slowCanWindow_.maxNodeStarvationGapMs) {
        slowCanWindow_.maxNodeStarvationGapMs = gapMs;
      }
    }
    lastNodeCanTxMs_[nodeKey] = nowMs;
  }

  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  if (objectKey == fairnessLastObjectKey_) {
    ++fairnessCurrentObjectStreak_;
  } else {
    fairnessLastObjectKey_ = objectKey;
    fairnessCurrentObjectStreak_ = 1;
  }
  if (fairnessCurrentObjectStreak_ > slowCanWindow_.maxConsecutiveObjectRequests) {
    slowCanWindow_.maxConsecutiveObjectRequests = fairnessCurrentObjectStreak_;
  }
  SlowObjectFairnessState* objectState = findOrCreateObjectFairnessState(objectKey);
  if (objectState != 0) {
    if (objectState->lastTxMs != 0 && nowMs > objectState->lastTxMs) {
      const uint32_t gapMs = nowMs - objectState->lastTxMs;
      if (gapMs > slowCanWindow_.maxObjectStarvationGapMs) {
        slowCanWindow_.maxObjectStarvationGapMs = gapMs;
      }
    }
    objectState->lastTxMs = nowMs;
  }
}

void R70GatewayV2::noteSlowCanResponse(const RequestTrace& trace) {
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(trace.replyTimestampMs != 0 ? trace.replyTimestampMs : trace.timestampMs);
  }
  if (trace.canRxTimestampMs == 0 || trace.canRxTimestampMs < trace.timestampMs) {
    return;
  }

  const uint32_t canLatencyMs = trace.canRxTimestampMs - trace.timestampMs;
  ExpensiveObjectWindow* expensiveBucket = expensiveObjectWindow(trace.expensiveGroup);
  SlowObjectBucket* objectBucket = findOrCreateSlowObjectBucket(trace.objectKey);
  if (objectBucket != 0) {
    if (canLatencyMs < 100u) {
      ++objectBucket->lt100Count;
    } else if (canLatencyMs < 250u) {
      ++objectBucket->bucket100To250Count;
    } else if (canLatencyMs < 500u) {
      ++objectBucket->bucket250To500Count;
    } else if (canLatencyMs <= 1000u) {
      ++objectBucket->bucket500To1000Count;
    } else {
      ++objectBucket->gt1000Count;
    }
  }
  if (canLatencyMs <= 500u) {
    return;
  }

  ++slowCanWindow_.over500Count;
  lastSlowOver500Ms_ = trace.replyTimestampMs != 0 ? trace.replyTimestampMs : trace.canRxTimestampMs;
  if (expensiveBucket != 0) {
    ++expensiveBucket->over500Count;
  }
  if ((trace.flags & kTraceFlagNearIdentBurst) != 0) {
    ++slowCanWindow_.identSlowOver500Count;
  }
  if (canLatencyMs > 1000u) {
    ++slowCanWindow_.over1000Count;
    if (expensiveBucket != 0) {
      ++expensiveBucket->over1000Count;
    }
    if ((trace.flags & kTraceFlagNearIdentBurst) != 0) {
      ++slowCanWindow_.identSlowOver1000Count;
    }
  }

  if (trace.queueWaitMs > slowCanWindow_.worstQueueWaitMs) {
    slowCanWindow_.worstQueueWaitMs = trace.queueWaitMs;
  }
  if (trace.canRoundtripMs > slowCanWindow_.worstCanRoundtripMs) {
    slowCanWindow_.worstCanRoundtripMs = trace.canRoundtripMs;
  }
  if (trace.postReplyMs > slowCanWindow_.worstPostReplyMs) {
    slowCanWindow_.worstPostReplyMs = trace.postReplyMs;
  }
  if (canLatencyMs > slowCanWindow_.worstLatencyMs) {
    slowCanWindow_.worstLatencyMs = static_cast<uint16_t>(canLatencyMs);
    slowCanWindow_.worstSubnet = trace.subnet;
    slowCanWindow_.worstDevice = trace.device;
    slowCanWindow_.worstChannel = trace.channel;
    slowCanWindow_.worstObjectKey = trace.objectKey;
    slowCanWindow_.worstCategory = trace.category;
  }

  if (objectBucket != 0) {
    ++objectBucket->over500Count;
    if (canLatencyMs > 1000u) {
      ++objectBucket->over1000Count;
    }
    if (canLatencyMs > objectBucket->maxLatencyMs) {
      objectBucket->maxLatencyMs = static_cast<uint16_t>(canLatencyMs);
    }
  }

  SlowNodeBucket* nodeBucket = findOrCreateSlowNodeBucket(trace.subnet, trace.device);
  if (nodeBucket != 0) {
    ++nodeBucket->over500Count;
    if (canLatencyMs > 1000u) {
      ++nodeBucket->over1000Count;
    }
    if (canLatencyMs > nodeBucket->maxLatencyMs) {
      nodeBucket->maxLatencyMs = static_cast<uint16_t>(canLatencyMs);
    }
  }

  if (trace.subnet < DebugCounters::kSubnetCount) {
    SlowSubnetBucket& subnetBucket = slowCanWindow_.subnetBuckets[trace.subnet];
    ++subnetBucket.over500Count;
    if (canLatencyMs > 1000u) {
      ++subnetBucket.over1000Count;
    }
    if (canLatencyMs > subnetBucket.maxLatencyMs) {
      subnetBucket.maxLatencyMs = static_cast<uint16_t>(canLatencyMs);
    }
  }
}

R70GatewayV2::ObjectStat* R70GatewayV2::findOrCreateObjectStat(uint16_t objectKey, FieldId fieldId) {
#if !R70_EXPERIMENTAL_DEBUG_CODE
  (void)objectKey;
  (void)fieldId;
  return 0;
#else
  ObjectStat* empty = 0;
  for (uint8_t index = 0; index < kObjectStatCapacity; ++index) {
    ObjectStat& stat = objectStats_[index];
    if (stat.used && stat.objectKey == objectKey) {
      if (stat.fieldId == FieldId::Unknown && fieldId != FieldId::Unknown) {
        stat.fieldId = fieldId;
      }
      return &stat;
    }
    if (!stat.used && empty == 0) {
      empty = &stat;
    }
  }
  if (empty == 0) {
    return &objectStats_[0];
  }
  empty->used = true;
  empty->objectKey = objectKey;
  empty->fieldId = fieldId;
  return empty;
#endif
}

void R70GatewayV2::logFocusedLiveMeter(const char* stage,
                                       const NormalizedRequest& request,
                                       const Frame& frame,
                                       uint32_t nowMs,
                                       uint32_t ageMs) const {
  if (!request.valid || request.canId.subnet != 2u || request.canId.device != 5u) return;
  switch (request.fieldId) {
    case FieldId::chstatus_isp_a:
    case FieldId::chstatus_isp_b:
    case FieldId::chstatus_input_voltage_a:
    case FieldId::chstatus_input_voltage_b:
    case FieldId::chstatus_input_voltage_peak_a:
    case FieldId::chstatus_input_voltage_peak_b:
    case FieldId::chstatus_gr_a:
    case FieldId::chstatus_gr_b:
    case FieldId::chstatus_ovl_a:
    case FieldId::chstatus_ovl_b:
    case FieldId::chstatus_osp_a:
    case FieldId::chstatus_osp_b:
    case FieldId::chstatus_gr_head_a:
    case FieldId::chstatus_gr_head_b:
    case FieldId::chstatus_gr_head_peak_a:
    case FieldId::chstatus_gr_head_peak_b:
    case FieldId::chstatus_remhold_isp_a:
    case FieldId::chstatus_remhold_isp_b:
    case FieldId::chstatus_remhold_gr_a:
    case FieldId::chstatus_remhold_gr_b:
    case FieldId::chstatus_remhold_ovl_a:
    case FieldId::chstatus_remhold_ovl_b:
    case FieldId::chstatus_remhold_osp_a:
    case FieldId::chstatus_remhold_osp_b:
      break;
    default:
      return;
  }
  char line[256] = {};
  snprintf(line, sizeof(line),
           "LIVE_METER_TRACE stage=%s t_ms=%lu node=2.05 field=%s object=%04X age_ms=%lu len=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
           stage != 0 ? stage : "unknown", static_cast<unsigned long>(nowMs),
           fieldIdName(request.fieldId), static_cast<unsigned>(normalizedObjectKey(request)),
           static_cast<unsigned long>(ageMs), static_cast<unsigned>(frame.length),
           static_cast<unsigned>(frame.data[0]), static_cast<unsigned>(frame.data[1]),
           static_cast<unsigned>(frame.data[2]), static_cast<unsigned>(frame.data[3]),
           static_cast<unsigned>(frame.data[4]), static_cast<unsigned>(frame.data[5]),
           static_cast<unsigned>(frame.data[6]), static_cast<unsigned>(frame.data[7]));
  debugCli_.logText(line);
}

void R70GatewayV2::noteObjectTcpRead(const NormalizedRequest& request, uint32_t nowMs) {
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  queueObject0100HeartbeatFromAliveRequest(request, nowMs);
  logFocusedLiveMeter("r1_request", request, request.frame, nowMs);
  ObjectStat* stat = findOrCreateObjectStat(objectKey, request.fieldId);
  if (stat != 0) {
    ++stat->tcpReadRequests;
  }
}

void R70GatewayV2::queueObject0100HeartbeatFromAliveRequest(
    const NormalizedRequest& request,
    uint32_t nowMs) {
  if (!request.valid || !request.canId.valid || request.canId.device == 0 ||
      !isReadFunction(request.admin.functionCode)) return;
  const uint16_t objectKey = normalizedObjectKey(request);
  const bool aliveRequest = objectKey == 0x0111u || objectKey == 0x010Du ||
                            objectKey == 0x0300u || objectKey == 0x0500u ||
                            objectKey == 0x2600u || objectKey == 0x4600u;
  if (!aliveRequest) return;

  static const uint32_t kTriggerIntervalMs = 500u;
  static const uint32_t kPendingTimeoutMs = 2000u;
  AmpState* amp = state_.amp(request.canId.subnet, request.canId.device, false);
  if (amp == 0) return;
  if (amp->object0100HeartbeatQueuedMs != 0) {
    if (amp->object0100HeartbeatQueuedMs != 0 &&
        nowMs - amp->object0100HeartbeatQueuedMs >= kPendingTimeoutMs) {
      amp->object0100HeartbeatQueuedMs = 0;
    } else {
      return;
    }
  }
  if (amp->object0100HeartbeatLastTriggerMs != 0 &&
      nowMs - amp->object0100HeartbeatLastTriggerMs < kTriggerIntervalMs) return;
  if (currentBusUtilAvgPercent(nowMs) >= 70u || scheduler_.queueDepth() != 0 ||
      pendingTcpReplyCount() >= 20u) return;

  Frame frame = {};
  frame.canId = protocol::makeCanId(
      protocol::kObjectClassPrimaryMaster,
      Address{request.canId.subnet, request.canId.device});
  frame.length = 3;
  frame.data[0] = 0x01;
  frame.data[1] = 0x01;
  frame.data[2] = 0x00;
  NormalizedRequest heartbeat = {};
  if (!codec::normalizeCanFrame(frame, RequestOrigin::Tcp, heartbeat)) return;
  const char* reason = 0;
  if (!scheduler_.enqueueMaintenance(heartbeat, nowMs, counters_, reason)) return;
  amp->object0100HeartbeatQueuedMs = nowMs;
  amp->object0100HeartbeatLastTriggerMs = nowMs;
  ++r1Object0100HeartbeatCanTx_;
}

void R70GatewayV2::noteObjectCanTxForwarded(const NormalizedRequest& request) {
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  ObjectStat* stat = findOrCreateObjectStat(objectKey, request.fieldId);
  if (stat != 0) {
    ++stat->canTxForwarded;
  }
}

void R70GatewayV2::noteObjectCacheReply(const NormalizedRequest& request, uint32_t nowMs) {
  queueLiveRefresh(request, nowMs, true);
  const AmpState* amp = state_.amp(request.canId.subnet, request.canId.device);
  const FieldCache* cache = amp != 0 ? amp->cache(request.fieldId) : 0;
  if (cache != 0 && cache->valid && cache->hasReplyFrame) {
    const uint32_t ageMs = cache->lastUpdateMs <= nowMs ? nowMs - cache->lastUpdateMs : 0u;
    logFocusedLiveMeter("r1_cache_reply", request, cache->replyFrame, nowMs, ageMs);
  }
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  ObjectStat* stat = findOrCreateObjectStat(objectKey, request.fieldId);
  if (stat != 0) {
    ++stat->cacheReplies;
  }
}

void R70GatewayV2::noteObjectCacheStale(const NormalizedRequest& request) {
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  ObjectStat* stat = findOrCreateObjectStat(objectKey, request.fieldId);
  if (stat != 0) {
    ++stat->cacheStale;
  }
}

void R70GatewayV2::noteObjectCacheMiss(const NormalizedRequest& request) {
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  ObjectStat* stat = findOrCreateObjectStat(objectKey, request.fieldId);
  if (stat != 0) {
    ++stat->cacheMiss;
  }
}

void R70GatewayV2::noteObjectCoalesced(const NormalizedRequest& request) {
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  ObjectStat* stat = findOrCreateObjectStat(objectKey, request.fieldId);
  if (stat != 0) {
    ++stat->coalescedDuplicates;
  }
}

void R70GatewayV2::noteObjectQueueFull(const NormalizedRequest& request) {
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0xFFFF) {
    return;
  }
  ObjectStat* stat = findOrCreateObjectStat(objectKey, request.fieldId);
  if (stat != 0) {
    ++stat->queueFull;
  }
}

void R70GatewayV2::logTopObjectStats() const {
#if !R70_EXPERIMENTAL_DEBUG_CODE
  return;
#else
  struct RankedStat {
    const ObjectStat* stat;
    uint32_t score;
  };
  RankedStat top[16] = {};
  for (uint8_t index = 0; index < kObjectStatCapacity; ++index) {
    const ObjectStat& stat = objectStats_[index];
    if (!stat.used) {
      continue;
    }
    const uint32_t score = stat.tcpReadRequests + stat.canTxForwarded + stat.cacheReplies +
                           stat.cacheStale + stat.cacheMiss + stat.coalescedDuplicates + stat.queueFull;
    if (score == 0) {
      continue;
    }
    for (uint8_t slot = 0; slot < 16; ++slot) {
      if (top[slot].stat == 0 || score > top[slot].score) {
        for (int8_t shift = 15; shift > static_cast<int8_t>(slot); --shift) {
          top[shift] = top[shift - 1];
        }
        top[slot].stat = &stat;
        top[slot].score = score;
        break;
      }
    }
  }

  char labelBuffer[24] = {};
  for (uint8_t slot = 0; slot < 16; ++slot) {
    if (top[slot].stat == 0) {
      continue;
    }
    const ObjectStat& stat = *top[slot].stat;
    const char* label = 0;
    if (stat.fieldId != FieldId::Unknown) {
      label = fieldIdName(stat.fieldId);
    } else {
      snprintf(labelBuffer, sizeof(labelBuffer), "unknown_%04X", stat.objectKey);
      label = labelBuffer;
    }
    debugCli_.logTopCanTxObject(static_cast<uint8_t>(slot + 1),
                                stat.objectKey,
                                label,
                                stat.tcpReadRequests,
                                stat.canTxForwarded,
                                stat.cacheReplies,
                                stat.cacheStale,
                                stat.cacheMiss,
                                stat.coalescedDuplicates,
                                stat.queueFull);
  }
#endif
}

void R70GatewayV2::noteBootstrapReply(const NormalizedRequest& reply, uint32_t nowMs) {
  if (reply.canId.device == 0) {
    return;
  }
  AmpState* ampState = state_.amp(reply.canId.subnet, reply.canId.device, true);
  if (ampState == 0) {
    return;
  }
  if (ampState->bootstrapStartedMs == 0) {
    ampState->bootstrapStartedMs = nowMs;
  }

  uint8_t bootstrapIndex = 0;
  if (!classifyBootstrapReply(reply, bootstrapIndex)) {
    return;
  }

  const uint32_t mask = bootstrapMaskFor(bootstrapIndex);
  ampState->bootstrapSeenMask |= mask;
  if ((ampState->bootstrapRetryPendingMask & mask) != 0) {
    ampState->bootstrapRetryPendingMask &= ~mask;
    ampState->bootstrapRetrySentMs[bootstrapIndex] = 0;
    ++counters_.bootstrap_retry_replied;
  }
}

void R70GatewayV2::noteBootstrapRequestObserved(const NormalizedRequest& request, uint32_t nowMs) {
  if (!request.valid || !request.admin.valid || !isReadFunction(request.admin.functionCode) || request.canId.device == 0) {
    return;
  }
  uint8_t bootstrapIndex = 0;
  if (!classifyBootstrapReply(request, bootstrapIndex)) {
    return;
  }
  AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device, false);
  if (ampState == 0 || !ampState->used) {
    return;
  }
  if (ampState->bootstrapStartedMs == 0) {
    ampState->bootstrapStartedMs = nowMs;
  }
  const uint32_t mask = bootstrapMaskFor(bootstrapIndex);
  if ((kBootstrapOptionalMask & mask) != 0) {
    ampState->bootstrapRequestedMask |= mask;
  }
}

void R70GatewayV2::noteStartupBlockerRequestObserved(const NormalizedRequest& request, uint32_t nowMs) {
  if (!request.valid || !request.admin.valid || !isReadFunction(request.admin.functionCode) || request.canId.device == 0) {
    return;
  }
  const uint8_t family = classifyStartupBlockerFamily(request);
  if (family >= kStartupBlockerFamilyCount) {
    return;
  }
  AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device, true);
  if (ampState == 0 || !ampState->used) {
    return;
  }
  if (ampState->bootstrapStartedMs == 0) {
    ampState->bootstrapStartedMs = nowMs;
  }
  ampState->startupBlockerRequestedMask |= static_cast<uint16_t>(1u << family);
}

void R70GatewayV2::noteStartupBlockerReplyForwarded(const NormalizedRequest& reply, uint32_t nowMs) {
  if (!reply.valid || reply.canId.device == 0) {
    return;
  }
  AmpState* ampState = state_.amp(reply.canId.subnet, reply.canId.device, true);
  if (ampState == 0 || !ampState->used) {
    return;
  }
  if (ampState->bootstrapStartedMs == 0) {
    ampState->bootstrapStartedMs = nowMs;
  }

  // This function is called only after a frame was successfully sent to R1.
  // Mark the answered bootstrap object from that exact request/reply shape so
  // every forwarding path (CAN, mirror, exact and segmented caches) provides
  // the same proof. Do not depend on the CAN reply's function-code decoding.
  uint8_t bootstrapIndex = 0;
  if (classifyBootstrapReply(reply, bootstrapIndex)) {
    const uint32_t bootstrapMask = bootstrapMaskFor(bootstrapIndex);
    ampState->bootstrapSeenMask |= bootstrapMask;
    if ((ampState->bootstrapRetryPendingMask & bootstrapMask) != 0) {
      ampState->bootstrapRetryPendingMask &= ~bootstrapMask;
      ampState->bootstrapRetrySentMs[bootstrapIndex] = 0;
      ++counters_.bootstrap_retry_replied;
    }
  }

  if (!reply.admin.valid || !isReadFunction(reply.admin.functionCode)) {
    return;
  }
  const uint8_t family = classifyStartupBlockerFamily(reply);
  if (family >= kStartupBlockerFamilyCount) {
    return;
  }
  ampState->startupBlockerRepliedMask |= static_cast<uint16_t>(1u << family);
}

uint16_t R70GatewayV2::currentMissingStartupBlockerCount(
    uint8_t familyCounts[kStartupBlockerFamilyCount]) const {
  for (uint8_t index = 0; index < kStartupBlockerFamilyCount; ++index) {
    familyCounts[index] = 0;
  }
  uint16_t totalMissing = 0;
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    const uint16_t missingMask =
        static_cast<uint16_t>(ampState->startupBlockerRequestedMask & ~ampState->startupBlockerRepliedMask);
    if (missingMask == 0) {
      continue;
    }
    for (uint8_t family = 0; family < kStartupBlockerFamilyCount; ++family) {
      if ((missingMask & static_cast<uint16_t>(1u << family)) == 0) {
        continue;
      }
      ++familyCounts[family];
      ++totalMissing;
    }
  }
  return totalMissing;
}

const char* R70GatewayV2::worstMissingStartupBlockerFamilyName(uint8_t* missingNodes) const {
  uint8_t familyCounts[kStartupBlockerFamilyCount] = {};
  currentMissingStartupBlockerCount(familyCounts);
  uint8_t worstFamily = kStartupBlockerFamilyCount;
  uint8_t worstCount = 0;
  for (uint8_t family = 0; family < kStartupBlockerFamilyCount; ++family) {
    if (familyCounts[family] > worstCount) {
      worstCount = familyCounts[family];
      worstFamily = family;
    }
  }
  if (missingNodes != 0) {
    *missingNodes = worstCount;
  }
  return startupBlockerFamilyName(worstFamily);
}

void R70GatewayV2::resetRuntimeStabilityTracking() {
  lastRuntimeBaselineSummaryMs_ = 0;
  runtimeStableCandidateSinceMs_ = 0;
  runtimeStableEnteredMs_ = 0;
  lastRuntimePendingTcpReplies_ = 0;
  lastRuntimePendingCanReads_ = 0;
  lastRuntimeMissingStartupBlockers_ = 0;
  lastRuntimeBootstrapNodesIncomplete_ = 0;
  lastRuntimeR1TimeoutEventsSnapshot_ = counters_.r1_timeout_events;
  runtimeStableEntered_ = false;
}

void R70GatewayV2::logRuntimeBaselineSummary(uint32_t nowMs) {
  uint8_t familyCounts[kStartupBlockerFamilyCount] = {};
  const uint16_t missingStartupBlockers = currentMissingStartupBlockerCount(familyCounts);
  uint8_t worstMissingNodes = 0;
  const char* worstFamily = worstMissingStartupBlockerFamilyName(&worstMissingNodes);
  const uint16_t pendingTcpReplies = pendingTcpReplyCount();
  const uint16_t pendingCanReads = scheduler_.queueDepth();
  const uint16_t pendingTcpReplyThreshold = 64u;
  const uint16_t pendingCanReadThreshold = 8u;
  const uint16_t pendingTcpReplyGrowthSlack = 8u;
  const uint16_t pendingCanReadGrowthSlack = 2u;
  char line[256] = {};
  snprintf(line, sizeof(line),
           "RUNTIME_BASELINE active_amp_count=%u pending_tcp_replies=%u pending_can_reads=%u worst_device_reply_gap_ms=%lu r1_timeout_events=%lu bootstrap_nodes_incomplete=%lu worst_missing_object_family=%s",
           static_cast<unsigned>(state_.activeAmpCount()),
           static_cast<unsigned>(pendingTcpReplies),
           static_cast<unsigned>(pendingCanReads),
           static_cast<unsigned long>(slowCanWindow_.worstDeviceReplyGapMs),
           static_cast<unsigned long>(counters_.r1_timeout_events),
           static_cast<unsigned long>(counters_.bootstrap_nodes_incomplete),
           worstFamily);
  debugCli_.logText(line);

  const bool activeCountStable =
      state_.activeAmpCount() != 0 &&
      lastActiveAmpCountChangeMs_ != 0 &&
      nowMs >= lastActiveAmpCountChangeMs_ &&
      static_cast<uint32_t>(nowMs - lastActiveAmpCountChangeMs_) >= kRuntimeStableRequiredMs;
  const bool bootstrapStable =
      counters_.bootstrap_nodes_incomplete == 0 ||
      lastRuntimeBaselineSummaryMs_ == 0 ||
      counters_.bootstrap_nodes_incomplete <= lastRuntimeBootstrapNodesIncomplete_;
  const bool pendingNotGrowing =
      lastRuntimeBaselineSummaryMs_ == 0 ||
      pendingTcpReplies <= static_cast<uint16_t>(lastRuntimePendingTcpReplies_ + pendingTcpReplyGrowthSlack);
  const bool queueNotGrowing =
      lastRuntimeBaselineSummaryMs_ == 0 ||
      pendingCanReads <= static_cast<uint16_t>(lastRuntimePendingCanReads_ + pendingCanReadGrowthSlack);
  const bool pendingSmallEnough = pendingTcpReplies <= pendingTcpReplyThreshold;
  const bool queueLowEnough = pendingCanReads <= pendingCanReadThreshold;
  const bool replyGapHealthy = slowCanWindow_.worstDeviceReplyGapMs < 500u;
  const bool timeoutStable =
      lastRuntimeBaselineSummaryMs_ == 0 || counters_.r1_timeout_events == lastRuntimeR1TimeoutEventsSnapshot_;
  const bool runtimeStableNow =
      activeCountStable &&
      bootstrapStable &&
      pendingNotGrowing &&
      pendingSmallEnough &&
      queueNotGrowing &&
      queueLowEnough &&
      replyGapHealthy &&
      timeoutStable &&
      counters_.channel_incomplete_count == 0;
  if (runtimeStableNow) {
    if (runtimeStableCandidateSinceMs_ == 0) {
      runtimeStableCandidateSinceMs_ = nowMs;
    } else if (!runtimeStableEntered_ &&
               nowMs >= runtimeStableCandidateSinceMs_ &&
               static_cast<uint32_t>(nowMs - runtimeStableCandidateSinceMs_) >= kRuntimeStableRequiredMs) {
      runtimeStableEntered_ = true;
      runtimeStableEnteredMs_ = nowMs;
      snprintf(line, sizeof(line),
               "RUNTIME_STABLE_ENTERED uptime_ms=%lu active_amp_count=%u pending_tcp_replies=%u pending_can_reads=%u bootstrap_nodes_incomplete=%lu worst_missing_object_family=%s",
               static_cast<unsigned long>(nowMs),
               static_cast<unsigned>(state_.activeAmpCount()),
               static_cast<unsigned>(pendingTcpReplies),
               static_cast<unsigned>(pendingCanReads),
               static_cast<unsigned long>(counters_.bootstrap_nodes_incomplete),
               worstFamily);
      debugCli_.logText(line);
    }
  } else {
    runtimeStableCandidateSinceMs_ = 0;
  }

  lastRuntimeBaselineSummaryMs_ = nowMs;
  lastRuntimePendingTcpReplies_ = pendingTcpReplies;
  lastRuntimePendingCanReads_ = pendingCanReads;
  lastRuntimeMissingStartupBlockers_ = missingStartupBlockers;
  lastRuntimeBootstrapNodesIncomplete_ = static_cast<uint16_t>(counters_.bootstrap_nodes_incomplete);
  lastRuntimeR1TimeoutEventsSnapshot_ = counters_.r1_timeout_events;

  const uint8_t cacheReasonCode =
      statefulStatusCacheDisabledReason(postBootstrapStatusCacheFeatureEnabled(),
                                       tcpServer_.connected(),
                                       state_.activeAmpCount(),
                                       firstAmpDiscoveryMs_,
                                       lastAmpDiscoveryMs_,
                                       lastActiveAmpCountChangeMs_,
                                       bootstrapCompleteStableSinceMs_,
                                       counters_.bootstrap_nodes_incomplete,
                                       nowMs);
  const bool semanticMirrorEnabled = semanticMirrorActive(nowMs);
  const bool cacheEnabled =
      semanticMirrorEnabled || (runtimeStableEntered_ && cacheReasonCode == kStatefulStatusCacheActive);
  const bool shapingEnabled = runtimeStableEntered_ && gentleStatusShapingEnabled();
  const char* postStableReason =
      semanticMirrorEnabled ? "semantic_mirror_active"
                            : (!runtimeStableEntered_ ? "runtime_not_stable"
                                                      : (cacheEnabled ? "stable_runtime"
                                                                      : statefulStatusCacheDisabledReasonName(
                                                                            cacheReasonCode)));
  snprintf(line, sizeof(line),
           "POST_STABLE_FEATURE_STATE cache_enabled=%u shaping_enabled=%u reason=%s",
           static_cast<unsigned>(cacheEnabled ? 1u : 0u),
           static_cast<unsigned>(shapingEnabled ? 1u : 0u),
           postStableReason);
  debugCli_.logText(line);
  logSemanticMirrorSummary();
  logTcpPendingSummary(nowMs);
  resetSemanticMirrorSummaryWindow();
}

void R70GatewayV2::restartBootstrapPhase(uint32_t nowMs, bool resetPerNodeState) {
  bootstrapProtectionUntilMs_ =
      bootstrapProtectionFeatureEnabled() ? (nowMs + kBootstrapProtectionMs) : 0;
  bootstrapCompleteStableSinceMs_ = 0;
  bootstrapRecoveryWindowStartedMs_ = 0;
  bootstrapRecoverySentInWindow_ = 0;
  powerOnBootstrapRecoveryNextAmp_ = 0;
  resetRuntimeStabilityTracking();
  clearGentleStatusShapingEntries();
  clearSoftInflightCollapseEntries();
  clearDeferredSegmentedRequests();
  clearDeferredIdentRequests();
  clearStatefulStatusCache();
  clearSemanticMirror();
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* activeAmp = state_.activeAmpByIndex(ampIndex);
    if (activeAmp == 0) {
      continue;
    }
    AmpState* ampState = state_.amp(activeAmp->subnet, activeAmp->device, false);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    if (resetPerNodeState) {
      ampState->bootstrapStartedMs = nowMs;
      ampState->bootstrapLastBurstMs = 0;
      ampState->bootstrapLastIncompleteLogMs = 0;
      ampState->bootstrapSeenMask = 0;
      ampState->bootstrapRequestedMask = 0;
      ampState->startupBlockerRequestedMask = 0;
      ampState->startupBlockerRepliedMask = 0;
      ampState->bootstrapRetryPendingMask = 0;
      for (uint8_t index = 0; index < AmpState::kBootstrapObjectCount; ++index) {
        ampState->bootstrapRetrySentMs[index] = 0;
      }
      ampState->powerOnBootstrapPendingConfirmation = false;
      ampState->powerOnBootstrapUntilMs = 0;
    } else if (ampState->bootstrapStartedMs == 0) {
      ampState->bootstrapStartedMs = nowMs;
    }
  }
}

void R70GatewayV2::armPowerOnBootstrap(uint8_t subnet, uint8_t device, uint32_t nowMs) {
  // A power command can arrive while the first TCP/startup bootstrap is still
  // draining. That global bootstrap already covers every discovered node;
  // resetting the same node here would duplicate the startup burst and erase
  // progress that R1 has already made.
  if (bootstrapCompleteStableSinceMs_ == 0) {
    char line[160] = {};
    snprintf(line, sizeof(line),
             "POWER_ON_NODE_BOOTSTRAP_SKIPPED node=%u.%02u reason=startup_bootstrap_active",
             static_cast<unsigned>(subnet), static_cast<unsigned>(device));
    debugCli_.logText(line);
    return;
  }
  AmpState* ampState = state_.amp(subnet, device, false);
  if (ampState == 0 || !ampState->used || device == 0) {
    return;
  }
  ampState->bootstrapStartedMs = nowMs;
  ampState->bootstrapLastBurstMs = 0;
  ampState->bootstrapLastIncompleteLogMs = 0;
  ampState->bootstrapSeenMask = 0;
  ampState->bootstrapRequestedMask = 0;
  ampState->startupBlockerRequestedMask = 0;
  ampState->startupBlockerRepliedMask = 0;
  ampState->bootstrapRetryPendingMask = 0;
  for (uint8_t index = 0; index < AmpState::kBootstrapObjectCount; ++index) {
    ampState->bootstrapRetrySentMs[index] = 0;
  }
  ampState->powerOnBootstrapPendingConfirmation = true;
  ampState->powerOnBootstrapUntilMs = nowMs + kBootstrapProtectionMs;

  char line[160] = {};
  snprintf(line, sizeof(line),
           "POWER_ON_NODE_BOOTSTRAP_ARMED node=%u.%02u protection_ms=%lu",
           static_cast<unsigned>(subnet), static_cast<unsigned>(device),
           static_cast<unsigned long>(kBootstrapProtectionMs));
  debugCli_.logText(line);
}

bool R70GatewayV2::powerOnBootstrapActive(const AmpState& ampState,
                                           uint32_t nowMs) const {
  return ampState.powerOnBootstrapUntilMs != 0 &&
         static_cast<int32_t>(nowMs - ampState.powerOnBootstrapUntilMs) < 0;
}

bool R70GatewayV2::bootstrapProtectionFeatureEnabled() const {
  return bootstrapProtectionRuntimeEnabled_;
}

bool R70GatewayV2::bootstrapProtectionActive(uint32_t nowMs) const {
  return bootstrapProtectionFeatureEnabled() &&
         bootstrapProtectionUntilMs_ != 0 &&
         static_cast<int32_t>(nowMs - bootstrapProtectionUntilMs_) < 0;
}

uint32_t R70GatewayV2::bootstrapRequiredMask(const AmpState& ampState) const {
  return kBootstrapAlwaysRequiredMask | (ampState.bootstrapRequestedMask & kBootstrapOptionalMask);
}

bool R70GatewayV2::buildBootstrapRetryRequest(uint8_t subnet,
                                              uint8_t device,
                                              uint8_t bootstrapIndex,
                                              NormalizedRequest& request) const {
  request = {};
  const BootstrapRequestSpec* spec = bootstrapSpec(bootstrapIndex);
  if (spec == 0 || device == 0) {
    return false;
  }

  Frame frame = {};
  frame.canId = static_cast<uint16_t>(kBootstrapPrimaryCanIdBase |
                                      ((static_cast<uint16_t>(subnet) & 0x07u) << 6) |
                                      (static_cast<uint16_t>(device) & 0x3Fu));
  frame.length = spec->payloadLength;
  for (uint8_t index = 0; index < spec->payloadLength; ++index) {
    frame.data[index] = spec->payload[index];
  }
  return codec::normalizeCanFrame(frame, RequestOrigin::Can, request);
}

bool R70GatewayV2::bootstrapRetryFeatureEnabled() const {
  return bootstrapRetryRuntimeEnabled_;
}

void R70GatewayV2::logBootstrapIncompleteNodes(uint32_t nowMs) {
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* currentAmp = state_.activeAmpByIndex(ampIndex);
    if (currentAmp == 0) {
      continue;
    }
    AmpState* ampState = state_.amp(currentAmp->subnet, currentAmp->device, false);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    if (ampState->bootstrapStartedMs == 0) {
      if (ampState->lastSeenMs == 0) {
        continue;
      }
      ampState->bootstrapStartedMs = ampState->lastSeenMs;
    }
    const uint32_t missingMask = bootstrapRequiredMask(*ampState) & ~ampState->bootstrapSeenMask;
    if (missingMask == 0) {
      continue;
    }
    if (static_cast<int32_t>(nowMs - (ampState->bootstrapStartedMs + kBootstrapIncompleteLogThresholdMs)) < 0) {
      continue;
    }
    if (ampState->bootstrapLastIncompleteLogMs != 0 &&
        static_cast<int32_t>(nowMs - (ampState->bootstrapLastIncompleteLogMs +
                                      kBootstrapIncompleteLogThresholdMs)) < 0) {
      continue;
    }

    char line[256] = {};
    const uint32_t lastReqAgeMs = ampState->lastTcpRxRequestMs != 0 ? nowMs - ampState->lastTcpRxRequestMs : 0;
    const uint32_t lastCanTxAgeMs = ampState->lastCanTxPollMs != 0 ? nowMs - ampState->lastCanTxPollMs : 0;
    const uint32_t lastCanRxAgeMs = ampState->lastCanRxMs != 0 ? nowMs - ampState->lastCanRxMs : 0;
    const uint32_t lastTcpReplyAgeMs = ampState->lastTcpTxToR1Ms != 0 ? nowMs - ampState->lastTcpTxToR1Ms : 0;
    size_t offset = static_cast<size_t>(snprintf(line, sizeof(line),
                                                 "BOOTSTRAP_NODE_MISSING node=%u.%02u missing=",
                                                 static_cast<unsigned>(ampState->subnet),
                                                 static_cast<unsigned>(ampState->device)));
    bool first = true;
    for (uint8_t index = 0; index < AmpState::kBootstrapObjectCount; ++index) {
      if ((missingMask & bootstrapMaskFor(index)) == 0) {
        continue;
      }
      offset += static_cast<size_t>(snprintf(line + offset,
                                             offset < sizeof(line) ? sizeof(line) - offset : 0,
                                             "%s%s",
                                             first ? "" : ",",
                                             bootstrapLabel(index)));
      first = false;
    }
    snprintf(line + offset,
             offset < sizeof(line) ? sizeof(line) - offset : 0,
             " last_req_age_ms=%lu last_can_tx_age_ms=%lu last_can_rx_age_ms=%lu last_tcp_reply_age_ms=%lu",
             static_cast<unsigned long>(lastReqAgeMs),
             static_cast<unsigned long>(lastCanTxAgeMs),
             static_cast<unsigned long>(lastCanRxAgeMs),
             static_cast<unsigned long>(lastTcpReplyAgeMs));
    debugCli_.logText(line);
    ampState->bootstrapLastIncompleteLogMs = nowMs;
  }
}

void R70GatewayV2::processBootstrapRecovery(uint32_t nowMs) {
  if (!bootstrapRetryFeatureEnabled()) {
    return;
  }
  if (bootstrapRecoveryWindowStartedMs_ == 0 ||
      static_cast<int32_t>(nowMs - (bootstrapRecoveryWindowStartedMs_ + 1000u)) >= 0) {
    bootstrapRecoveryWindowStartedMs_ = nowMs;
    bootstrapRecoverySentInWindow_ = 0;
  }
  bool targetedPowerOnRecovery = false;
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState != 0 && powerOnBootstrapActive(*ampState, nowMs) &&
        !ampState->powerOnBootstrapPendingConfirmation) {
      targetedPowerOnRecovery = true;
      break;
    }
  }
  const uint8_t retryBudget = targetedPowerOnRecovery
                                  ? kPowerOnBootstrapRetriesPerSecond
                                  : 10u;
  if (bootstrapRecoverySentInWindow_ >= retryBudget) {
    return;
  }
  const uint8_t activeAmpCount = state_.activeAmpCount();
  for (uint8_t offset = 0; offset < activeAmpCount; ++offset) {
    const uint8_t ampIndex = targetedPowerOnRecovery
                                 ? static_cast<uint8_t>((powerOnBootstrapRecoveryNextAmp_ + offset) % activeAmpCount)
                                 : offset;
    const AmpState* currentAmp = state_.activeAmpByIndex(ampIndex);
    if (currentAmp == 0) {
      continue;
    }
    AmpState* ampState = state_.amp(currentAmp->subnet, currentAmp->device, false);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    if (targetedPowerOnRecovery &&
        (!powerOnBootstrapActive(*ampState, nowMs) ||
         ampState->powerOnBootstrapPendingConfirmation)) {
      continue;
    }
    if (ampState->bootstrapStartedMs == 0) {
      if (ampState->lastSeenMs == 0) {
        continue;
      }
      ampState->bootstrapStartedMs = ampState->lastSeenMs;
    }
    const bool targetedPowerOnBootstrap = powerOnBootstrapActive(*ampState, nowMs);
    const uint32_t retryStartMs = targetedPowerOnBootstrap
                                      ? kPowerOnBootstrapRetryStartMs
                                      : kBootstrapRetryStartMs;
    // Do not poll an amplifier merely because Power ON was written. Its first
    // real CAN reply is the proof that it is awake enough for targeted refill.
    if ((targetedPowerOnBootstrap && ampState->powerOnBootstrapPendingConfirmation) ||
        static_cast<int32_t>(nowMs - (ampState->bootstrapStartedMs + retryStartMs)) < 0) {
      continue;
    }

    const uint32_t missingMask = bootstrapRequiredMask(*ampState) & ~ampState->bootstrapSeenMask;
    if (missingMask == 0) {
      ampState->bootstrapRetryPendingMask = 0;
      continue;
    }
    if (ampState->bootstrapLastBurstMs != 0 &&
        static_cast<int32_t>(nowMs - (ampState->bootstrapLastBurstMs + kBootstrapRetryBurstMs)) < 0) {
      continue;
    }

    for (uint8_t index = 0; index < AmpState::kBootstrapObjectCount; ++index) {
      const uint32_t mask = bootstrapMaskFor(index);
      if ((missingMask & mask) == 0) {
        continue;
      }
      if ((ampState->bootstrapRetryPendingMask & mask) != 0 &&
          ampState->bootstrapRetrySentMs[index] != 0 &&
          static_cast<int32_t>(nowMs - (ampState->bootstrapRetrySentMs[index] +
                                        kBootstrapRetryBurstMs)) >= 0) {
        ampState->bootstrapRetryPendingMask &= ~mask;
        ampState->bootstrapRetrySentMs[index] = 0;
        ++counters_.bootstrap_retry_failed;
      }
      if ((ampState->bootstrapRetryPendingMask & mask) != 0) {
        continue;
      }

      NormalizedRequest request = {};
      if (!buildBootstrapRetryRequest(ampState->subnet, ampState->device, index, request)) {
        continue;
      }
      if (state_.shouldSuppressPending(request) || state_.shouldSuppressRecent(request, nowMs)) {
        continue;
      }

      const char* failureReason = 0;
      if (!scheduler_.enqueue(request, 0, false, nowMs, counters_, failureReason)) {
        if (failureReason != 0 && strcmp(failureReason, "queue_full") == 0) {
          ++counters_.bootstrap_retry_failed;
        }
        ampState->bootstrapLastBurstMs = nowMs;
        break;
      }

      ampState->bootstrapRetryPendingMask =
          ampState->bootstrapRetryPendingMask | mask;
      ampState->bootstrapRetrySentMs[index] = nowMs;
      ampState->bootstrapLastBurstMs = nowMs;
      ++bootstrapRecoverySentInWindow_;
      ++counters_.bootstrap_retry_sent;
      ++counters_.retry_requests_sent;
      if (targetedPowerOnRecovery && activeAmpCount != 0) {
        powerOnBootstrapRecoveryNextAmp_ =
            static_cast<uint8_t>((ampIndex + 1u) % activeAmpCount);
      }
      break;
    }
    if (bootstrapRecoverySentInWindow_ >= retryBudget) {
      return;
    }
  }
}

void R70GatewayV2::updateBootstrapCounters(uint32_t nowMs) {
  counters_.bootstrap_nodes_total = 0;
  counters_.bootstrap_nodes_incomplete = 0;
  counters_.bootstrap_worst_node_subnet = 0;
  counters_.bootstrap_worst_node_device = 0;
  counters_.bootstrap_worst_missing_object = 0;
  counters_.channel_incomplete_count = 0;
  counters_.bootstrap_incomplete_after_10s = 0;

  uint8_t worstMissingCount = 0;
  uint32_t worstNodeAgeMs = 0;
  uint8_t missingCounts[AmpState::kBootstrapObjectCount] = {};
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    ++counters_.bootstrap_nodes_total;
    if (ampState->bootstrapStartedMs == 0) {
      continue;
    }
    const uint32_t missingMask = bootstrapRequiredMask(*ampState) & ~ampState->bootstrapSeenMask;
    if (missingMask == 0) {
      continue;
    }
    ++counters_.bootstrap_nodes_incomplete;
    const uint32_t nodeAgeMs = nowMs - ampState->bootstrapStartedMs;
    if ((missingMask & kBootstrapChannelRequiredMask) != 0 &&
        static_cast<int32_t>(nodeAgeMs - kBootstrapIncompleteLogThresholdMs) >= 0) {
      ++counters_.channel_incomplete_count;
    }
    if (static_cast<int32_t>(nodeAgeMs - kBootstrapIncompleteLogThresholdMs) >= 0) {
      ++counters_.bootstrap_incomplete_after_10s;
    }
    for (uint8_t index = 0; index < AmpState::kBootstrapObjectCount; ++index) {
      if ((missingMask & bootstrapMaskFor(index)) != 0) {
        ++missingCounts[index];
      }
    }
    const uint8_t missingCount = bitCount32(missingMask);
    if (missingCount > worstMissingCount ||
        (missingCount == worstMissingCount && nodeAgeMs > worstNodeAgeMs)) {
      worstMissingCount = missingCount;
      worstNodeAgeMs = nodeAgeMs;
      counters_.bootstrap_worst_node_subnet = ampState->subnet;
      counters_.bootstrap_worst_node_device = ampState->device;
    }
  }

  uint8_t worstObjectIndex = AmpState::kBootstrapObjectCount;
  uint8_t worstObjectCount = 0;
  for (uint8_t index = 0; index < AmpState::kBootstrapObjectCount; ++index) {
    if (missingCounts[index] > worstObjectCount) {
      worstObjectCount = missingCounts[index];
      worstObjectIndex = index;
    }
  }
  if (worstObjectIndex < AmpState::kBootstrapObjectCount) {
    counters_.bootstrap_worst_missing_object = bootstrapObjectKey(worstObjectIndex);
  }
  if (counters_.bootstrap_nodes_incomplete == 0 && counters_.bootstrap_nodes_total != 0) {
    if (bootstrapCompleteStableSinceMs_ == 0) {
      bootstrapCompleteStableSinceMs_ = nowMs;
    }
  } else {
    bootstrapCompleteStableSinceMs_ = 0;
  }
}

const char* R70GatewayV2::regressionFeatureName(RegressionFeature feature) {
  switch (feature) {
    case RegressionFeature::GentleStatusShaping:
      return "ENABLE_GENTLE_STATUS_SHAPING";
    case RegressionFeature::BootstrapProtection:
      return "ENABLE_BOOTSTRAP_PROTECTION";
    case RegressionFeature::BootstrapRetry:
      return "ENABLE_BOOTSTRAP_RETRY";
    case RegressionFeature::SegmentedDampening:
      return "ENABLE_SEGMENTED_DAMPENING";
    case RegressionFeature::FanoutAttach:
      return "ENABLE_FANOUT_ATTACH";
    case RegressionFeature::PostBootstrapStatusCache:
      return "ENABLE_POST_BOOTSTRAP_STATUS_CACHE";
    default:
      return "unknown";
  }
}

void R70GatewayV2::disableRegressionFeature(RegressionFeature feature, const char* reason) {
  bool* enabled = 0;
  switch (feature) {
    case RegressionFeature::GentleStatusShaping:
      if (!gentleStatusShapingAutoDisableEligible(reason)) {
        return;
      }
      enabled = &gentleStatusShapingRuntimeEnabled_;
      break;
    case RegressionFeature::BootstrapProtection:
      enabled = &bootstrapProtectionRuntimeEnabled_;
      bootstrapProtectionUntilMs_ = 0;
      break;
    case RegressionFeature::BootstrapRetry:
      enabled = &bootstrapRetryRuntimeEnabled_;
      bootstrapRecoveryWindowStartedMs_ = 0;
      bootstrapRecoverySentInWindow_ = 0;
      break;
    case RegressionFeature::SegmentedDampening:
      enabled = &segmentedDampeningRuntimeEnabled_;
      clearDeferredSegmentedRequests();
      break;
    case RegressionFeature::FanoutAttach:
      if (!gentleStatusShapingAutoDisableEligible(reason)) {
        return;
      }
      enabled = &fanoutAttachRuntimeEnabled_;
      clearSoftInflightCollapseEntries();
      break;
    case RegressionFeature::PostBootstrapStatusCache:
      autoDisableStatefulStatusCache(kStatefulStatusCacheAutoDisabled, reason);
      break;
    default:
      break;
  }
  if (feature == RegressionFeature::PostBootstrapStatusCache) {
    return;
  }
  if (enabled == 0 || !*enabled) {
    return;
  }
  *enabled = false;
  if (feature == RegressionFeature::GentleStatusShaping) {
    clearGentleStatusShapingEntries();
  }
  char line[128] = {};
  snprintf(line,
           sizeof(line),
           "FEATURE_AUTO_DISABLED feature=%s reason=%s",
           regressionFeatureName(feature),
           reason != 0 ? reason : "unspecified");
  debugCli_.logText(line);
}

void R70GatewayV2::disableActiveRegressionFeatures(const char* reason, bool includePostBootstrapStatusCache) {
  disableRegressionFeature(RegressionFeature::GentleStatusShaping, reason);
  disableRegressionFeature(RegressionFeature::BootstrapProtection, reason);
  disableRegressionFeature(RegressionFeature::BootstrapRetry, reason);
  disableRegressionFeature(RegressionFeature::SegmentedDampening, reason);
  disableRegressionFeature(RegressionFeature::FanoutAttach, reason);
  if (includePostBootstrapStatusCache) {
    disableRegressionFeature(RegressionFeature::PostBootstrapStatusCache, reason);
  }
}

void R70GatewayV2::updateRegressionCounters(uint32_t nowMs) {
  counters_.deferred_requests_active = activeGentleDeferredCount();
  counters_.fanout_waiters_active = activeFanoutWaiterCount();
  const bool statefulCacheActiveNow = postBootstrapStatusCacheActive(nowMs);
  if (statefulCacheActiveNow && !statefulStatusCacheLastEnabled_) {
    statefulStatusCacheHasEverBeenActive_ = true;
    statefulStatusCacheLastEnabled_ = true;
    r1TimeoutEventsAtStatefulCacheEnable_ = counters_.r1_timeout_events;
    lastR1TimeoutEvents_ = counters_.r1_timeout_events;
    uint8_t devicesOver5s = 0;
    const uint32_t maxAgeMs = maxR1ReplyAgeMs(nowMs, &devicesOver5s);
    char line[256] = {};
    snprintf(line,
             sizeof(line),
             "STATEFUL_CACHE_ENABLED uptime_ms=%lu active_amp_count=%u pending_tcp_replies=%u pending_can_reads=%u max_r1_reply_age_ms_at_activation=%lu devices_over_5s_at_activation=%u",
             static_cast<unsigned long>(nowMs),
             static_cast<unsigned>(state_.activeAmpCount()),
             static_cast<unsigned>(pendingTcpReplyCount()),
             static_cast<unsigned>(scheduler_.queueDepth()),
             static_cast<unsigned long>(maxAgeMs),
             static_cast<unsigned>(devicesOver5s));
    debugCli_.logText(line);
    logR1FreshnessSummary(nowMs, "R1_FRESHNESS_AT_CACHE_ACTIVATION");
  } else if (!statefulCacheActiveNow) {
    statefulStatusCacheLastEnabled_ = false;
  }
  const bool gentleStatusShapingActiveNow = gentleStatusShapingEnabled();
  if (gentleStatusShapingActiveNow && !gentleStatusShapingLastEnabled_) {
    gentleStatusShapingHasEverBeenActive_ = true;
    gentleStatusShapingLastEnabled_ = true;
    r1TimeoutEventsAtGentleStatusShapingEnable_ = counters_.r1_timeout_events;
    uint8_t devicesOver5s = 0;
    const uint32_t maxAgeMs = maxR1ReplyAgeMs(nowMs, &devicesOver5s);
    char line[256] = {};
    snprintf(line,
             sizeof(line),
             "GENTLE_STATUS_SHAPING_ENABLED uptime_ms=%lu active_amp_count=%u pending_tcp_replies=%u pending_can_reads=%u max_r1_reply_age_ms_at_activation=%lu devices_over_5s_at_activation=%u",
             static_cast<unsigned long>(nowMs),
             static_cast<unsigned>(state_.activeAmpCount()),
             static_cast<unsigned>(pendingTcpReplyCount()),
             static_cast<unsigned>(scheduler_.queueDepth()),
             static_cast<unsigned long>(maxAgeMs),
             static_cast<unsigned>(devicesOver5s));
    debugCli_.logText(line);
    logR1FreshnessSummary(nowMs, "R1_FRESHNESS_AT_SHAPING_ACTIVATION");
  } else if (!gentleStatusShapingActiveNow) {
    gentleStatusShapingLastEnabled_ = false;
  }
  if (statefulCacheActiveNow &&
      counters_.stateful_cache_age_max_ms > postBootstrapStatusCacheHardMaxAgeMs()) {
    autoDisableStatefulStatusCache(kStatefulStatusCacheAutoDisabledAge,
                                   "cache_age_gt_1500ms");
  }
  if (statefulCacheActiveNow &&
      counters_.channel_incomplete_count > lastChannelIncompleteCount_) {
    autoDisableStatefulStatusCache(kStatefulStatusCacheAutoDisabledChannelIncrease,
                                   "channel_incomplete_increase");
  }
  if (statefulCacheActiveNow &&
      counters_.subnet_service_gap_count > lastSubnetServiceGapCount_) {
    autoDisableStatefulStatusCache(kStatefulStatusCacheAutoDisabledServiceGap,
                                   "subnet_service_gap");
  }
  if (statefulStatusCacheDisabledReasonCode_ >= kStatefulStatusCacheAutoDisabled &&
      !postBootstrapStatusCacheFeatureEnabled()) {
    counters_.stateful_cache_enabled = 0;
    counters_.stateful_cache_disabled_reason = statefulStatusCacheDisabledReasonCode_;
  } else {
    statefulStatusCacheDisabledReasonCode_ =
        statefulStatusCacheDisabledReason(postBootstrapStatusCacheFeatureEnabled(),
                                         tcpServer_.connected(),
                                         state_.activeAmpCount(),
                                         firstAmpDiscoveryMs_,
                                         lastAmpDiscoveryMs_,
                                         lastActiveAmpCountChangeMs_,
                                         bootstrapCompleteStableSinceMs_,
                                         counters_.bootstrap_nodes_incomplete,
                                         nowMs);
    counters_.stateful_cache_enabled = statefulStatusCacheDisabledReasonCode_ == kStatefulStatusCacheActive ? 1u : 0u;
    counters_.stateful_cache_disabled_reason = statefulStatusCacheDisabledReasonCode_;
  }
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* activeAmp = state_.activeAmpByIndex(ampIndex);
    if (activeAmp == 0) {
      continue;
    }
    AmpState* ampState = state_.amp(activeAmp->subnet, activeAmp->device, false);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    if (ampState->lastTcpRxRequestMs == 0 ||
        ampState->lastTcpRxRequestMs <= ampState->lastTcpTxToR1Ms) {
      ampState->r1TimeoutActive = false;
      continue;
    }
    const uint32_t requestAgeMs = nowMs - ampState->lastTcpRxRequestMs;
    if (requestAgeMs > kWatchdogCriticalAgeMs && !ampState->r1TimeoutActive) {
      ampState->r1TimeoutActive = true;
      ++counters_.r1_timeout_events;
      ++counters_.subnet_service_gap_count;
    } else if (requestAgeMs <= kWatchdogCriticalAgeMs) {
      ampState->r1TimeoutActive = false;
    }
  }
  if (counters_.r1_timeout_events > lastR1TimeoutEvents_) {
    beginTimeoutWindowDump(nowMs);
    pauseSemanticMirrorBackgroundRefresh(nowMs, "r1_timeout");
  } else if (slowCanWindow_.worstDeviceReplyGapMs > kSemanticMirrorBackgroundRefreshPauseServiceGapMs) {
    pauseSemanticMirrorBackgroundRefresh(nowMs, "service_gap");
  }
  lastChannelIncompleteCount_ = counters_.channel_incomplete_count;
  lastSubnetServiceGapCount_ = counters_.subnet_service_gap_count;
}

void R70GatewayV2::evaluateRegressionGuards(uint32_t nowMs) {
  (void)nowMs;
  const bool timeoutRegression = counters_.r1_timeout_events > lastR1TimeoutEvents_;
  const bool incompleteRegression =
      counters_.channel_incomplete_count != 0 || counters_.bootstrap_incomplete_after_10s != 0;
  const bool pendingAgeRegression = counters_.normal_read_queue_oldest_age_ms > kWatchdogCriticalAgeMs;
  if (!timeoutRegression && !incompleteRegression && !pendingAgeRegression) {
    lastR1TimeoutEvents_ = counters_.r1_timeout_events;
    return;
  }
  if (timeoutRegression) {
    disableActiveRegressionFeatures("r1_timeout_events", false);
    if (statefulStatusCacheHasEverBeenActive_ &&
        counters_.r1_timeout_events > r1TimeoutEventsAtStatefulCacheEnable_) {
      autoDisableStatefulStatusCache(kStatefulStatusCacheAutoDisabled, "r1_timeout_events");
    }
  }
  if (incompleteRegression) {
    disableActiveRegressionFeatures(counters_.channel_incomplete_count != 0
                                        ? "channel_incomplete"
                                        : "bootstrap_incomplete_after_10s");
  }
  if (pendingAgeRegression) {
    disableActiveRegressionFeatures("pending_age_gt_1500ms");
  }
  lastR1TimeoutEvents_ = counters_.r1_timeout_events;
}

void R70GatewayV2::printSlowCanSummary(Print& out) const {
  if (!slowCanWindow_.initialized) {
    resetSlowCanWindow(millis());
  }

  const uint32_t nowMs = millis();
  const uint32_t durationMs =
      slowCanWindow_.startedMs != 0 && nowMs > slowCanWindow_.startedMs ? nowMs - slowCanWindow_.startedMs : 1u;
  uint8_t current100msBucket = static_cast<uint8_t>(durationMs / 100u);
  if (current100msBucket >= 100u) {
    current100msBucket = 99u;
  }
  uint8_t current1sBucket = static_cast<uint8_t>(durationMs / 1000u);
  if (current1sBucket >= 10u) {
    current1sBucket = 9u;
  }
  const uint32_t bitrate = config_.canBitrate != 0 ? config_.canBitrate : 100000u;
  const uint32_t avgBusUtilPercent =
      bitrate != 0
          ? static_cast<uint32_t>((slowCanWindow_.estimatedBusBitsTotal * 100000ull) /
                                  (static_cast<uint64_t>(bitrate) * durationMs))
          : 0u;
  const uint32_t peakBusUtilPercent100ms =
      bitrate != 0
          ? static_cast<uint32_t>((static_cast<uint64_t>(slowCanWindow_.peakBusBitsPer100ms) * 1000ull) /
                                  static_cast<uint64_t>(bitrate))
          : 0u;

  char line[320] = {};
  snprintf(line, sizeof(line),
           "CAN_BUS_SUMMARY tx10s=%lu rx10s=%lu tx_frames_per_100ms=%u rx_frames_per_100ms=%u tx_frames_per_1s=%u rx_frames_per_1s=%u util_avg=%lu util_peak100ms=%lu",
           static_cast<unsigned long>(slowCanWindow_.tx10sCount),
           static_cast<unsigned long>(slowCanWindow_.rx10sCount),
           static_cast<unsigned>(slowCanWindow_.txFramesPer100msBins[current100msBucket]),
           static_cast<unsigned>(slowCanWindow_.rxFramesPer100msBins[current100msBucket]),
           static_cast<unsigned>(slowCanWindow_.txFramesPer1sBins[current1sBucket]),
           static_cast<unsigned>(slowCanWindow_.rxFramesPer1sBins[current1sBucket]),
           static_cast<unsigned long>(avgBusUtilPercent),
           static_cast<unsigned long>(peakBusUtilPercent100ms));
  out.println(line);

  if (slowCanWindow_.worstLatencyMs == 0) {
    snprintf(line, sizeof(line),
             "SLOW_CAN_SUMMARY over500_10s=%lu over1000_10s=%lu worst=0 node=none object=none ch=0",
             static_cast<unsigned long>(slowCanWindow_.over500Count),
             static_cast<unsigned long>(slowCanWindow_.over1000Count));
  } else {
    snprintf(line, sizeof(line),
             "SLOW_CAN_SUMMARY over500_10s=%lu over1000_10s=%lu worst=%u node=%u.%02u object=%04X ch=%u src=%s",
             static_cast<unsigned long>(slowCanWindow_.over500Count),
             static_cast<unsigned long>(slowCanWindow_.over1000Count),
             static_cast<unsigned>(slowCanWindow_.worstLatencyMs),
             static_cast<unsigned>(slowCanWindow_.worstSubnet),
             static_cast<unsigned>(slowCanWindow_.worstDevice),
             static_cast<unsigned>(slowCanWindow_.worstObjectKey),
             static_cast<unsigned>(slowCanWindow_.worstChannel),
             slowTraceCategoryName(slowCanWindow_.worstCategory));
  }
  out.println(line);

  const SlowObjectBucket* topObjects[5] = {};
  for (uint8_t index = 0; index < kSlowObjectBucketCapacity; ++index) {
    const SlowObjectBucket& bucket = slowCanWindow_.objectBuckets[index];
    if (!bucket.used || bucket.over500Count == 0) {
      continue;
    }
    for (uint8_t slot = 0; slot < 5; ++slot) {
      if (topObjects[slot] == 0 ||
          bucket.over500Count > topObjects[slot]->over500Count ||
          (bucket.over500Count == topObjects[slot]->over500Count &&
           bucket.maxLatencyMs > topObjects[slot]->maxLatencyMs)) {
        for (int8_t shift = 4; shift > static_cast<int8_t>(slot); --shift) {
          topObjects[shift] = topObjects[shift - 1];
        }
        topObjects[slot] = &bucket;
        break;
      }
    }
  }
  for (uint8_t slot = 0; slot < 5; ++slot) {
    if (topObjects[slot] == 0) {
      continue;
    }
    snprintf(line, sizeof(line),
             "WORST_OBJECT object=%04X count=%lu max=%u",
             static_cast<unsigned>(topObjects[slot]->objectKey),
             static_cast<unsigned long>(topObjects[slot]->over500Count),
             static_cast<unsigned>(topObjects[slot]->maxLatencyMs));
    out.println(line);
  }

  const SlowNodeBucket* topNodes[5] = {};
  for (uint8_t index = 0; index < kSlowNodeBucketCapacity; ++index) {
    const SlowNodeBucket& bucket = slowCanWindow_.nodeBuckets[index];
    if (!bucket.used || bucket.over500Count == 0) {
      continue;
    }
    for (uint8_t slot = 0; slot < 5; ++slot) {
      if (topNodes[slot] == 0 ||
          bucket.over500Count > topNodes[slot]->over500Count ||
          (bucket.over500Count == topNodes[slot]->over500Count &&
           bucket.maxLatencyMs > topNodes[slot]->maxLatencyMs)) {
        for (int8_t shift = 4; shift > static_cast<int8_t>(slot); --shift) {
          topNodes[shift] = topNodes[shift - 1];
        }
        topNodes[slot] = &bucket;
        break;
      }
    }
  }
  for (uint8_t slot = 0; slot < 5; ++slot) {
    if (topNodes[slot] == 0) {
      continue;
    }
    snprintf(line, sizeof(line),
             "WORST_NODE node=%u.%02u count=%lu max=%u",
             static_cast<unsigned>(topNodes[slot]->subnet),
             static_cast<unsigned>(topNodes[slot]->device),
             static_cast<unsigned long>(topNodes[slot]->over500Count),
             static_cast<unsigned>(topNodes[slot]->maxLatencyMs));
    out.println(line);
  }

  uint8_t worstSubnet = 0xFFu;
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    const SlowSubnetBucket& bucket = slowCanWindow_.subnetBuckets[subnet];
    if (bucket.over500Count == 0) {
      continue;
    }
    if (worstSubnet == 0xFFu ||
        bucket.over500Count > slowCanWindow_.subnetBuckets[worstSubnet].over500Count ||
        (bucket.over500Count == slowCanWindow_.subnetBuckets[worstSubnet].over500Count &&
         bucket.maxLatencyMs > slowCanWindow_.subnetBuckets[worstSubnet].maxLatencyMs)) {
      worstSubnet = subnet;
    }
  }
  if (worstSubnet != 0xFFu) {
    const SlowSubnetBucket& bucket = slowCanWindow_.subnetBuckets[worstSubnet];
    snprintf(line, sizeof(line),
             "WORST_SUBNET subnet=%u count=%lu max=%u",
             static_cast<unsigned>(worstSubnet),
             static_cast<unsigned long>(bucket.over500Count),
             static_cast<unsigned>(bucket.maxLatencyMs));
    out.println(line);
  }

  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    if (slowCanWindow_.txPerSubnet[subnet] == 0 &&
        slowCanWindow_.rxPerSubnet[subnet] == 0 &&
        slowCanWindow_.subnetBuckets[subnet].over500Count == 0 &&
        slowCanWindow_.subnetBuckets[subnet].over1000Count == 0) {
      continue;
    }
    const SlowSubnetBucket& bucket = slowCanWindow_.subnetBuckets[subnet];
    snprintf(line, sizeof(line),
             "SUBNET_LATENCY subnet=%u tx=%lu rx=%lu slow500=%lu slow1000=%lu",
             static_cast<unsigned>(subnet),
             static_cast<unsigned long>(slowCanWindow_.txPerSubnet[subnet]),
             static_cast<unsigned long>(slowCanWindow_.rxPerSubnet[subnet]),
             static_cast<unsigned long>(bucket.over500Count),
             static_cast<unsigned long>(bucket.over1000Count));
    out.println(line);
  }

  uint32_t totalNormalReadTx10s = 0;
  uint32_t worstNormalReadSubnetTx10s = 0;
  uint8_t worstNormalReadSubnet = 0xFFu;
  uint8_t activeNormalReadSubnets = 0;
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    const uint32_t subnetTx = slowCanWindow_.normalReadTxPerSubnet[subnet];
    totalNormalReadTx10s += subnetTx;
    if (subnetTx != 0) {
      ++activeNormalReadSubnets;
    }
    if (subnetTx > worstNormalReadSubnetTx10s) {
      worstNormalReadSubnetTx10s = subnetTx;
      worstNormalReadSubnet = subnet;
    }
  }
  const uint32_t worstNormalReadSubnetSharePct =
      totalNormalReadTx10s != 0
          ? static_cast<uint32_t>((static_cast<uint64_t>(worstNormalReadSubnetTx10s) * 100u) /
                                  static_cast<uint64_t>(totalNormalReadTx10s))
          : 0u;
  out.print("SUBNET_FAIRNESS_SUMMARY fairness_enforced=0");
  out.print(" fairness_limit_hit_total=");
  out.print(counters_.subnet_fairness_limit_hit_total);
  out.print(" fairness_switch_total=");
  out.print(counters_.subnet_fairness_switch_total);
  out.print(" fairness_no_alternate_total=");
  out.print(counters_.subnet_fairness_no_alternate_total);
  out.print(" fairness_enforced=0");
  out.print(" normal_read_tx10s=");
  out.print(totalNormalReadTx10s);
  out.print(" active_subnets=");
  out.print(activeNormalReadSubnets);
  out.print(" worst_subnet=");
  if (worstNormalReadSubnet == 0xFFu) {
    out.print("none");
  } else {
    out.print(worstNormalReadSubnet);
  }
  out.print(" worst_subnet_tx10s=");
  out.print(worstNormalReadSubnetTx10s);
  out.print(" worst_subnet_share_pct=");
  out.println(worstNormalReadSubnetSharePct);
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    const uint32_t subnetTx = slowCanWindow_.normalReadTxPerSubnet[subnet];
    if (subnetTx == 0 && slowCanWindow_.subnetReplyGapMaxMs[subnet] == 0 &&
        scheduler_.consecutiveNormalReadsForSubnet(subnet) == 0) {
      continue;
    }
    const uint32_t subnetSharePct =
        totalNormalReadTx10s != 0
            ? static_cast<uint32_t>((static_cast<uint64_t>(subnetTx) * 100u) /
                                    static_cast<uint64_t>(totalNormalReadTx10s))
            : 0u;
    snprintf(line, sizeof(line),
             "SUBNET_FAIRNESS_SUMMARY subnet=%u subnet_tx10s=%lu subnet_share_pct=%lu consecutive_reads_per_subnet=%u worst_subnet_reply_gap_ms=%lu boosted_subnet=0",
             static_cast<unsigned>(subnet),
             static_cast<unsigned long>(subnetTx),
             static_cast<unsigned long>(subnetSharePct),
             static_cast<unsigned>(scheduler_.consecutiveNormalReadsForSubnet(subnet)),
             static_cast<unsigned long>(slowCanWindow_.subnetReplyGapMaxMs[subnet]));
    out.println(line);
  }

  if (slowCanWindow_.worstDeviceReplyGapMs != 0 || slowCanWindow_.missedServiceWindowCount != 0) {
    out.print("SUBNET_SERVICE_CADENCE missed_service_window_count=");
    out.print(slowCanWindow_.missedServiceWindowCount);
    out.print(" worst_device_reply_gap_ms=");
    out.print(slowCanWindow_.worstDeviceReplyGapMs);
    out.print(" worst_device=");
    if (slowCanWindow_.worstDeviceReplyGapMs == 0) {
      out.println("none");
    } else {
      out.print(slowCanWindow_.worstDeviceReplyGapSubnet);
      out.print(".");
      if (slowCanWindow_.worstDeviceReplyGapDevice < 10u) {
        out.print("0");
      }
      out.println(slowCanWindow_.worstDeviceReplyGapDevice);
    }
  }
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    if (slowCanWindow_.subnetReplyGapMaxMs[subnet] == 0) {
      continue;
    }
    snprintf(line, sizeof(line),
             "SUBNET_SERVICE_CADENCE subnet=%u subnet_reply_gap_max_ms=%lu",
             static_cast<unsigned>(subnet),
             static_cast<unsigned long>(slowCanWindow_.subnetReplyGapMaxMs[subnet]));
    out.println(line);
  }

  snprintf(line, sizeof(line),
           "WORST_QUEUE_WAIT max=%u WORST_CAN_ROUNDTRIP max=%u WORST_POST_REPLY max=%u",
           static_cast<unsigned>(slowCanWindow_.worstQueueWaitMs),
           static_cast<unsigned>(slowCanWindow_.worstCanRoundtripMs),
           static_cast<unsigned>(slowCanWindow_.worstPostReplyMs));
  out.println(line);

  snprintf(line, sizeof(line),
           "STARVATION node_gap_max=%lu object_gap_max=%lu node_consec_max=%u object_consec_max=%u",
           static_cast<unsigned long>(slowCanWindow_.maxNodeStarvationGapMs),
           static_cast<unsigned long>(slowCanWindow_.maxObjectStarvationGapMs),
           static_cast<unsigned>(slowCanWindow_.maxConsecutiveNodeRequests),
           static_cast<unsigned>(slowCanWindow_.maxConsecutiveObjectRequests));
  out.println(line);

  snprintf(line, sizeof(line),
           "IDENT_IMPACT ident_tx_10s=%lu slow500_after_ident=%lu slow1000_after_ident=%lu",
           static_cast<unsigned long>(slowCanWindow_.identTxCount),
           static_cast<unsigned long>(slowCanWindow_.identSlowOver500Count),
           static_cast<unsigned long>(slowCanWindow_.identSlowOver1000Count));
  out.println(line);

  uint32_t expensiveCanTxTotal = 0;
  for (uint8_t index = 0; index < kExpensiveObjectGroupCount; ++index) {
    const ExpensiveObjectWindow& bucket = slowCanWindow_.expensiveGroups[index];
    expensiveCanTxTotal += bucket.canTxCount;
    if (bucket.group == ExpensiveObjectGroup::None ||
        (bucket.tcpReadCount == 0 && bucket.canTxCount == 0 && bucket.canRxCount == 0 &&
         bucket.duplicateInflightCount == 0 && bucket.coalescedCount == 0 && bucket.burstRepeatCount == 0 &&
         bucket.segmentedActiveRepeatCount == 0)) {
      continue;
    }
    snprintf(line, sizeof(line),
             "EXPENSIVE_OBJECT group=%s req10s=%lu can_tx10s=%lu can_rx10s=%lu burst_peak100ms=%u burst_peak1s=%u burst_repeat=%lu dup_inflight=%lu dup_pending_max=%u seg_active_repeat=%lu coalesced=%lu slow500=%lu slow1000=%lu max_can_rt=%u",
             expensiveObjectGroupName(bucket.group),
             static_cast<unsigned long>(bucket.tcpReadCount),
             static_cast<unsigned long>(bucket.canTxCount),
             static_cast<unsigned long>(bucket.canRxCount),
             static_cast<unsigned>(bucket.peakRequestPer100ms),
             static_cast<unsigned>(bucket.peakRequestPer1s),
             static_cast<unsigned long>(bucket.burstRepeatCount),
             static_cast<unsigned long>(bucket.duplicateInflightCount),
             static_cast<unsigned>(bucket.duplicatePendingMax),
             static_cast<unsigned long>(bucket.segmentedActiveRepeatCount),
             static_cast<unsigned long>(bucket.coalescedCount),
             static_cast<unsigned long>(bucket.over500Count),
             static_cast<unsigned long>(bucket.over1000Count),
             static_cast<unsigned>(bucket.maxCanRoundtripMs));
    out.println(line);
  }

  const uint32_t collapseSavedPercent =
      (slowCanWindow_.collapseSavedCanTx + expensiveCanTxTotal) != 0
          ? static_cast<uint32_t>((static_cast<uint64_t>(slowCanWindow_.collapseSavedCanTx) * 100u) /
                                  static_cast<uint64_t>(slowCanWindow_.collapseSavedCanTx + expensiveCanTxTotal))
          : 0u;
  snprintf(line, sizeof(line),
           "SOFT_COLLAPSE_SUMMARY fanout_attach=%s segmented_dampening=%s duplicate_inflight_10s=%lu burst_repeat_10s=%lu segmented_repeat_10s=%lu inflight_collapsed=%lu inflight_waiters_max=%u inflight_timeout_release=%lu collapse_saved_can_tx=%lu collapse_saved_percent=%lu",
           fanoutAttachFeatureEnabled() ? "on" : "off",
           segmentedDampeningFeatureEnabled() ? "on" : "off",
           static_cast<unsigned long>(slowCanWindow_.duplicateInflightReadCount),
           static_cast<unsigned long>(slowCanWindow_.burstRepeatReadCount),
           static_cast<unsigned long>(slowCanWindow_.segmentedActiveRepeatCount),
           static_cast<unsigned long>(slowCanWindow_.inflightCollapsedCount),
           static_cast<unsigned>(slowCanWindow_.inflightWaitersMax),
           static_cast<unsigned long>(slowCanWindow_.inflightTimeoutReleaseCount),
           static_cast<unsigned long>(slowCanWindow_.collapseSavedCanTx),
           static_cast<unsigned long>(collapseSavedPercent));
  out.println(line);

  const uint32_t shapingExtraWaitAvgMs =
      slowCanWindow_.shapingExtraWaitCount != 0
          ? static_cast<uint32_t>(slowCanWindow_.shapingExtraWaitSumMs / slowCanWindow_.shapingExtraWaitCount)
          : 0u;
  const uint32_t shapingAddedWaitAvgMs =
      slowCanWindow_.shapingAddedWaitCount != 0
          ? static_cast<uint32_t>(slowCanWindow_.shapingAddedWaitSumMs / slowCanWindow_.shapingAddedWaitCount)
          : 0u;
  snprintf(line, sizeof(line),
           "SHAPING_ACTIVE_SUMMARY shaping_enabled=%u shaped_requests_10s=%lu attached_to_recent_10s=%lu deferred_requests_10s=%lu reused_recent_response_10s=%lu shape_saved_can_tx_10s=%lu shape_bypass_due_to_age_10s=%lu avg_added_wait_ms=%lu max_added_wait_ms=%u",
           static_cast<unsigned>(gentleStatusShapingEnabled() ? 1u : 0u),
           static_cast<unsigned long>(slowCanWindow_.shapingShapedRequestsCount),
           static_cast<unsigned long>(slowCanWindow_.shapingAttachedToInflightCount),
           static_cast<unsigned long>(slowCanWindow_.shapedDeferredCount),
           static_cast<unsigned long>(slowCanWindow_.shapingReusedRecentResponseCount),
           static_cast<unsigned long>(slowCanWindow_.shapingSavedCanTx),
           static_cast<unsigned long>(slowCanWindow_.shapedBypassDueAgeCount),
           static_cast<unsigned long>(shapingAddedWaitAvgMs),
           static_cast<unsigned>(slowCanWindow_.shapingAddedWaitMaxMs));
  out.println(line);
  snprintf(line, sizeof(line),
           "GENTLE_SHAPING_SUMMARY shaping_enabled=%s shaping_candidates_10s=%lu shaping_allowed_immediate_10s=%lu shaped_deferred_10s=%lu shaping_release_sent_10s=%lu shaping_bypass_age_10s=%lu shaping_saved_can_tx_10s=%lu shaping_extra_wait_avg_ms=%lu shaping_extra_wait_max_ms=%u bus_util_avg=%lu bus_util_peak100ms=%lu over500_10s=%lu over1000_10s=%lu r1_missing_data_events=%lu shaping_disabled_reason=%s",
           gentleStatusShapingEnabled() ? "on" : "off",
           static_cast<unsigned long>(slowCanWindow_.shapingCandidatesCount),
           static_cast<unsigned long>(slowCanWindow_.shapingAllowedImmediateCount),
           static_cast<unsigned long>(slowCanWindow_.shapedDeferredCount),
           static_cast<unsigned long>(slowCanWindow_.shapingReleaseSentCount),
           static_cast<unsigned long>(slowCanWindow_.shapedBypassDueAgeCount),
           static_cast<unsigned long>(slowCanWindow_.shapingSavedCanTx),
           static_cast<unsigned long>(shapingExtraWaitAvgMs),
           static_cast<unsigned>(slowCanWindow_.shapingExtraWaitMaxMs),
           static_cast<unsigned long>(avgBusUtilPercent),
           static_cast<unsigned long>(peakBusUtilPercent100ms),
           static_cast<unsigned long>(slowCanWindow_.over500Count),
           static_cast<unsigned long>(slowCanWindow_.over1000Count),
           static_cast<unsigned long>(slowCanWindow_.r1MissingDataEvents),
           shapingDisabledReasonName(slowCanWindow_.shapingDisabledReasonCode));
  out.println(line);
  const uint32_t segmentedGuardWaitAvgMs =
      slowCanWindow_.segmentedGuardWaitCount != 0
          ? static_cast<uint32_t>(slowCanWindow_.segmentedGuardWaitSumMs / slowCanWindow_.segmentedGuardWaitCount)
          : 0u;
  snprintf(line, sizeof(line),
           "SEGMENTED_GUARD_SUMMARY enabled=%s candidates_10s=%lu deferred_10s=%lu released_10s=%lu forced_release_10s=%lu coalesced_10s=%lu immediate_10s=%lu blocked_queue_10s=%lu blocked_util_10s=%lu blocked_slow500_10s=%lu pending_max=%u avg_wait_ms=%lu max_wait_ms=%u",
           segmentedDampeningFeatureEnabled() ? "on" : "off",
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardCandidates),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardDeferred),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardReleased),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardForcedRelease),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardCoalesced),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardImmediate),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardBlockedByQueue),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardBlockedByUtil),
           static_cast<unsigned long>(slowCanWindow_.segmentedGuardBlockedBySlow500),
           static_cast<unsigned>(slowCanWindow_.segmentedGuardPendingMax),
           static_cast<unsigned long>(segmentedGuardWaitAvgMs),
           static_cast<unsigned>(slowCanWindow_.segmentedGuardWaitMaxMs));
  out.println(line);
  const uint32_t identGuardWaitAvgMs =
      slowCanWindow_.identGuardWaitCount != 0
          ? static_cast<uint32_t>(slowCanWindow_.identGuardWaitSumMs / slowCanWindow_.identGuardWaitCount)
          : 0u;
  snprintf(line, sizeof(line),
           "IDENT_GUARD_SUMMARY candidates_10s=%lu deferred_10s=%lu released_10s=%lu forced_release_10s=%lu coalesced_10s=%lu immediate_10s=%lu blocked_queue_10s=%lu blocked_util_10s=%lu blocked_slow500_10s=%lu pending_max=%u avg_wait_ms=%lu max_wait_ms=%u",
           static_cast<unsigned long>(slowCanWindow_.identGuardCandidates),
           static_cast<unsigned long>(slowCanWindow_.identGuardDeferred),
           static_cast<unsigned long>(slowCanWindow_.identGuardReleased),
           static_cast<unsigned long>(slowCanWindow_.identGuardForcedRelease),
           static_cast<unsigned long>(slowCanWindow_.identGuardCoalesced),
           static_cast<unsigned long>(slowCanWindow_.identGuardImmediate),
           static_cast<unsigned long>(slowCanWindow_.identGuardBlockedByQueue),
           static_cast<unsigned long>(slowCanWindow_.identGuardBlockedByUtil),
           static_cast<unsigned long>(slowCanWindow_.identGuardBlockedBySlow500),
           static_cast<unsigned>(slowCanWindow_.identGuardPendingMax),
           static_cast<unsigned long>(identGuardWaitAvgMs),
           static_cast<unsigned>(slowCanWindow_.identGuardWaitMaxMs));
  out.println(line);
  if (gentleStatusShapingEnabled() && slowCanWindow_.shapingCandidatesCount == 0) {
    out.println("SHAPING_NOT_HIT eligible object classifier mismatch");
  }

  for (uint8_t index = 0; index < kShapingSimGroupCount; ++index) {
    const ShapingSimGroupWindow& bucket = slowCanWindow_.shapingGroups[index];
    if (bucket.group == ShapingSimGroup::None || bucket.actualCanTxCount == 0) {
      continue;
    }
    const uint32_t simulatedSavedPercent =
        bucket.actualCanTxCount != 0
            ? static_cast<uint32_t>((static_cast<uint64_t>(bucket.simulatedSavedTx) * 100u) /
                                    static_cast<uint64_t>(bucket.actualCanTxCount))
            : 0u;
    snprintf(line, sizeof(line),
             "SHAPING_SIM_GROUP group=%s actual_can_tx10s=%lu simulated_can_tx10s=%lu simulated_saved_tx=%lu simulated_saved_percent=%lu affected_nodes=%u max_extra_wait_ms=%u",
             shapingSimGroupName(bucket.group),
             static_cast<unsigned long>(bucket.actualCanTxCount),
             static_cast<unsigned long>(bucket.simulatedCanTxCount),
             static_cast<unsigned long>(bucket.simulatedSavedTx),
             static_cast<unsigned long>(simulatedSavedPercent),
             static_cast<unsigned>(bucket.affectedNodeCount),
             static_cast<unsigned>(bucket.maxExtraWaitMs));
    out.println(line);
    if (isGentleStatusShapingGroup(bucket.group) ||
        bucket.shapedSavedTx != 0 ||
        bucket.softCollapseSavedTx != 0 ||
        bucket.shapedRequests != 0 ||
        bucket.attachedToRecentCount != 0 ||
        bucket.deferredRequests != 0 ||
        bucket.reusedRecentResponseCount != 0) {
      const uint32_t actualSaved = bucket.shapedSavedTx + bucket.softCollapseSavedTx;
      const uint32_t actualSavedPercent =
          (bucket.actualCanTxCount + actualSaved) != 0
              ? static_cast<uint32_t>((static_cast<uint64_t>(actualSaved) * 100u) /
                                      static_cast<uint64_t>(bucket.actualCanTxCount + actualSaved))
              : 0u;
      const uint32_t addedWaitAvgMs =
          bucket.shapedDispatchCount != 0
              ? static_cast<uint32_t>(bucket.shapedDispatchAddedWaitSumMs / bucket.shapedDispatchCount)
              : 0u;
      snprintf(line, sizeof(line),
               "SHAPING_ACTIVE_GROUP group=%s enabled=%u shaped_requests_10s=%lu attached_to_recent_10s=%lu deferred_requests_10s=%lu reused_recent_response_10s=%lu shaped_saved_can_tx_10s=%lu collapse_saved_can_tx_10s=%lu actual_saved_can_tx_10s=%lu actual_saved_percent=%lu avg_shape_wait_ms=%lu max_shape_wait_ms=%u bypass_due_to_age_10s=%lu bypass_first_request_10s=%lu bypass_cache_ready_10s=%lu bypass_disabled_10s=%lu bypass_ineligible_10s=%lu bypass_no_cache_10s=%lu bypass_cache_too_old_10s=%lu bypass_no_inflight_10s=%lu bypass_release_forced_10s=%lu bypass_write_or_control_10s=%lu simulated_saved_tx=%lu",
               shapingSimGroupName(bucket.group),
               static_cast<unsigned>(isGentleStatusShapingGroup(bucket.group) ? 1u : 0u),
               static_cast<unsigned long>(bucket.shapedRequests),
               static_cast<unsigned long>(bucket.attachedToRecentCount),
               static_cast<unsigned long>(bucket.deferredRequests),
               static_cast<unsigned long>(bucket.reusedRecentResponseCount),
               static_cast<unsigned long>(bucket.shapedSavedTx),
               static_cast<unsigned long>(bucket.softCollapseSavedTx),
               static_cast<unsigned long>(actualSaved),
               static_cast<unsigned long>(actualSavedPercent),
               static_cast<unsigned long>(addedWaitAvgMs),
               static_cast<unsigned>(bucket.shapedDispatchAddedWaitMaxMs),
               static_cast<unsigned long>(bucket.bypassDueToAgeCount),
               static_cast<unsigned long>(bucket.bypassFirstRequestCount),
               static_cast<unsigned long>(bucket.bypassCacheReadyCount),
               static_cast<unsigned long>(bucket.bypassDisabledCount),
               static_cast<unsigned long>(bucket.bypassIneligibleCount),
               static_cast<unsigned long>(bucket.bypassNoCacheCount),
               static_cast<unsigned long>(bucket.bypassCacheTooOldCount),
               static_cast<unsigned long>(bucket.bypassNoInflightCount),
               static_cast<unsigned long>(bucket.bypassReleaseForcedCount),
               static_cast<unsigned long>(bucket.bypassWriteOrControlCount),
               static_cast<unsigned long>(bucket.simulatedSavedTx));
      out.println(line);
    }
  }

  for (uint8_t slot = 0; slot < 5; ++slot) {
    if (topObjects[slot] == 0) {
      continue;
    }
    snprintf(line, sizeof(line),
             "OBJECT_LATENCY object=%04X lt100=%lu 100_250=%lu 250_500=%lu 500_1000=%lu gt1000=%lu",
             static_cast<unsigned>(topObjects[slot]->objectKey),
             static_cast<unsigned long>(topObjects[slot]->lt100Count),
             static_cast<unsigned long>(topObjects[slot]->bucket100To250Count),
             static_cast<unsigned long>(topObjects[slot]->bucket250To500Count),
             static_cast<unsigned long>(topObjects[slot]->bucket500To1000Count),
             static_cast<unsigned long>(topObjects[slot]->gt1000Count));
    out.println(line);
  }

  resetSlowCanWindow(nowMs);
}

void R70GatewayV2::rebuildWatchdogState(uint32_t nowMs) {
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* activeAmp = state_.activeAmpByIndex(ampIndex);
    if (activeAmp == 0) {
      continue;
    }
    AmpState* ampState = state_.amp(activeAmp->subnet, activeAmp->device, false);
    if (ampState == 0) {
      continue;
    }
    ampState->maxRequestToTcpTxLatencyMs = 0;
    ampState->pendingFieldCount = 0;
    ampState->oldestPendingFieldAgeMs = 0;
    ampState->unknown0111Pending = 0;
    ampState->unknown0111MaxLatencyMs = 0;
  }

  for (uint16_t index = 0; index < kRecentRequestTraceCapacity; ++index) {
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used || trace.device == 0) {
      continue;
    }
    AmpState* ampState = state_.amp(trace.subnet, trace.device, false);
    if (ampState == 0) {
      continue;
    }
    const uint32_t ageMs = nowMs - trace.timestampMs;
    if ((trace.flags & kTraceFlagTcpTxSent) == 0) {
      ++ampState->pendingFieldCount;
      if (ageMs > ampState->oldestPendingFieldAgeMs) {
        ampState->oldestPendingFieldAgeMs = ageMs;
      }
      if (trace.objectKey == 0x0111) {
        ++ampState->unknown0111Pending;
      }
    }
    if (ageMs <= kSlowFieldLookbackMs && (trace.flags & kTraceFlagTcpTxSent) != 0) {
      if (trace.latencyMs > ampState->maxRequestToTcpTxLatencyMs) {
        ampState->maxRequestToTcpTxLatencyMs = trace.latencyMs;
      }
      if (trace.objectKey == 0x0111 && trace.latencyMs > ampState->unknown0111MaxLatencyMs) {
        ampState->unknown0111MaxLatencyMs = trace.latencyMs;
      }
    }
  }
}

void R70GatewayV2::logTimeoutCorrelation(uint8_t subnet,
                                         uint8_t device,
                                         uint32_t nowMs,
                                         const char* prefix) const {
  char line[256] = {};
  snprintf(line, sizeof(line), "T=%lums %s node=%u.%02u now_ms=%lu",
           static_cast<unsigned long>(nowMs),
           prefix != 0 ? prefix : "TIMEOUT_CORRELATION",
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned long>(nowMs));
  debugCli_.logText(line);
  logAliveCandidateCorrelation(subnet, device, nowMs);
  logHealthyPeerComparison(subnet, device, nowMs);

  const AmpState* ampState = state_.amp(subnet, device);
  if (ampState == 0) {
    snprintf(line, sizeof(line), "T=%lums node=%u.%02u state=not_active",
             static_cast<unsigned long>(nowMs),
             static_cast<unsigned>(subnet),
             static_cast<unsigned>(device));
    debugCli_.logText(line);
    return;
  }

  const uint32_t lastTcpTxAgeMs = ampState->lastTcpTxToR1Ms == 0 ? 0 : (nowMs - ampState->lastTcpTxToR1Ms);
  const uint32_t lastRealCanRxAgeMs = ampState->lastCanRxMs == 0 ? 0 : (nowMs - ampState->lastCanRxMs);
  const uint32_t lastCacheReplyAgeMs = ampState->lastCacheReplyMs == 0 ? 0 : (nowMs - ampState->lastCacheReplyMs);
  const uint32_t lastCanTxPollAgeMs = ampState->lastCanTxPollMs == 0 ? 0 : (nowMs - ampState->lastCanTxPollMs);
  snprintf(line, sizeof(line),
           "T=%lums node=%u.%02u last_tcp_tx_age_ms=%lu last_real_can_rx_age_ms=%lu last_cache_reply_age_ms=%lu last_can_tx_poll_age_ms=%lu oldest_pending_age_ms=%lu pending_count=%u max_req_to_tx_latency_ms=%lu queue_depth_now=%lu normal_depth_now=%lu",
           static_cast<unsigned long>(nowMs),
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned long>(lastTcpTxAgeMs),
           static_cast<unsigned long>(lastRealCanRxAgeMs),
           static_cast<unsigned long>(lastCacheReplyAgeMs),
           static_cast<unsigned long>(lastCanTxPollAgeMs),
           static_cast<unsigned long>(ampState->oldestPendingFieldAgeMs),
           static_cast<unsigned>(ampState->pendingFieldCount),
           static_cast<unsigned long>(ampState->maxRequestToTcpTxLatencyMs),
           static_cast<unsigned long>(counters_.can_tx_queue_depth),
           static_cast<unsigned long>(counters_.normal_read_queue_depth));
  debugCli_.logText(line);

  uint16_t pendingCount = 0;
  uint32_t oldestPendingAgeMs = 0;
  uint16_t canAlreadyComplete = 0;
  uint16_t waitingForSegment = 0;
  uint16_t waitingForFanout = 0;
  uint16_t waitingForTcpFlush = 0;
  const char* missingFamily = "none";

  uint32_t maxNormalDepth10s = 0;
  uint32_t maxCanDepth10s = 0;
  struct SlowField {
    uint16_t objectKey;
    FieldId fieldId;
    uint16_t latencyMs;
  };
  SlowField slow[8] = {};
  uint8_t printedRequests = 0;

  for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
    const uint16_t index =
        static_cast<uint16_t>((recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) %
                              kRecentRequestTraceCapacity);
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used || trace.subnet != subnet || trace.device != device) {
      continue;
    }
    if ((trace.flags & kTraceFlagTcpTxSent) == 0 &&
        trace.pendingState != TracePendingState::TimeoutCleanup) {
      ++pendingCount;
      const uint32_t pendingAgeMs = nowMs >= trace.timestampMs ? nowMs - trace.timestampMs : 0u;
      if (pendingAgeMs > oldestPendingAgeMs) {
        oldestPendingAgeMs = pendingAgeMs;
        NormalizedRequest pendingRequest = {};
        pendingRequest.valid = true;
        pendingRequest.canId.subnet = trace.subnet;
        pendingRequest.canId.device = trace.device;
        pendingRequest.objectAddress.channelNumber = trace.channel;
        pendingRequest.objectAddress.boxNumber = static_cast<uint8_t>((trace.objectKey >> 8) & 0xFFu);
        pendingRequest.objectAddress.objectNumber = static_cast<uint8_t>(trace.objectKey & 0xFFu);
        pendingRequest.frame.length = 3;
        pendingRequest.frame.data[1] = pendingRequest.objectAddress.boxNumber;
        pendingRequest.frame.data[2] = pendingRequest.objectAddress.objectNumber;
        missingFamily = semanticMirrorGroupName(classifySemanticMirrorGroup(pendingRequest));
      }
      if ((trace.flags & kTraceFlagRealCanRxSeen) != 0) {
        ++canAlreadyComplete;
        ++waitingForTcpFlush;
      }
      if (trace.pendingState == TracePendingState::PartialComplete) {
        ++waitingForSegment;
      }
      if (trace.waiterFanoutCount != 0) {
        ++waitingForFanout;
      }
    }
    const uint32_t ageMs = nowMs - trace.timestampMs;
    if (ageMs <= kTimeoutTraceLookbackMs && printedRequests < 20) {
      const char* sourceText = traceSourceName(trace.source);
      const char* label = trace.fieldId != FieldId::Unknown ? fieldIdName(trace.fieldId) :
          (trace.objectKey == 0x0111 ? "unknown_0111" : "unknown");
      snprintf(line, sizeof(line),
               "T=%lums req field=%s object=%04X req_age_ms=%lu source=%s mirror=%s bg_refresh=%s latency_ms=%u can_tx=%u cache=%u can_rx=%u tcp_tx=%u stale=%u pending_tcp=%u queue_depth=%u can_tx_queue_depth=%u",
               static_cast<unsigned long>(nowMs),
               label,
               static_cast<unsigned>(trace.objectKey),
               static_cast<unsigned long>(ageMs),
               sourceText,
               traceMirrorDecisionName(trace.mirrorDecision),
               traceBackgroundRefreshStateName(trace.backgroundRefreshState),
               static_cast<unsigned>(trace.latencyMs),
               (trace.flags & kTraceFlagForwardedToCan) != 0 ? 1u : 0u,
               (trace.flags & kTraceFlagCacheReplied) != 0 ? 1u : 0u,
               (trace.flags & kTraceFlagRealCanRxSeen) != 0 ? 1u : 0u,
               (trace.flags & kTraceFlagTcpTxSent) != 0 ? 1u : 0u,
               (trace.flags & kTraceFlagStaleAtRequest) != 0 ? 1u : 0u,
               static_cast<unsigned>(trace.pendingTcpRepliesAtRequest),
               static_cast<unsigned>(trace.normalQueueDepthAtRequest),
               static_cast<unsigned>(trace.canTxQueueDepthAtRequest));
      debugCli_.logText(line);
      ++printedRequests;
    }
    if (ageMs <= kSlowFieldLookbackMs) {
      if (trace.normalQueueDepthAtRequest > maxNormalDepth10s) {
        maxNormalDepth10s = trace.normalQueueDepthAtRequest;
      }
      if (trace.canTxQueueDepthAtRequest > maxCanDepth10s) {
        maxCanDepth10s = trace.canTxQueueDepthAtRequest;
      }
      if ((trace.flags & kTraceFlagTcpTxSent) != 0) {
        for (uint8_t slot = 0; slot < 8; ++slot) {
          if (slow[slot].objectKey == trace.objectKey && slow[slot].fieldId == trace.fieldId) {
            if (trace.latencyMs > slow[slot].latencyMs) {
              slow[slot].latencyMs = trace.latencyMs;
            }
            break;
          }
          if (slow[slot].objectKey == 0 || trace.latencyMs > slow[slot].latencyMs) {
            for (int8_t shift = 7; shift > static_cast<int8_t>(slot); --shift) {
              slow[shift] = slow[shift - 1];
            }
            slow[slot].objectKey = trace.objectKey;
            slow[slot].fieldId = trace.fieldId;
            slow[slot].latencyMs = trace.latencyMs;
            break;
          }
        }
      }
    }
  }

  snprintf(line, sizeof(line),
           "T=%lums R1_TIMEOUT_PENDING_CORRELATION node=%u.%02u pending_request_count=%u oldest_pending_age_ms=%lu can_already_complete=%u missing_object_family=%s waiting_for_segment=%u waiting_for_fanout=%u waiting_for_tcp_flush=%u",
           static_cast<unsigned long>(nowMs),
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned>(pendingCount),
           static_cast<unsigned long>(oldestPendingAgeMs),
           static_cast<unsigned>(canAlreadyComplete),
           missingFamily,
           static_cast<unsigned>(waitingForSegment),
           static_cast<unsigned>(waitingForFanout),
           static_cast<unsigned>(waitingForTcpFlush));
  debugCli_.logText(line);

  uint32_t protectedSemanticAgeMs = 0;
  uint32_t lastRefreshMsAgo = 0;
  uint32_t staleServedCount10s = 0;
  const char* refreshBlockedReason = "none";
  for (uint16_t mirrorIndex = 0; mirrorIndex < kSemanticMirrorCapacity; ++mirrorIndex) {
    const SemanticMirrorEntry& entry = semanticMirror_[mirrorIndex];
    if (!entry.used || !entry.learned || entry.subnet != subnet || entry.device != device ||
        entry.lastUpdateMs == 0 || entry.lastUpdateMs > nowMs) {
      continue;
    }
    NormalizedRequest mirrorRequest = {};
    mirrorRequest.valid = true;
    mirrorRequest.canId.subnet = entry.subnet;
    mirrorRequest.canId.device = entry.device;
    mirrorRequest.objectAddress.channelNumber = entry.channel;
    mirrorRequest.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
    mirrorRequest.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
    mirrorRequest.fieldId = entry.fieldId;
    mirrorRequest.admin.valid = true;
    mirrorRequest.admin.telegramType = entry.telegramType;
    const SemanticMirrorGroup group = classifySemanticMirrorGroup(mirrorRequest);
    if (!isProtectedSemanticMirrorGroup(group)) {
      continue;
    }
    const uint32_t ageMs = nowMs - entry.lastUpdateMs;
    if (ageMs > protectedSemanticAgeMs) {
      protectedSemanticAgeMs = ageMs;
      lastRefreshMsAgo = ageMs;
      refreshBlockedReason = semanticRefreshBlockedReasonName(entry.lastRefreshBlockedReason);
    }
    staleServedCount10s += entry.staleServedCountWindow;
  }
  snprintf(line, sizeof(line),
           "T=%lums DEVICE_TIMEOUT_CORRELATION node=%u.%02u semantic_age_ms=%lu last_refresh_ms_ago=%lu stale_served_count_10s=%lu refresh_blocked_reason=%s",
           static_cast<unsigned long>(nowMs),
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned long>(protectedSemanticAgeMs),
           static_cast<unsigned long>(lastRefreshMsAgo),
           static_cast<unsigned long>(staleServedCount10s),
           refreshBlockedReason);
  debugCli_.logText(line);
  logNodeSemanticEpoch(subnet, device, nowMs, "R1_TIMEOUT_SEMANTIC_STATE");
  logNodeSemanticInconsistency(subnet, device, nowMs, "SEMANTIC_INCONSISTENCY");

  snprintf(line, sizeof(line),
           "T=%lums node=%u.%02u last_0111_req_age_ms=%lu last_0111_rx_age_ms=%lu last_0111_tx_age_ms=%lu unknown0111_pending=%u unknown0111_max_latency_ms=%lu last_value=%02X max_normal_depth_10s=%lu max_can_tx_depth_10s=%lu",
           static_cast<unsigned long>(nowMs),
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           ampState->lastUnknown0111RequestMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - ampState->lastUnknown0111RequestMs),
           ampState->lastUnknown0111RxMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - ampState->lastUnknown0111RxMs),
           ampState->lastUnknown0111TcpTxMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - ampState->lastUnknown0111TcpTxMs),
           static_cast<unsigned>(ampState->unknown0111Pending),
           static_cast<unsigned long>(ampState->unknown0111MaxLatencyMs),
           static_cast<unsigned>(ampState->unknown0111LastValue),
           static_cast<unsigned long>(maxNormalDepth10s),
           static_cast<unsigned long>(maxCanDepth10s));
  debugCli_.logText(line);

  snprintf(line, sizeof(line),
           "T=%lums UNKNOWN_0111_SUMMARY node=%u.%02u req=%lu can_tx=%lu can_rx=%lu tcp_tx=%lu pending=%u max_latency_ms=%lu last_value=%02X",
           static_cast<unsigned long>(nowMs),
           static_cast<unsigned>(ampState->subnet),
           static_cast<unsigned>(ampState->device),
           static_cast<unsigned long>(ampState->unknown0111Requests),
           static_cast<unsigned long>(ampState->unknown0111CanTx),
           static_cast<unsigned long>(ampState->unknown0111CanRx),
           static_cast<unsigned long>(ampState->unknown0111TcpTx),
           static_cast<unsigned>(ampState->unknown0111Pending),
           static_cast<unsigned long>(ampState->unknown0111MaxLatencyMs),
           static_cast<unsigned>(ampState->unknown0111LastValue));
  debugCli_.logText(line);

  for (uint8_t slot = 0; slot < 8; ++slot) {
    if (slow[slot].objectKey == 0) {
      continue;
    }
    const char* label = slow[slot].fieldId != FieldId::Unknown ? fieldIdName(slow[slot].fieldId) :
        (slow[slot].objectKey == 0x0111 ? "unknown_0111" : "unknown");
    snprintf(line, sizeof(line),
             "T=%lums slow_field rank=%u field=%s object=%04X latency_ms=%u",
             static_cast<unsigned long>(nowMs),
             static_cast<unsigned>(slot + 1),
             label,
             static_cast<unsigned>(slow[slot].objectKey),
             static_cast<unsigned>(slow[slot].latencyMs));
    debugCli_.logText(line);
  }
}

void R70GatewayV2::logHealthyPeerComparison(uint8_t timeoutSubnet,
                                            uint8_t timeoutDevice,
                                            uint32_t nowMs) const {
  const AmpState* peer = 0;
  uint32_t bestReplyAgeMs = 0xFFFFFFFFu;

  // Prefer the most recently responsive amplifier in the same subnet. This
  // gives each timeout snapshot a healthy control observed under the same bus
  // and R1 polling conditions without adding any CAN traffic.
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* candidate = state_.activeAmpByIndex(ampIndex);
    if (candidate == 0 || candidate->subnet != timeoutSubnet ||
        candidate->device == timeoutDevice || candidate->lastTcpTxToR1Ms == 0 ||
        candidate->lastCanRxMs == 0) {
      continue;
    }
    const uint32_t replyAgeMs = nowMs - candidate->lastTcpTxToR1Ms;
    if (peer == 0 || replyAgeMs < bestReplyAgeMs) {
      peer = candidate;
      bestReplyAgeMs = replyAgeMs;
    }
  }

  char line[256] = {};
  if (peer == 0) {
    snprintf(line, sizeof(line),
             "HEALTHY_PEER_COMPARISON timeout_node=%u.%02u peer=unavailable",
             static_cast<unsigned>(timeoutSubnet),
             static_cast<unsigned>(timeoutDevice));
    debugCli_.logText(line);
    return;
  }

  snprintf(line, sizeof(line),
           "HEALTHY_PEER_COMPARISON timeout_node=%u.%02u peer_node=%u.%02u r1_reply_age_ms=%lu can_rx_age_ms=%lu cache_reply_age_ms=%lu",
           static_cast<unsigned>(timeoutSubnet),
           static_cast<unsigned>(timeoutDevice),
           static_cast<unsigned>(peer->subnet),
           static_cast<unsigned>(peer->device),
           static_cast<unsigned long>(bestReplyAgeMs),
           static_cast<unsigned long>(nowMs - peer->lastCanRxMs),
           static_cast<unsigned long>(peer->lastCacheReplyMs == 0 ? 0u : nowMs - peer->lastCacheReplyMs));
  debugCli_.logText(line);
  logAliveCandidateCorrelation(peer->subnet, peer->device, nowMs);
}

void R70GatewayV2::logAliveCandidateCorrelation(uint8_t subnet, uint8_t device,
                                                uint32_t nowMs) const {
  // Channel error is transferred with the channel bits in data1, therefore
  // the raw trace keys are 2600 (A) and 4600 (B), not the logical family 0600.
  static const uint16_t objects[] = {0x0111, 0x0300, 0x0500, 0x2600,
                                     0x4600, 0x0100, 0x010D};
  char line[320] = {};
  for (uint8_t objectIndex = 0; objectIndex < sizeof(objects) / sizeof(objects[0]); ++objectIndex) {
    const uint16_t objectKey = objects[objectIndex];
    const RequestTrace* latest = 0;
    for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
      const uint16_t index = static_cast<uint16_t>(
          (recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) % kRecentRequestTraceCapacity);
      const RequestTrace& trace = recentRequestTraces_[index];
      if (trace.used && trace.subnet == subnet && trace.device == device && trace.objectKey == objectKey) {
        latest = &trace;
        break;
      }
    }
    const Frame* cachedFrame = 0;
    const AmpState* amp = state_.amp(subnet, device);
    if (objectKey == 0x0111 && amp != 0 && amp->unknown0111CacheValid) {
      cachedFrame = &amp->unknown0111CacheFrame;
    } else if (latest != 0) {
      NormalizedRequest request = {};
      request.valid = true;
      request.canId.subnet = subnet;
      request.canId.device = device;
      request.objectAddress.channelNumber = latest->channel;
      request.objectAddress.boxNumber = static_cast<uint8_t>(objectKey >> 8);
      request.objectAddress.objectNumber = static_cast<uint8_t>(objectKey);
      request.fieldId = latest->fieldId;
      request.admin.valid = true;
      request.admin.telegramType = TelegramType::Unstructured;
      const SemanticMirrorEntry* entry = findSemanticMirrorEntry(request);
      if (entry != 0 && entry->learned && !entry->invalidatedByWrite) cachedFrame = &entry->replyFrame;
    }
    snprintf(line, sizeof(line),
             "ALIVE_CANDIDATE_STATE node=%u.%02u object=%04X req_age_ms=%lu reply_age_ms=%lu source=%s latency_ms=%u pending=%u request_len=%u request=%02X%02X%02X%02X response_valid=%u response_len=%u response=%02X%02X%02X%02X%02X%02X%02X%02X",
             subnet, device, objectKey,
             static_cast<unsigned long>(latest != 0 ? nowMs - latest->timestampMs : 0u),
             static_cast<unsigned long>(latest != 0 && latest->replyTimestampMs != 0 ? nowMs - latest->replyTimestampMs : 0u),
             latest != 0 ? traceSourceName(latest->source) : "none",
             latest != 0 ? latest->latencyMs : 0u,
             latest != 0 && (latest->flags & kTraceFlagTcpTxSent) == 0 ? 1u : 0u,
             latest != 0 ? latest->length : 0u,
             latest != 0 ? latest->data[0] : 0u, latest != 0 ? latest->data[1] : 0u,
             latest != 0 ? latest->data[2] : 0u, latest != 0 ? latest->data[3] : 0u,
             cachedFrame != 0 ? 1u : 0u, cachedFrame != 0 ? cachedFrame->length : 0u,
             cachedFrame != 0 ? cachedFrame->data[0] : 0u, cachedFrame != 0 ? cachedFrame->data[1] : 0u,
             cachedFrame != 0 ? cachedFrame->data[2] : 0u, cachedFrame != 0 ? cachedFrame->data[3] : 0u,
             cachedFrame != 0 ? cachedFrame->data[4] : 0u, cachedFrame != 0 ? cachedFrame->data[5] : 0u,
             cachedFrame != 0 ? cachedFrame->data[6] : 0u, cachedFrame != 0 ? cachedFrame->data[7] : 0u);
    debugCli_.logText(line);
  }
  if (aliveDiagnosticTraces_ == 0) {
    debugCli_.logText("ALIVE_SEQUENCE unavailable=allocation_failed");
    return;
  }
  uint8_t emitted = 0;
  for (uint16_t offset = 0; offset < kAliveDiagnosticTraceCapacity && emitted < 96; ++offset) {
    const uint16_t index = static_cast<uint16_t>(
        (aliveDiagnosticTraceHead_ + kAliveDiagnosticTraceCapacity - 1u - offset) %
        kAliveDiagnosticTraceCapacity);
    const AliveDiagnosticTrace& trace = aliveDiagnosticTraces_[index];
    if (trace.timestampMs == 0 || trace.subnet != subnet || trace.device != device ||
        nowMs - trace.timestampMs > 10000u) continue;
    snprintf(line, sizeof(line),
             "ALIVE_SEQUENCE kind=all_r1_reads node=%u.%02u reverse_index=%u object=%04X channel=%u req_age_ms=%lu reply_age_ms=%lu source=%s latency_ms=%u",
             subnet, device, emitted, trace.objectKey,
             trace.channel,
             static_cast<unsigned long>(nowMs - trace.timestampMs),
             static_cast<unsigned long>(trace.replyTimestampMs ? nowMs - trace.replyTimestampMs : 0u),
             traceSourceName(trace.source), trace.latencyMs);
    debugCli_.logText(line);
    ++emitted;
  }
}

void R70GatewayV2::logGroupTimeoutCorrelation(const uint8_t* subnets,
                                              const uint8_t* devices,
                                              uint8_t count,
                                              uint32_t nowMs,
                                              const char* prefix) const {
  if (subnets == 0 || devices == 0 || count == 0) {
    return;
  }
  char line[256] = {};
  size_t used = static_cast<size_t>(snprintf(line, sizeof(line), "T=%lums %s nodes=",
                                             static_cast<unsigned long>(nowMs),
                                             prefix != 0 ? prefix : "GROUP_TIMEOUT_CORRELATION"));
  for (uint8_t index = 0; index < count && used + 8 < sizeof(line); ++index) {
    const int written = snprintf(line + used, sizeof(line) - used, "%s%u.%02u",
                                 index == 0 ? "" : ",",
                                 static_cast<unsigned>(subnets[index]),
                                 static_cast<unsigned>(devices[index]));
    if (written <= 0) {
      break;
    }
    used += static_cast<size_t>(written);
  }
  debugCli_.logText(line);

  struct GroupFieldStat {
    bool used;
    uint16_t objectKey;
    FieldId fieldId;
    uint32_t pendingMask;
    uint32_t staleMask;
    uint16_t maxLatencyMs;
  };
  GroupFieldStat stats[16] = {};

  for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
    const uint16_t index =
        static_cast<uint16_t>((recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) %
                              kRecentRequestTraceCapacity);
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used) {
      continue;
    }
    uint8_t nodeIndex = 0;
    if (!nodeInList(trace.subnet, trace.device, subnets, devices, count, &nodeIndex)) {
      continue;
    }
    const uint32_t ageMs = nowMs - trace.timestampMs;
    GroupFieldStat* stat = 0;
    GroupFieldStat* empty = 0;
    for (uint8_t slot = 0; slot < 16; ++slot) {
      if (!stats[slot].used) {
        if (empty == 0) {
          empty = &stats[slot];
        }
        continue;
      }
      if (stats[slot].objectKey == trace.objectKey && stats[slot].fieldId == trace.fieldId) {
        stat = &stats[slot];
        break;
      }
    }
    if (stat == 0 && empty != 0) {
      empty->used = true;
      empty->objectKey = trace.objectKey;
      empty->fieldId = trace.fieldId;
      empty->pendingMask = 0;
      empty->staleMask = 0;
      empty->maxLatencyMs = 0;
      stat = empty;
    }
    if (stat == 0) {
      continue;
    }
    if ((trace.flags & kTraceFlagTcpTxSent) == 0 && ageMs > kWatchdogWarnAgeMs) {
      stat->pendingMask |= (1ul << nodeIndex);
    }
    if ((trace.flags & kTraceFlagStaleAtRequest) != 0) {
      stat->staleMask |= (1ul << nodeIndex);
    }
    if ((trace.flags & kTraceFlagTcpTxSent) != 0 && ageMs <= kSlowFieldLookbackMs &&
        trace.latencyMs > stat->maxLatencyMs) {
      stat->maxLatencyMs = trace.latencyMs;
    }
  }

  struct RankedGroupField {
    const GroupFieldStat* stat;
    uint32_t score;
  };
  RankedGroupField top[8] = {};
  for (uint8_t slot = 0; slot < 16; ++slot) {
    if (!stats[slot].used) {
      continue;
    }
    const uint8_t sharedPendingCount = bitCount32(stats[slot].pendingMask);
    const uint8_t sharedStaleCount = bitCount32(stats[slot].staleMask);
    const uint32_t score = static_cast<uint32_t>(sharedPendingCount) * 100000u +
                           static_cast<uint32_t>(sharedStaleCount) * 10000u +
                           stats[slot].maxLatencyMs;
    if (score == 0) {
      continue;
    }
    for (uint8_t rank = 0; rank < 8; ++rank) {
      if (top[rank].stat == 0 || score > top[rank].score) {
        for (int8_t shift = 7; shift > static_cast<int8_t>(rank); --shift) {
          top[shift] = top[shift - 1];
        }
        top[rank].stat = &stats[slot];
        top[rank].score = score;
        break;
      }
    }
  }

  for (uint8_t rank = 0; rank < 8; ++rank) {
    if (top[rank].stat == 0) {
      continue;
    }
    const GroupFieldStat& stat = *top[rank].stat;
    const char* label = stat.fieldId != FieldId::Unknown ? fieldIdName(stat.fieldId) :
        (stat.objectKey == 0x0111 ? "unknown_0111" : "unknown");
    snprintf(line, sizeof(line),
             "T=%lums GROUP_FIELD rank=%u object=%04X field=%s shared_pending_nodes=%u shared_stale_nodes=%u max_latency_ms=%u",
             static_cast<unsigned long>(nowMs),
             static_cast<unsigned>(rank + 1),
             static_cast<unsigned>(stat.objectKey),
             label,
             static_cast<unsigned>(bitCount32(stat.pendingMask)),
             static_cast<unsigned>(bitCount32(stat.staleMask)),
             static_cast<unsigned>(stat.maxLatencyMs));
    debugCli_.logText(line);
  }
}

// Only confirmed scalar reads are eligible. Never replay writes or segments.
static int8_t liveRefreshIndex(const NormalizedRequest& request) {
  if (!request.valid || !request.admin.valid || request.canId.device == 0 ||
      !isReadFunction(request.admin.functionCode) ||
      request.admin.telegramType != TelegramType::Unstructured ||
      request.objectAddress.hasRecord || request.objectAddress.hasSegment) return -1;
  if (request.fieldId == FieldId::chstatus_isp_a) return 1;
  if (request.fieldId == FieldId::chstatus_isp_b) return 2;
  if (request.fieldId == FieldId::chstatus_input_voltage_peak_a) return 3;
  if (request.fieldId == FieldId::chstatus_input_voltage_peak_b) return 4;
  const uint16_t objectKey = normalizedObjectKey(request);
  if (objectKey == 0x0500) return 0;
  return -1;
}

static bool isExtraLiveRefreshField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::chstatus_gr_a:
    case FieldId::chstatus_gr_b:
    case FieldId::chstatus_ovl_a:
    case FieldId::chstatus_ovl_b:
    case FieldId::chstatus_osp_a:
    case FieldId::chstatus_osp_b:
    case FieldId::chstatus_gr_head_a:
    case FieldId::chstatus_gr_head_b:
    case FieldId::chstatus_gr_head_peak_a:
    case FieldId::chstatus_gr_head_peak_b:
    case FieldId::chstatus_remhold_isp_a:
    case FieldId::chstatus_remhold_isp_b:
    case FieldId::chstatus_remhold_gr_a:
    case FieldId::chstatus_remhold_gr_b:
    case FieldId::chstatus_remhold_ovl_a:
    case FieldId::chstatus_remhold_ovl_b:
    case FieldId::chstatus_remhold_osp_a:
    case FieldId::chstatus_remhold_osp_b:
      return true;
    default:
      return false;
  }
}

static bool isRemoteHoldLiveField(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::chstatus_remhold_isp_a:
    case FieldId::chstatus_remhold_isp_b:
    case FieldId::chstatus_remhold_gr_a:
    case FieldId::chstatus_remhold_gr_b:
    case FieldId::chstatus_remhold_ovl_a:
    case FieldId::chstatus_remhold_ovl_b:
    case FieldId::chstatus_remhold_osp_a:
    case FieldId::chstatus_remhold_osp_b:
      return true;
    default:
      return false;
  }
}

void R70GatewayV2::queueLiveRefresh(const NormalizedRequest& request,
                                    uint32_t nowMs,
                                    bool r1Requested) {
  const int8_t index = liveRefreshIndex(request);
  AmpState* amp = state_.amp(request.canId.subnet, request.canId.device, false);
  if (amp == 0 || (index < 0 && !isExtraLiveRefreshField(request.fieldId))) return;
  AmpState::LiveRefresh* trackPtr = index >= 0 ? &amp->liveRefresh[index] : 0;
  if (trackPtr == 0) {
    ExtraLiveRefresh* oldest = 0;
    for (uint8_t i = 0; i < kExtraLiveRefreshCount; ++i) {
      ExtraLiveRefresh& entry = extraLiveRefresh_[i];
      if (entry.used && entry.subnet == request.canId.subnet &&
          entry.device == request.canId.device && entry.fieldId == request.fieldId) {
        trackPtr = &entry.track;
        break;
      }
      if (!entry.used) oldest = &entry;
      else if (!entry.track.pending &&
               (oldest == 0 || oldest->track.lastR1RequestMs > entry.track.lastR1RequestMs)) oldest = &entry;
    }
    if (trackPtr == 0 && oldest != 0) {
      *oldest = {};
      oldest->used = true;
      oldest->subnet = request.canId.subnet;
      oldest->device = request.canId.device;
      oldest->fieldId = request.fieldId;
      trackPtr = &oldest->track;
    }
  }
  if (trackPtr == 0) return;
  AmpState::LiveRefresh& track = *trackPtr;
  track.request = request.frame;
  track.learned = true;
  if (r1Requested) track.lastR1RequestMs = nowMs;
  const bool retryReady = track.recovery.ready(track.pending, track.queuedMs, nowMs);
  if (!semanticMirrorActive(nowMs) || !retryReady ||
      nowMs - amp->lastCanRxMs > kNodeFreshMirrorServeMaxAgeMs) return;
  const bool mirrorBacked = index == 0;
  const SemanticMirrorEntry* mirror = mirrorBacked ? findSemanticMirrorEntry(request) : 0;
  const FieldCache* cache = amp->cache(request.fieldId);
  const bool valid = mirrorBacked ? mirror != 0 && mirror->learned && !mirror->invalidatedByWrite
                                  : cache != 0 && cache->valid;
  if (!valid) return;
  const uint32_t updatedMs = mirrorBacked ? mirror->lastUpdateMs : cache->lastUpdateMs;
  const uint32_t intervalMs = index == 0 ? 3000u : (index == 1 || index == 2 ? 200u : 100u);
  if (nowMs - updatedMs < intervalMs || (track.queued != 0 && nowMs - track.queuedMs < intervalMs)) return;
  const char* reason = 0;
  bool limited = false;
  bool allowed = false;
  if (index == 0) {
    allowed = currentBusUtilAvgPercent(nowMs) < 70u &&
              semanticMirrorBackgroundRefreshAllowed(request, nowMs, reason, limited);
  } else {
    // ISP and Signal share a bounded live-value budget.  At most 125 extra
    // reads/s, low priority, and stop adding work at 70% estimated bus load
    // (headroom below the requested 80% ceiling).
    allowed = nowMs - lastIspRefreshMs_ >= 4u &&
              !semanticMirrorBackgroundRefreshPaused(nowMs) &&
              currentBusUtilAvgPercent(nowMs) < 70u &&
              pendingTcpReplyCount() < 20u && scheduler_.queueDepth() == 0 &&
              counters_.can_tx_queue_depth <= 4u;
  }
  if (!allowed || !scheduler_.enqueueMaintenance(request, nowMs, counters_, reason)) {
    ++track.skipped;
    return;
  }
  track.pending = true;
  // Retain sent ownership on retries: a late reply is still cache-only.
  track.recovery.enqueued(track.recovery.expired);
  track.queuedMs = nowMs;
  ++track.queued;
  if (index == 0) noteSemanticMirrorBackgroundRefreshSent(request, nowMs);
  else lastIspRefreshMs_ = nowMs;
}

void R70GatewayV2::processLocalControlRefresh(uint32_t nowMs) {
  // Keep startup behavior identical to V128.  Local controls begin only after
  // the amplifier set has had a full minute to bootstrap and only while R1
  // service and CAN load have ample headroom.
  const bool mirrorActive = semanticMirrorActive(nowMs);
  const uint32_t firstAmpAgeMs = firstAmpDiscoveryMs_ != 0 && firstAmpDiscoveryMs_ <= nowMs
                                     ? nowMs - firstAmpDiscoveryMs_ : 0u;
  const uint32_t busUtil = currentBusUtilAvgPercent(nowMs);
  const uint16_t pendingTcp = pendingTcpReplyCount();
  const uint32_t serviceGapMs = maxR1ReplyAgeMs(nowMs, 0);
  const char* gate = !mirrorActive ? "mirror_inactive" :
                     (firstAmpDiscoveryMs_ == 0 ? "no_first_amp" :
                     (firstAmpAgeMs < 60000u ? "startup_60s" :
                     (nowMs - lastLocalControlRefreshMs_ < 24u ? "spacing" :
                     (busUtil >= 70u ? "bus_util" :
                     (pendingTcp >= 20u ? "tcp_pressure" :
                     (serviceGapMs >= 400u ? "service_gap" : "scan"))))));
  if (strcmp(gate, "scan") != 0) {
    if (lastLocalControlRefreshDiagMs_ == 0 ||
        nowMs - lastLocalControlRefreshDiagMs_ >= 10000u) {
      lastLocalControlRefreshDiagMs_ = nowMs;
      char line[224] = {};
      snprintf(line, sizeof(line),
               "LOCAL_CONTROL_REFRESH_DIAG gate=%s mirror_active=%u first_amp_age_ms=%lu bus_util=%lu pending_tcp=%u service_gap_ms=%lu",
               gate, static_cast<unsigned>(mirrorActive ? 1u : 0u),
               static_cast<unsigned long>(firstAmpAgeMs),
               static_cast<unsigned long>(busUtil), static_cast<unsigned>(pendingTcp),
               static_cast<unsigned long>(serviceGapMs));
      debugCli_.logText(line);
    }
    return;
  }

  uint16_t usedCount = 0;
  uint16_t objectCount = 0;
  uint16_t unlearnedCount = 0;
  uint16_t invalidatedCount = 0;
  uint16_t timestampCount = 0;
  uint16_t inflightCount = 0;
  uint16_t buildFailedCount = 0;
  uint16_t enqueueFailedCount = 0;
  for (uint16_t scanned = 0; scanned < kSemanticMirrorCapacity; ++scanned) {
    if (nextLocalControlRefreshSlot_ >= kSemanticMirrorCapacity) {
      nextLocalControlRefreshSlot_ = 0;
    }
    SemanticMirrorEntry& entry = semanticMirror_[nextLocalControlRefreshSlot_++];
    if (entry.used) ++usedCount;
    const bool localControlObject = entry.objectKey == 0x2205u || entry.objectKey == 0x4205u ||
                                    entry.objectKey == 0x2206u || entry.objectKey == 0x4206u;
    if (!entry.used || !localControlObject) continue;
    ++objectCount;
    if (!entry.learned) { ++unlearnedCount; continue; }
    if (entry.invalidatedByWrite) { ++invalidatedCount; continue; }
    if (entry.lastUpdateMs == 0 || entry.lastUpdateMs > nowMs) {
      ++timestampCount;
      continue;
    }
    if (entry.refreshInflight) {
      if (entry.refreshEnqueuedMs != 0 && nowMs - entry.refreshEnqueuedMs >= 2000u) {
        entry.refreshInflight = false;
        entry.refreshEnqueuedMs = 0;
      } else {
        ++inflightCount;
        continue;
      }
    }

    NormalizedRequest request = {};
    if (!buildSemanticMirrorRefreshRequest(entry, request)) {
      // The confirmed scalar reply contains the complete three-byte read
      // address.  Use it only for these four explicitly allow-listed objects
      // when the separately remembered R1 template is absent.
      if (entry.replyFrame.length < 3) { ++buildFailedCount; continue; }
      request.valid = true;
      request.tcpWrappedCan = true;
      request.canId.valid = true;
      request.canId.subnet = entry.subnet;
      request.canId.device = entry.device;
      request.admin.valid = true;
      request.admin.functionCode = FunctionCode::ReadObject;
      request.admin.telegramType = entry.telegramType;
      request.objectAddress.channelNumber = entry.channel;
      request.objectAddress.boxNumber = static_cast<uint8_t>((entry.objectKey >> 8) & 0xFFu);
      request.objectAddress.objectNumber = static_cast<uint8_t>(entry.objectKey & 0xFFu);
      request.fieldId = entry.fieldId;
      request.frame.canId = protocol::makeCanId(
          protocol::kObjectClassPrimaryMaster, Address{entry.subnet, entry.device});
      request.frame.length = 3;
      request.frame.data[0] = static_cast<uint8_t>(
          (entry.replyFrame.data[0] & 0xF0u) | static_cast<uint8_t>(FunctionCode::ReadObject));
      request.frame.data[1] = entry.replyFrame.data[1];
      request.frame.data[2] = entry.replyFrame.data[2];
    }

    const char* reason = 0;
    if (!scheduler_.enqueueMaintenance(request, nowMs, counters_, reason)) {
      ++enqueueFailedCount;
      continue;
    }
    entry.refreshInflight = true;
    entry.refreshEnqueuedMs = nowMs;
    lastLocalControlRefreshMs_ = nowMs;

    if (shouldLogLowRateCounter(semanticMirrorWindow_.backgroundRefreshSent + 1u)) {
      char line[192] = {};
      snprintf(line, sizeof(line),
               "LOCAL_CONTROL_REFRESH_QUEUED node=%u.%02u object=%04X age_ms=%lu",
               static_cast<unsigned>(request.canId.subnet),
               static_cast<unsigned>(request.canId.device),
               static_cast<unsigned>(normalizedObjectKey(request)),
               static_cast<unsigned long>(nowMs - entry.lastUpdateMs));
      debugCli_.logText(line);
    }
    ++semanticMirrorWindow_.backgroundRefreshSent;
    return;
  }

  if (lastLocalControlRefreshDiagMs_ == 0 ||
      nowMs - lastLocalControlRefreshDiagMs_ >= 10000u) {
    lastLocalControlRefreshDiagMs_ = nowMs;
    char line[256] = {};
    snprintf(line, sizeof(line),
             "LOCAL_CONTROL_REFRESH_DIAG gate=no_candidate used=%u objects=%u unlearned=%u invalidated=%u bad_time=%u inflight=%u build_failed=%u enqueue_failed=%u",
             static_cast<unsigned>(usedCount), static_cast<unsigned>(objectCount),
             static_cast<unsigned>(unlearnedCount), static_cast<unsigned>(invalidatedCount),
             static_cast<unsigned>(timestampCount), static_cast<unsigned>(inflightCount),
             static_cast<unsigned>(buildFailedCount), static_cast<unsigned>(enqueueFailedCount));
    debugCli_.logText(line);
  }
}

void R70GatewayV2::processOverdueAliveRefresh(uint32_t nowMs) {
  if (scheduler_.queueDepth() != 0 || currentBusUtilAvgPercent(nowMs) >= 70u) return;
  const uint8_t slots = state_.activeAmpCount() * 2u;
  for (uint16_t n = 0; n < slots; ++n) {
    if (nextAliveRefreshSlot_ >= slots) nextAliveRefreshSlot_ = 0;
    const uint8_t slot = nextAliveRefreshSlot_++;
    const AmpState* amp = state_.activeAmpByIndex(slot / 2u);
    if (amp == 0 || nowMs - amp->lastCanRxMs > kNodeFreshMirrorServeMaxAgeMs) continue;
    const bool exact = slot % 2u == 0;
    const Frame& frame = exact ? amp->unknown0111RefreshRequest : amp->liveRefresh[0].request;
    if (frame.length == 0) continue;
    NormalizedRequest request = {};
    if (!codec::normalizeCanFrame(frame, RequestOrigin::Tcp, request)) continue;
    const SemanticMirrorEntry* mirror = exact ? 0 : findSemanticMirrorEntry(request);
    if (exact ? !amp->unknown0111CacheValid : (mirror == 0 || !mirror->learned || mirror->invalidatedByWrite)) continue;
    const uint32_t age = nowMs - (exact ? amp->unknown0111CacheUpdateMs : mirror->lastUpdateMs);
    if (age < 4000u) continue;
    const uint32_t before = exact ? amp->unknown0111RefreshQueuedCount : amp->liveRefresh[0].queued;
    if (exact) queueUnknown0111Refresh(request, nowMs);
    else queueLiveRefresh(request, nowMs);
    const uint32_t after = exact ? amp->unknown0111RefreshQueuedCount : amp->liveRefresh[0].queued;
    if (before != after) return;
  }

  // Keep the remaining R1 watchdog candidates genuinely fresh on CAN. R1
  // still gets its requested replies immediately from the mirror. Object
  // 0100 is refreshed separately from live R1 alive traffic, so it must not
  // also be scheduled here. No cached or synthetic 0100 frame is pushed.
  static const uint32_t kStableAliveRefreshIntervalMs = 5000u;
  static const uint32_t kStableAliveRefreshRetryMs = 2000u;
  for (uint16_t scanned = 0; scanned < kSemanticMirrorCapacity; ++scanned) {
    if (nextStableAliveRefreshSlot_ >= kSemanticMirrorCapacity) {
      nextStableAliveRefreshSlot_ = 0;
    }
    SemanticMirrorEntry& entry = semanticMirror_[nextStableAliveRefreshSlot_++];
    const bool stableAliveObject = entry.objectKey == 0x0300u ||
                                   entry.objectKey == 0x2600u || entry.objectKey == 0x4600u;
    const uint32_t refreshIntervalMs = kStableAliveRefreshIntervalMs;
    if (!entry.used || !entry.learned || entry.invalidatedByWrite || !stableAliveObject ||
        entry.lastUpdateMs == 0 || entry.lastUpdateMs > nowMs ||
        nowMs - entry.lastUpdateMs < refreshIntervalMs) {
      continue;
    }
    const AmpState* amp = state_.amp(entry.subnet, entry.device);
    if (amp == 0 || amp->lastCanRxMs == 0 || nowMs < amp->lastCanRxMs ||
        nowMs - amp->lastCanRxMs > kNodeFreshMirrorServeMaxAgeMs) {
      continue;
    }
    if (entry.fieldId == FieldId::settings_power && entry.replyFrame.length >= 4 &&
        entry.replyFrame.data[3] == 0u) {
      continue;
    }
    if (entry.refreshInflight) {
      if (entry.refreshEnqueuedMs != 0 &&
          nowMs - entry.refreshEnqueuedMs >= kStableAliveRefreshRetryMs) {
        entry.refreshInflight = false;
        entry.refreshEnqueuedMs = 0;
      } else {
        continue;
      }
    }

    NormalizedRequest refresh = {};
    if (!buildSemanticMirrorRefreshRequest(entry, refresh)) continue;
    const char* reason = 0;
    bool rateLimited = false;
    if (!semanticMirrorBackgroundRefreshAllowed(refresh, nowMs, reason, rateLimited) ||
        !scheduler_.enqueueMaintenance(refresh, nowMs, counters_, reason)) {
      continue;
    }
    entry.refreshInflight = true;
    entry.refreshEnqueuedMs = nowMs;
    noteSemanticMirrorBackgroundRefreshSent(refresh, nowMs);
    if (entry.objectKey == 0x0100u) {
      ++r1Object0100HeartbeatCanTx_;
    }
    if (shouldLogLowRateCounter(semanticMirrorWindow_.backgroundRefreshSent)) {
      char line[176] = {};
      snprintf(line, sizeof(line),
               "STABLE_ALIVE_REFRESH_QUEUED node=%u.%02u object=%04X value_age_ms=%lu",
               static_cast<unsigned>(entry.subnet), static_cast<unsigned>(entry.device),
               static_cast<unsigned>(entry.objectKey),
               static_cast<unsigned long>(nowMs - entry.lastUpdateMs));
      debugCli_.logText(line);
    }
    return;
  }
}

void R70GatewayV2::processIspRefresh(uint32_t nowMs) {
  if (!semanticMirrorActive(nowMs) || nowMs - lastIspRefreshMs_ < 4u ||
      nowMs - lastIspScanMs_ < 4u) return;
  lastIspScanMs_ = nowMs;
  // Overdue alive/error reads get first use of an idle maintenance slot.
  // Round-robin prevents a repeatedly unanswered node from starving others.
  processOverdueAliveRefresh(nowMs);
  const uint16_t slots = static_cast<uint16_t>(state_.activeAmpCount()) * 4u;
  // Values requested by R1 in the last two seconds are visible now.  Give
  // those live meters first use of the maintenance slot; all learned ISP and
  // Signal values still remain in the background round-robin below.
  for (uint8_t n = 0; n < slots; ++n) {
    if (nextFastLiveRefreshSlot_ >= slots) nextFastLiveRefreshSlot_ = 0;
    const uint8_t slot = nextFastLiveRefreshSlot_++;
    const uint8_t liveIndex = static_cast<uint8_t>(1u + slot % 4u);
    const AmpState* amp = state_.activeAmpByIndex(slot / 4u);
    if (amp == 0) continue;
    const AmpState::LiveRefresh& track = amp->liveRefresh[liveIndex];
    if (!track.learned || track.lastR1RequestMs == 0 ||
        nowMs - track.lastR1RequestMs > 2000u) continue;
    NormalizedRequest request = {};
    if (!codec::normalizeCanFrame(track.request, RequestOrigin::Tcp, request)) continue;
    queueLiveRefresh(request, nowMs);
    if (lastIspRefreshMs_ == nowMs) return;
  }
  // RAM-bounded tracks for the additional meters. They are allocated only
  // when R1 actually requests one of these values, rather than per amplifier.
  for (uint8_t n = 0; n < kExtraLiveRefreshCount; ++n) {
    if (nextExtraLiveRefreshSlot_ >= kExtraLiveRefreshCount) nextExtraLiveRefreshSlot_ = 0;
    ExtraLiveRefresh& entry = extraLiveRefresh_[nextExtraLiveRefreshSlot_++];
    AmpState::LiveRefresh& track = entry.track;
    if (!entry.used || !track.learned || track.lastR1RequestMs == 0 ||
        nowMs - track.lastR1RequestMs > 2000u) continue;
    NormalizedRequest request = {};
    if (!codec::normalizeCanFrame(track.request, RequestOrigin::Tcp, request)) continue;
    queueLiveRefresh(request, nowMs);
    if (lastIspRefreshMs_ == nowMs) return;
  }
  for (uint8_t n = 0; n < slots; ++n) {
    if (nextIspRefreshSlot_ >= slots) nextIspRefreshSlot_ = 0;
    const uint8_t slot = nextIspRefreshSlot_++;
    const uint8_t liveIndex = static_cast<uint8_t>(1u + slot % 4u);
    const AmpState* amp = state_.activeAmpByIndex(slot / 4u);
    if (amp == 0 || !amp->liveRefresh[liveIndex].learned) continue;
    NormalizedRequest request = {};
    if (!codec::normalizeCanFrame(amp->liveRefresh[liveIndex].request, RequestOrigin::Tcp, request)) continue;
    queueLiveRefresh(request, nowMs);
    if (lastIspRefreshMs_ == nowMs) break;
  }
  if (lastIspRefreshMs_ == nowMs) return;
  for (uint8_t n = 0; n < kExtraLiveRefreshCount; ++n) {
    if (nextExtraLiveRefreshSlot_ >= kExtraLiveRefreshCount) nextExtraLiveRefreshSlot_ = 0;
    ExtraLiveRefresh& entry = extraLiveRefresh_[nextExtraLiveRefreshSlot_++];
    if (!entry.used || !entry.track.learned) continue;
    NormalizedRequest request = {};
    if (!codec::normalizeCanFrame(entry.track.request, RequestOrigin::Tcp, request)) continue;
    queueLiveRefresh(request, nowMs);
    if (lastIspRefreshMs_ == nowMs) break;
  }
}

void R70GatewayV2::queueUnknown0111Refresh(const NormalizedRequest& request, uint32_t nowMs) {
  AmpState* amp = state_.amp(request.canId.subnet, request.canId.device, false);
  if (amp != 0) amp->unknown0111RefreshRequest = request.frame;
  if (amp == 0 || !amp->unknown0111CacheValid || !semanticMirrorActive(nowMs)) {
    return;
  }
  // Expire after 2s and retry with bounded exponential backoff; existing
  // queue, node/subnet and global rate limits still apply.
  if (!amp->unknown0111Recovery.ready(amp->unknown0111RefreshPending,
                                    amp->unknown0111RefreshQueuedMs, nowMs) ||
      nowMs - amp->unknown0111CacheUpdateMs < 3000u ||
      (amp->unknown0111RefreshQueuedCount != 0 &&
       nowMs - amp->unknown0111RefreshQueuedMs < 3000u)) {
    return;
  }
  const char* reason = 0;
  bool rateLimited = false;
  if (currentBusUtilAvgPercent(nowMs) >= 70u ||
      !semanticMirrorBackgroundRefreshAllowed(request, nowMs, reason, rateLimited) ||
      !scheduler_.enqueueMaintenance(request, nowMs, counters_, reason)) {
    ++amp->unknown0111RefreshSkippedCount;
    return;
  }
  amp->unknown0111RefreshPending = true;
  amp->unknown0111Recovery.enqueued(amp->unknown0111Recovery.expired);
  amp->unknown0111RefreshQueuedMs = nowMs;
  ++amp->unknown0111RefreshQueuedCount;
  noteSemanticMirrorBackgroundRefreshSent(request, nowMs);
}

void R70GatewayV2::logTrueCacheAges(uint8_t subnet, uint8_t device, uint32_t nowMs) const {
  const AmpState* amp = state_.amp(subnet, device);
  if (amp == 0) return;
  char line[320] = {};
  for (uint8_t i = 0; i < AmpState::kLiveRefreshCount; ++i) {
    const AmpState::LiveRefresh& track = amp->liveRefresh[i];
    static const char* const targets[AmpState::kLiveRefreshCount] = {
      "0500", "isp_a", "isp_b", "signal_a", "signal_b"
    };
    snprintf(line, sizeof(line),
             "CACHE_REFRESH_TRACK node=%u.%02u target=%s learned=%u visible_age_ms=%lu bg_pending=%u sent=%u bg_age_ms=%lu queued=%lu completed=%lu skipped=%lu timeouts=%u retries=%u backoff_ms=%lu",
             subnet, device,
             targets[i],
             track.learned ? 1u : 0u,
             static_cast<unsigned long>(track.lastR1RequestMs != 0 && track.lastR1RequestMs <= nowMs
                                            ? nowMs - track.lastR1RequestMs : 0u),
             track.pending ? 1u : 0u, track.sent ? 1u : 0u,
             static_cast<unsigned long>(track.pending ? nowMs - track.queuedMs : 0u),
             static_cast<unsigned long>(track.queued), static_cast<unsigned long>(track.completed),
             static_cast<unsigned long>(track.skipped), track.recovery.timeouts, track.recovery.retries,
             static_cast<unsigned long>(track.recovery.failures ? track.recovery.backoffMs() : 0u));
    debugCli_.logText(line);
  }
  snprintf(line, sizeof(line),
           "CACHE_REFRESH_TRACK node=%u.%02u object=0111 valid=%u value_age_ms=%lu bg_pending=%u bg_age_ms=%lu queued=%lu completed=%lu skipped=%lu can_rx=%lu r1_tx=%lu timeouts=%u retries=%u",
           subnet, device, amp->unknown0111CacheValid ? 1u : 0u,
           static_cast<unsigned long>(amp->unknown0111CacheValid ? nowMs - amp->unknown0111CacheUpdateMs : 0u),
           amp->unknown0111RefreshPending ? 1u : 0u,
           static_cast<unsigned long>(amp->unknown0111RefreshPending ? nowMs - amp->unknown0111RefreshQueuedMs : 0u),
           static_cast<unsigned long>(amp->unknown0111RefreshQueuedCount),
           static_cast<unsigned long>(amp->unknown0111RefreshDoneCount),
           static_cast<unsigned long>(amp->unknown0111RefreshSkippedCount),
           static_cast<unsigned long>(amp->unknown0111CanRx),
           static_cast<unsigned long>(amp->unknown0111TcpTx),
           amp->unknown0111Recovery.timeouts, amp->unknown0111Recovery.retries);
  debugCli_.logText(line);
  for (uint16_t i = 0; i < kSemanticMirrorCapacity; ++i) {
    const SemanticMirrorEntry& entry = semanticMirror_[i];
    if (!entry.used || !entry.learned || entry.subnet != subnet || entry.device != device) continue;
    snprintf(line, sizeof(line),
             "CACHE_VALUE_AGE node=%u.%02u object=%04X store=mirror value_age_ms=%lu served=%u r1_serve_age_ms=%lu invalidated=%u",
             subnet, device, entry.objectKey,
             static_cast<unsigned long>(nowMs - entry.lastUpdateMs),
             entry.lastServedMs != 0 ? 1u : 0u,
             static_cast<unsigned long>(entry.lastServedMs != 0 ? nowMs - entry.lastServedMs : 0u),
             entry.invalidatedByWrite ? 1u : 0u);
    debugCli_.logText(line);
  }
  for (uint16_t i = 0; i < amp->cacheCapacity; ++i) {
    const FieldCache& cache = amp->caches[i];
    if (!cache.valid) continue;
    snprintf(line, sizeof(line), "CACHE_VALUE_AGE node=%u.%02u field=%s store=field value_age_ms=%lu",
             subnet, device, fieldIdName(cache.fieldId),
             static_cast<unsigned long>(nowMs - cache.lastUpdateMs));
    debugCli_.logText(line);
  }
}

void R70GatewayV2::logUnknown0111Summary(uint32_t nowMs) const {
  struct RankedNode {
    const AmpState* amp;
    uint32_t score;
  };
  RankedNode top[8] = {};
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || ampState->unknown0111Requests == 0) {
      continue;
    }
    const uint32_t score = static_cast<uint32_t>(ampState->unknown0111Pending) * 100000u +
                           ampState->unknown0111MaxLatencyMs;
    for (uint8_t slot = 0; slot < 8; ++slot) {
      if (top[slot].amp == 0 || score > top[slot].score) {
        for (int8_t shift = 7; shift > static_cast<int8_t>(slot); --shift) {
          top[shift] = top[shift - 1];
        }
        top[slot].amp = ampState;
        top[slot].score = score;
        break;
      }
    }
  }

  char line[256] = {};
  for (uint8_t slot = 0; slot < 8; ++slot) {
    if (top[slot].amp == 0) {
      continue;
    }
    const AmpState& ampState = *top[slot].amp;
    snprintf(line, sizeof(line),
             "T=%lums UNKNOWN_0111_SUMMARY node=%u.%02u req=%lu can_tx=%lu can_rx=%lu tcp_tx=%lu pending=%u max_latency_ms=%lu last_value=%02X",
             static_cast<unsigned long>(nowMs),
             static_cast<unsigned>(ampState.subnet),
             static_cast<unsigned>(ampState.device),
             static_cast<unsigned long>(ampState.unknown0111Requests),
             static_cast<unsigned long>(ampState.unknown0111CanTx),
             static_cast<unsigned long>(ampState.unknown0111CanRx),
             static_cast<unsigned long>(ampState.unknown0111TcpTx),
             static_cast<unsigned>(ampState.unknown0111Pending),
             static_cast<unsigned long>(ampState.unknown0111MaxLatencyMs),
             static_cast<unsigned>(ampState.unknown0111LastValue));
    debugCli_.logText(line);
  }
}

void R70GatewayV2::pollCli(uint32_t nowMs) {
  if (cliStream_ == 0) {
    return;
  }
  while (cliStream_->available() > 0) {
    const int ch = cliStream_->read();
    if (ch < 0) {
      break;
    }
    if (ch == '\r' || ch == '\n') {
      if (cliBufferLength_ != 0) {
        cliBuffer_[cliBufferLength_] = '\0';
        handleCliLine(cliBuffer_, nowMs);
        cliBufferLength_ = 0;
      }
      continue;
    }
    if (cliBufferLength_ + 1u < kCliBufferCapacity) {
      cliBuffer_[cliBufferLength_++] = static_cast<char>(ch);
    }
  }
}

void R70GatewayV2::handleCliLine(const char* line, uint32_t nowMs) {
  if (line == 0) {
    return;
  }
  if (cliStream_ != 0 && NetworkConfig_handleCliCommand(line, *cliStream_)) {
    return;
  }
  if (cliStream_ != 0 && GpiManager_handleCliCommand(line, *cliStream_, nowMs)) {
    return;
  }
  if (strncmp(line, "timeout ", 8) == 0) {
    const char* cursor = line + 8;
    char* end = 0;
    const unsigned long subnet = strtoul(cursor, &end, 10);
    if (end == cursor) {
      return;
    }
    while (*end == ' ') {
      ++end;
    }
    char* deviceEnd = 0;
    const unsigned long device = strtoul(end, &deviceEnd, 10);
    if (deviceEnd == end || subnet > 7 || device > 63) {
      return;
    }
    logTimeoutCorrelation(static_cast<uint8_t>(subnet),
                          static_cast<uint8_t>(device),
                          nowMs,
                          "TIMEOUT_CORRELATION");
    return;
  }
  if (strncmp(line, "mark ", 5) != 0) {
    return;
  }

  const char* text = line + 5;
  char markerLine[256] = {};
  snprintf(markerLine, sizeof(markerLine), "T=%lums EVENT_MARKER text=%s",
           static_cast<unsigned long>(nowMs),
           text);
  debugCli_.logText(markerLine);

  snprintf(markerLine, sizeof(markerLine),
           "T=%lums EVENT_MARKER_QUEUES can_tx_queue_depth=%lu normal_read_queue_depth=%lu normal_read_queue_oldest_age_ms=%lu",
           static_cast<unsigned long>(nowMs),
           static_cast<unsigned long>(counters_.can_tx_queue_depth),
           static_cast<unsigned long>(counters_.normal_read_queue_depth),
           static_cast<unsigned long>(counters_.normal_read_queue_oldest_age_ms));
  debugCli_.logText(markerLine);

  if (strncmp(text, "timeout ", 8) == 0) {
    uint8_t subnets[8] = {};
    uint8_t devices[8] = {};
    const uint8_t count = parseNodeListInText(text + 8, subnets, devices, 8);
    if (count != 0) {
      for (uint8_t index = 0; index < count; ++index) {
        logTimeoutCorrelation(subnets[index], devices[index], nowMs, "TIMEOUT_CORRELATION");
      }
      logGroupTimeoutCorrelation(subnets, devices, count, nowMs, "TIMEOUT_CORRELATION_GROUP");
      return;
    }
  }

  uint8_t subnet = 0;
  uint8_t device = 0;
  if (parseNodeSpecInText(text, &subnet, &device)) {
    logTimeoutCorrelation(subnet, device, nowMs, "EVENT_MARKER_TIMEOUT_CORRELATION");
    return;
  }

  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0) {
      continue;
    }
    const uint32_t lastTcpTxAgeMs =
        ampState->lastTcpTxToR1Ms == 0 ? 0 : (nowMs - ampState->lastTcpTxToR1Ms);
    const uint32_t lastRealCanRxAgeMs =
        ampState->lastCanRxMs == 0 ? 0 : (nowMs - ampState->lastCanRxMs);
    const uint32_t lastCacheReplyAgeMs =
        ampState->lastCacheReplyMs == 0 ? 0 : (nowMs - ampState->lastCacheReplyMs);
    const uint32_t lastCanTxPollAgeMs =
        ampState->lastCanTxPollMs == 0 ? 0 : (nowMs - ampState->lastCanTxPollMs);
    debugCli_.logNodeReplyAgeTop(ampState->subnet,
                                 ampState->device,
                                 static_cast<uint8_t>(ampIndex + 1),
                                 nowMs,
                                 lastTcpTxAgeMs,
                                 lastRealCanRxAgeMs,
                                 lastCacheReplyAgeMs,
                                 lastCanTxPollAgeMs,
                                 ampState->tcpTxCount,
                                 ampState->realCanRxCount,
                                 ampState->cacheReplyCount,
                                 ampState->canTxPollCount);
    // A manual marker is also the healthy baseline for timeout comparison.
    // Always snapshot the per-node semantic family time range here; timeout
    // markers already emit the same diagnostic through logTimeoutCorrelation().
    logNodeSemanticInconsistency(ampState->subnet,
                                 ampState->device,
                                 nowMs,
                                 "SEMANTIC_TIME_SPREAD");
    logAliveCandidateCorrelation(ampState->subnet, ampState->device, nowMs);
  }
  logUnknown0111Summary(nowMs);
}

const char* R70GatewayV2::traceSourceName(TraceSource source) {
  switch (source) {
    case TraceSource::Cache:
      return "cache";
    case TraceSource::RealCan:
      return "real_can";
    case TraceSource::Mirror:
      return "mirror";
    case TraceSource::None:
    default:
      return "none";
  }
}

const char* R70GatewayV2::traceMirrorDecisionName(TraceMirrorDecision decision) {
  switch (decision) {
    case TraceMirrorDecision::MirrorServed:
      return "mirror_served";
    case TraceMirrorDecision::ForcedCanMissing:
      return "forced_can_missing";
    case TraceMirrorDecision::ForcedCanStale:
      return "forced_can_stale";
    case TraceMirrorDecision::ForcedCanNotSafe:
      return "forced_can_not_safe";
    case TraceMirrorDecision::ForcedCanWriteInvalidated:
      return "forced_can_write_invalidated";
    case TraceMirrorDecision::None:
    default:
      return "none";
  }
}

const char* R70GatewayV2::traceBackgroundRefreshStateName(TraceBackgroundRefreshState state) {
  switch (state) {
    case TraceBackgroundRefreshState::Sent:
      return "sent";
    case TraceBackgroundRefreshState::SkippedTcpPressure:
      return "skipped_tcp_pressure";
    case TraceBackgroundRefreshState::SkippedCanPending:
      return "skipped_can_pending";
    case TraceBackgroundRefreshState::SkippedServiceGap:
      return "skipped_service_gap";
    case TraceBackgroundRefreshState::SkippedRateLimit:
      return "skipped_rate_limit";
    case TraceBackgroundRefreshState::SkippedDiagnosticHold:
      return "skipped_diagnostic_hold";
    case TraceBackgroundRefreshState::None:
    default:
      return "none";
  }
}

const char* R70GatewayV2::tracePendingStateName(TracePendingState state) {
  switch (state) {
    case TracePendingState::Created:
      return "created";
    case TracePendingState::WaitingCan:
      return "waiting_can";
    case TracePendingState::CanComplete:
      return "can_complete";
    case TracePendingState::PartialComplete:
      return "partial_complete";
    case TracePendingState::ReplySent:
      return "reply_sent";
    case TracePendingState::TimeoutCleanup:
      return "timeout_cleanup";
    default:
      return "unknown";
  }
}

const char* R70GatewayV2::traceReleaseReasonName(TraceReleaseReason reason) {
  switch (reason) {
    case TraceReleaseReason::MirrorReply:
      return "mirror_reply";
    case TraceReleaseReason::CacheReply:
      return "cache_reply";
    case TraceReleaseReason::RealCanReply:
      return "real_can_reply";
    case TraceReleaseReason::AgedOutNoReply:
      return "aged_out_no_reply";
    case TraceReleaseReason::TcpSendFailed:
      return "tcp_send_failed";
    case TraceReleaseReason::None:
    default:
      return "none";
  }
}

const R70GatewayV2::RequestTrace* R70GatewayV2::findLastSuccessfulTraceForNode(uint8_t subnet,
                                                                                uint8_t device) const {
  for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
    const uint16_t index =
        static_cast<uint16_t>((recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) %
                              kRecentRequestTraceCapacity);
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used || trace.subnet != subnet || trace.device != device) {
      continue;
    }
    if ((trace.flags & kTraceFlagTcpTxSent) == 0) {
      continue;
    }
    return &trace;
  }
  return 0;
}

uint8_t R70GatewayV2::semanticFamilyIndex(SemanticMirrorGroup group) {
  if (group == SemanticMirrorGroup::None) {
    return 0xFFu;
  }
  const uint8_t index = static_cast<uint8_t>(group) - 1u;
  return index < AmpState::kSemanticFamilyCount ? index : 0xFFu;
}

const char* R70GatewayV2::semanticSourceName(uint8_t sourceCode) {
  switch (sourceCode) {
    case 1:
      return "mirror";
    case 2:
      return "real_can";
    case 3:
      return "cache";
    case 4:
      return "bg_refresh";
    default:
      return "none";
  }
}

void R70GatewayV2::noteSemanticFamilyRequested(const NormalizedRequest& request, uint32_t nowMs) {
  const SemanticMirrorGroup group = classifySemanticMirrorGroup(request);
  const uint8_t index = semanticFamilyIndex(group);
  if (index == 0xFFu || request.canId.device == 0) {
    return;
  }
  AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device, false);
  if (ampState == 0) {
    return;
  }
  AmpState::SemanticFamilyState& family = ampState->semanticFamilies[index];
  family.pending = true;
  family.requestMs = nowMs;
  family.objectKey = normalizedObjectKey(request);
}

void R70GatewayV2::noteSemanticFamilyUpdated(const NormalizedRequest& request,
                                             uint32_t nowMs,
                                             uint8_t sourceCode,
                                             bool servedToR1) {
  const SemanticMirrorGroup group = classifySemanticMirrorGroup(request);
  const uint8_t index = semanticFamilyIndex(group);
  if (index == 0xFFu || request.canId.device == 0) {
    return;
  }
  AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device, false);
  if (ampState == 0) {
    return;
  }
  AmpState::SemanticFamilyState& family = ampState->semanticFamilies[index];
  family.valid = true;
  family.source = sourceCode;
  family.objectKey = normalizedObjectKey(request);
  family.lastUpdateMs = nowMs;
  if (servedToR1) {
    family.pending = false;
  }
  ++ampState->semanticEpoch;
  if (ampState->semanticEpoch == 0) {
    ampState->semanticEpoch = 1u;
  }
  family.epoch = ampState->semanticEpoch;
}

void R70GatewayV2::logNodeSemanticEpoch(uint8_t subnet,
                                        uint8_t device,
                                        uint32_t nowMs,
                                        const char* prefix) const {
  const AmpState* ampState = state_.amp(subnet, device);
  if (ampState == 0) {
    return;
  }
  char line[256] = {};
  snprintf(line,
           sizeof(line),
           "%s node=%u.%02u epoch=%u",
           prefix != 0 ? prefix : "SEMANTIC_EPOCH",
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned>(ampState->semanticEpoch));
  debugCli_.logText(line);
  for (uint8_t index = 0; index < AmpState::kSemanticFamilyCount; ++index) {
    const AmpState::SemanticFamilyState& family = ampState->semanticFamilies[index];
    if (!family.valid && !family.pending) {
      continue;
    }
    const uint32_t ageMs = family.valid && family.lastUpdateMs != 0 && nowMs >= family.lastUpdateMs
                               ? nowMs - family.lastUpdateMs
                               : 0u;
    snprintf(line,
             sizeof(line),
             "%s_FAMILY node=%u.%02u family=%s object=%04X source=%s age_ms=%lu epoch=%u pending=%u",
             prefix != 0 ? prefix : "SEMANTIC_EPOCH",
             static_cast<unsigned>(subnet),
             static_cast<unsigned>(device),
             semanticMirrorGroupName(static_cast<SemanticMirrorGroup>(index + 1u)),
             static_cast<unsigned>(family.objectKey),
             semanticSourceName(family.source),
             static_cast<unsigned long>(ageMs),
             static_cast<unsigned>(family.epoch),
             static_cast<unsigned>(family.pending ? 1u : 0u));
    debugCli_.logText(line);
  }
}

void R70GatewayV2::logNodeSemanticInconsistency(uint8_t subnet,
                                                uint8_t device,
                                                uint32_t nowMs,
                                                const char* prefix) const {
  logTrueCacheAges(subnet, device, nowMs);
  const AmpState* ampState = state_.amp(subnet, device);
  if (ampState == 0) {
    return;
  }
  uint32_t minAgeMs = 0xFFFFFFFFu;
  uint32_t maxAgeMs = 0;
  uint8_t firstSource = 0;
  bool mixedSources = false;
  bool epochMismatch = false;
  uint16_t firstEpoch = 0;
  uint8_t pendingFamilies = 0;
  uint8_t validFamilies = 0;
  uint8_t newestFamilyIndex = 0xFFu;
  uint8_t oldestFamilyIndex = 0xFFu;
  for (uint8_t index = 0; index < AmpState::kSemanticFamilyCount; ++index) {
    const AmpState::SemanticFamilyState& family = ampState->semanticFamilies[index];
    if (family.pending) {
      ++pendingFamilies;
    }
    if (!family.valid || family.lastUpdateMs == 0 || nowMs < family.lastUpdateMs) {
      continue;
    }
    ++validFamilies;
    const uint32_t ageMs = nowMs - family.lastUpdateMs;
    if (ageMs < minAgeMs) {
      minAgeMs = ageMs;
      newestFamilyIndex = index;
    }
    if (ageMs > maxAgeMs) {
      maxAgeMs = ageMs;
      oldestFamilyIndex = index;
    }
    if (firstSource == 0) {
      firstSource = family.source;
    } else if (family.source != 0 && family.source != firstSource) {
      mixedSources = true;
    }
    if (firstEpoch == 0) {
      firstEpoch = family.epoch;
    } else if (family.epoch != 0 && family.epoch != firstEpoch) {
      epochMismatch = true;
    }
  }
  const uint32_t ageDeltaMs =
      validFamilies >= 2 && minAgeMs != 0xFFFFFFFFu && maxAgeMs >= minAgeMs ? (maxAgeMs - minAgeMs) : 0u;
  const AmpState::SemanticFamilyState* newestFamily =
      newestFamilyIndex != 0xFFu ? &ampState->semanticFamilies[newestFamilyIndex] : 0;
  const AmpState::SemanticFamilyState* oldestFamily =
      oldestFamilyIndex != 0xFFu ? &ampState->semanticFamilies[oldestFamilyIndex] : 0;
  char line[256] = {};
  snprintf(line,
           sizeof(line),
           "%s node=%u.%02u valid_families=%u object_time_spread_ms=%lu mixed_sources=%u cache_epoch_mismatch=%u pending_families=%u",
           prefix != 0 ? prefix : "SEMANTIC_INCONSISTENCY",
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned>(validFamilies),
           static_cast<unsigned long>(ageDeltaMs),
           static_cast<unsigned>(mixedSources ? 1u : 0u),
           static_cast<unsigned>(epochMismatch ? 1u : 0u),
           static_cast<unsigned>(pendingFamilies));
  debugCli_.logText(line);
  snprintf(line,
           sizeof(line),
           "%s_RANGE node=%u.%02u newest_object=%04X newest_family=%s newest_source=%s newest_age_ms=%lu oldest_object=%04X oldest_family=%s oldest_source=%s oldest_age_ms=%lu",
           prefix != 0 ? prefix : "SEMANTIC_INCONSISTENCY",
           static_cast<unsigned>(subnet),
           static_cast<unsigned>(device),
           static_cast<unsigned>(newestFamily != 0 ? newestFamily->objectKey : 0u),
           newestFamilyIndex != 0xFFu
               ? semanticMirrorGroupName(static_cast<SemanticMirrorGroup>(newestFamilyIndex + 1u))
               : "none",
           newestFamily != 0 ? semanticSourceName(newestFamily->source) : "none",
           static_cast<unsigned long>(newestFamily != 0 ? minAgeMs : 0u),
           static_cast<unsigned>(oldestFamily != 0 ? oldestFamily->objectKey : 0u),
           oldestFamilyIndex != 0xFFu
               ? semanticMirrorGroupName(static_cast<SemanticMirrorGroup>(oldestFamilyIndex + 1u))
               : "none",
           oldestFamily != 0 ? semanticSourceName(oldestFamily->source) : "none",
           static_cast<unsigned long>(oldestFamily != 0 ? maxAgeMs : 0u));
  debugCli_.logText(line);
}

void R70GatewayV2::beginTimeoutWindowDump(uint32_t nowMs) {
  timeoutWindowDumpActive_ = false;
  timeoutWindowDumpCount_ = 0;
  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount() && timeoutWindowDumpCount_ < 8; ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || !ampState->used || !ampState->r1TimeoutActive) {
      continue;
    }
    timeoutWindowDumpSubnets_[timeoutWindowDumpCount_] = ampState->subnet;
    timeoutWindowDumpDevices_[timeoutWindowDumpCount_] = ampState->device;
    ++timeoutWindowDumpCount_;
  }
  if (timeoutWindowDumpCount_ == 0) {
    return;
  }
  timeoutWindowDumpActive_ = true;
  timeoutWindowDumpEventMs_ = nowMs;
  timeoutWindowDumpDueMs_ = nowMs + 5000u;
  logTimeoutWindowDump(nowMs >= 10000u ? nowMs - 10000u : 0u, nowMs, nowMs, "TIMEOUT_WINDOW_PREV10S");
}

void R70GatewayV2::processTimeoutWindowDump(uint32_t nowMs) {
  if (!timeoutWindowDumpActive_ || nowMs < timeoutWindowDumpDueMs_) {
    return;
  }
  logTimeoutWindowDump(timeoutWindowDumpEventMs_, timeoutWindowDumpDueMs_, nowMs, "TIMEOUT_WINDOW_NEXT5S");
  timeoutWindowDumpActive_ = false;
  timeoutWindowDumpCount_ = 0;
}

void R70GatewayV2::logTimeoutWindowDump(uint32_t startMs,
                                        uint32_t endMs,
                                        uint32_t nowMs,
                                        const char* prefix) const {
  if (timeoutWindowDumpCount_ == 0) {
    return;
  }
  char line[256] = {};
  snprintf(line,
           sizeof(line),
           "T=%lums %s start_ms=%lu end_ms=%lu pending_tcp_replies=%u pending_can_reads=%u bg_refresh_pause_active=%u",
           static_cast<unsigned long>(nowMs),
           prefix != 0 ? prefix : "TIMEOUT_WINDOW",
           static_cast<unsigned long>(startMs),
           static_cast<unsigned long>(endMs),
           static_cast<unsigned>(pendingTcpReplyCount()),
           static_cast<unsigned>(scheduler_.queueDepth()),
           static_cast<unsigned>(semanticMirrorBackgroundRefreshPaused(nowMs) ? 1u : 0u));
  debugCli_.logText(line);

  for (uint8_t nodeIndex = 0; nodeIndex < timeoutWindowDumpCount_; ++nodeIndex) {
    const uint8_t subnet = timeoutWindowDumpSubnets_[nodeIndex];
    const uint8_t device = timeoutWindowDumpDevices_[nodeIndex];
    uint16_t requestCount = 0;
    uint16_t replyCount = 0;
    uint16_t forcedCanCount = 0;
    uint16_t mirrorServeCount = 0;
    uint16_t mirrorMissCount = 0;
    uint16_t mirrorStaleCount = 0;
    uint16_t mirrorNotSafeCount = 0;
    uint16_t bgRefreshSentCount = 0;
    uint16_t bgRefreshSkippedCount = 0;

    const RequestTrace* lastSuccess = findLastSuccessfulTraceForNode(subnet, device);
    NormalizedRequest lastSuccessRequest = {};
    if (lastSuccess != 0) {
      lastSuccessRequest.valid = true;
      lastSuccessRequest.canId.subnet = subnet;
      lastSuccessRequest.canId.device = device;
      lastSuccessRequest.objectAddress.channelNumber = lastSuccess->channel;
      lastSuccessRequest.objectAddress.boxNumber = static_cast<uint8_t>((lastSuccess->objectKey >> 8) & 0xFFu);
      lastSuccessRequest.objectAddress.objectNumber = static_cast<uint8_t>(lastSuccess->objectKey & 0xFFu);
      lastSuccessRequest.frame.length = 3;
      lastSuccessRequest.frame.data[1] = lastSuccessRequest.objectAddress.boxNumber;
      lastSuccessRequest.frame.data[2] = lastSuccessRequest.objectAddress.objectNumber;
    }
    const SemanticMirrorGroup lastSuccessGroup =
        lastSuccess != 0 ? classifySemanticMirrorGroup(lastSuccessRequest) : SemanticMirrorGroup::None;
    snprintf(line,
             sizeof(line),
             "T=%lums %s node=%u.%02u last_success_object=%04X last_success_family=%s last_success_source=%s",
             static_cast<unsigned long>(nowMs),
             prefix != 0 ? prefix : "TIMEOUT_WINDOW",
             static_cast<unsigned>(subnet),
             static_cast<unsigned>(device),
             static_cast<unsigned>(lastSuccess != 0 ? lastSuccess->objectKey : 0u),
             semanticMirrorGroupName(lastSuccessGroup),
             lastSuccess != 0 ? traceSourceName(lastSuccess->source) : "none");
    debugCli_.logText(line);

    for (uint16_t offset = 0; offset < kRecentRequestTraceCapacity; ++offset) {
      const uint16_t index =
          static_cast<uint16_t>((recentRequestTraceHead_ + kRecentRequestTraceCapacity - 1u - offset) %
                                kRecentRequestTraceCapacity);
      const RequestTrace& trace = recentRequestTraces_[index];
      if (!trace.used || trace.subnet != subnet || trace.device != device) {
        continue;
      }
      const bool requestInWindow = trace.timestampMs >= startMs && trace.timestampMs <= endMs;
      const bool replyInWindow = trace.replyTimestampMs != 0 &&
                                 trace.replyTimestampMs >= startMs && trace.replyTimestampMs <= endMs;
      if (!requestInWindow && !replyInWindow) {
        continue;
      }
      if (requestInWindow) {
        ++requestCount;
      }
      if (replyInWindow) {
        ++replyCount;
      }
      if (trace.mirrorDecision == TraceMirrorDecision::MirrorServed) {
        ++mirrorServeCount;
      } else if (trace.mirrorDecision == TraceMirrorDecision::ForcedCanMissing ||
                 trace.mirrorDecision == TraceMirrorDecision::ForcedCanWriteInvalidated) {
        ++mirrorMissCount;
        ++forcedCanCount;
      } else if (trace.mirrorDecision == TraceMirrorDecision::ForcedCanStale) {
        ++mirrorStaleCount;
        ++forcedCanCount;
      } else if (trace.mirrorDecision == TraceMirrorDecision::ForcedCanNotSafe) {
        ++mirrorNotSafeCount;
        ++forcedCanCount;
      }
      if (trace.backgroundRefreshState == TraceBackgroundRefreshState::Sent) {
        ++bgRefreshSentCount;
      } else if (trace.backgroundRefreshState != TraceBackgroundRefreshState::None) {
        ++bgRefreshSkippedCount;
      }
      snprintf(line,
               sizeof(line),
               "T=%lums %s_TRACE node=%u.%02u object=%04X field=%s req_ms=%lu reply_ms=%lu source=%s mirror=%s bg_refresh=%s pending_tcp=%u pending_can=%u can_rx=%u tcp_tx=%u stale=%u latency_ms=%u",
               static_cast<unsigned long>(nowMs),
               prefix != 0 ? prefix : "TIMEOUT_WINDOW",
               static_cast<unsigned>(subnet),
               static_cast<unsigned>(device),
               static_cast<unsigned>(trace.objectKey),
               trace.fieldId != FieldId::Unknown ? fieldIdName(trace.fieldId) : "unknown",
               static_cast<unsigned long>(trace.timestampMs),
               static_cast<unsigned long>(trace.replyTimestampMs),
               traceSourceName(trace.source),
               traceMirrorDecisionName(trace.mirrorDecision),
               traceBackgroundRefreshStateName(trace.backgroundRefreshState),
               static_cast<unsigned>(trace.pendingTcpRepliesAtRequest),
               static_cast<unsigned>(trace.canTxQueueDepthAtRequest),
               (trace.flags & kTraceFlagRealCanRxSeen) != 0 ? 1u : 0u,
               (trace.flags & kTraceFlagTcpTxSent) != 0 ? 1u : 0u,
               (trace.flags & kTraceFlagStaleAtRequest) != 0 ? 1u : 0u,
               static_cast<unsigned>(trace.latencyMs));
      debugCli_.logText(line);
    }

    snprintf(line,
             sizeof(line),
             "T=%lums %s_SUMMARY node=%u.%02u requests=%u replies=%u mirror_served=%u forced_can=%u mirror_missing=%u mirror_stale=%u mirror_not_safe=%u bg_refresh_sent=%u bg_refresh_skipped=%u",
             static_cast<unsigned long>(nowMs),
             prefix != 0 ? prefix : "TIMEOUT_WINDOW",
             static_cast<unsigned>(subnet),
             static_cast<unsigned>(device),
             static_cast<unsigned>(requestCount),
             static_cast<unsigned>(replyCount),
             static_cast<unsigned>(mirrorServeCount),
             static_cast<unsigned>(forcedCanCount),
             static_cast<unsigned>(mirrorMissCount),
             static_cast<unsigned>(mirrorStaleCount),
             static_cast<unsigned>(mirrorNotSafeCount),
             static_cast<unsigned>(bgRefreshSentCount),
             static_cast<unsigned>(bgRefreshSkippedCount));
    debugCli_.logText(line);
  }
}

uint32_t R70GatewayV2::maxPendingTcpOldestMs(uint32_t nowMs) const {
  uint32_t oldestMs = 0;
  for (uint16_t index = 0; index < kRecentRequestTraceCapacity; ++index) {
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used || (trace.flags & kTraceFlagTcpTxSent) != 0 ||
        trace.pendingState == TracePendingState::TimeoutCleanup) {
      continue;
    }
    const uint32_t ageMs = nowMs >= trace.timestampMs ? nowMs - trace.timestampMs : 0u;
    if (ageMs > oldestMs) {
      oldestMs = ageMs;
    }
  }
  return oldestMs;
}

void R70GatewayV2::logTcpPendingSummary(uint32_t nowMs) const {
  uint16_t pendingTotal = 0;
  uint16_t pendingOver250 = 0;
  uint16_t pendingOver500 = 0;
  uint16_t pendingOver1000 = 0;
  uint16_t canCompleteButNotSent = 0;
  uint16_t partialCompleteWaiting = 0;
  uint16_t stuckWaiters = 0;
  uint16_t stuckFanouts = 0;
  uint16_t pendingByGroup[kSemanticMirrorGroupCount] = {};

  for (uint16_t index = 0; index < kRecentRequestTraceCapacity; ++index) {
    const RequestTrace& trace = recentRequestTraces_[index];
    if (!trace.used || (trace.flags & kTraceFlagTcpTxSent) != 0 ||
        trace.pendingState == TracePendingState::TimeoutCleanup) {
      continue;
    }
    ++pendingTotal;
    const uint32_t ageMs = nowMs >= trace.timestampMs ? nowMs - trace.timestampMs : 0u;
    if (ageMs > 250u) {
      ++pendingOver250;
    }
    if (ageMs > 500u) {
      ++pendingOver500;
    }
    if (ageMs > 1000u) {
      ++pendingOver1000;
    }
    if ((trace.flags & kTraceFlagRealCanRxSeen) != 0) {
      ++canCompleteButNotSent;
    }
    if (trace.pendingState == TracePendingState::PartialComplete) {
      ++partialCompleteWaiting;
    }
    if (trace.waiterFanoutCount != 0) {
      ++stuckWaiters;
      ++stuckFanouts;
    }
    NormalizedRequest pendingRequest = {};
    pendingRequest.valid = true;
    pendingRequest.canId.subnet = trace.subnet;
    pendingRequest.canId.device = trace.device;
    pendingRequest.objectAddress.channelNumber = trace.channel;
    pendingRequest.objectAddress.boxNumber = static_cast<uint8_t>((trace.objectKey >> 8) & 0xFFu);
    pendingRequest.objectAddress.objectNumber = static_cast<uint8_t>(trace.objectKey & 0xFFu);
    const SemanticMirrorGroup group = classifySemanticMirrorGroup(pendingRequest);
    if (group != SemanticMirrorGroup::None) {
      const uint8_t groupIndex = static_cast<uint8_t>(group) - 1u;
      if (groupIndex < kSemanticMirrorGroupCount) {
        ++pendingByGroup[groupIndex];
      }
    }
  }

  char line[224] = {};
  snprintf(line,
           sizeof(line),
           "TCP_PENDING_SUMMARY pending_total=%u pending_over_250ms=%u pending_over_500ms=%u pending_over_1000ms=%u can_complete_but_not_sent=%u partial_complete_waiting=%u stuck_waiters=%u stuck_fanouts=%u max_pending_tcp_oldest_ms=%lu",
           static_cast<unsigned>(pendingTotal),
           static_cast<unsigned>(pendingOver250),
           static_cast<unsigned>(pendingOver500),
           static_cast<unsigned>(pendingOver1000),
           static_cast<unsigned>(canCompleteButNotSent),
           static_cast<unsigned>(partialCompleteWaiting),
           static_cast<unsigned>(stuckWaiters),
           static_cast<unsigned>(stuckFanouts),
           static_cast<unsigned long>(maxPendingTcpOldestMs(nowMs)));
  debugCli_.logText(line);

  for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
    const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
    if (ampState == 0 || !ampState->used || ampState->device == 0) {
      continue;
    }
    uint16_t nodePending = 0;
    uint32_t oldestNodePendingMs = 0;
    for (uint16_t index = 0; index < kRecentRequestTraceCapacity; ++index) {
      const RequestTrace& trace = recentRequestTraces_[index];
      if (!trace.used || trace.subnet != ampState->subnet || trace.device != ampState->device ||
          (trace.flags & kTraceFlagTcpTxSent) != 0 ||
          trace.pendingState == TracePendingState::TimeoutCleanup) {
        continue;
      }
      ++nodePending;
      const uint32_t ageMs = nowMs >= trace.timestampMs ? nowMs - trace.timestampMs : 0u;
      if (ageMs > oldestNodePendingMs) {
        oldestNodePendingMs = ageMs;
      }
    }
    if (nodePending == 0) {
      continue;
    }
    snprintf(line,
             sizeof(line),
             "TCP_PENDING_BY_NODE node=%u.%02u pending=%u oldest_ms=%lu",
             static_cast<unsigned>(ampState->subnet),
             static_cast<unsigned>(ampState->device),
             static_cast<unsigned>(nodePending),
             static_cast<unsigned long>(oldestNodePendingMs));
    debugCli_.logText(line);
  }

  for (uint8_t groupIndex = 0; groupIndex < kSemanticMirrorGroupCount; ++groupIndex) {
    if (pendingByGroup[groupIndex] == 0) {
      continue;
    }
    snprintf(line,
             sizeof(line),
             "TCP_PENDING_BY_OBJECT_GROUP group=%s pending=%u",
             semanticMirrorGroupName(static_cast<SemanticMirrorGroup>(groupIndex + 1u)),
             static_cast<unsigned>(pendingByGroup[groupIndex]));
    debugCli_.logText(line);
  }
}

bool R70GatewayV2::begin(const GatewayConfig& config, Print& log) {
  config_ = config;
  if (config_.maxTcpBackpressureBurstMs == 0) {
    config_.maxTcpBackpressureBurstMs = 50;
  }
  if (config_.maxTcpBackpressureTotalPerSecondMs == 0) {
    config_.maxTcpBackpressureTotalPerSecondMs = 150;
  }
  if (!config.enableEmergencyTcpBackpressure &&
      config.maxTcpBackpressureBurstMs == 0 &&
      config.maxTcpBackpressureTotalPerSecondMs == 0) {
    config_.enableEmergencyTcpBackpressure = true;
  }
  log_ = &log;
  cliStream_ = (&log == static_cast<Print*>(&Serial)) ? &Serial : 0;
  cliBufferLength_ = 0;
  counters_.clear();
  resetObjectStats();
  if (aliveDiagnosticTraces_ == 0) {
#if defined(ARDUINO_ARCH_ESP32)
    const uint32_t caps = psramFound() ? (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : MALLOC_CAP_8BIT;
    aliveDiagnosticTraces_ = static_cast<AliveDiagnosticTrace*>(
        heap_caps_calloc(kAliveDiagnosticTraceCapacity, sizeof(AliveDiagnosticTrace), caps));
#else
    aliveDiagnosticTraces_ = static_cast<AliveDiagnosticTrace*>(
        calloc(kAliveDiagnosticTraceCapacity, sizeof(AliveDiagnosticTrace)));
#endif
  }
  resetRequestTraces();
  debugCli_.begin(log);
  debugCli_.setVerboseFrameLogEnabled(config_.enableVerboseFrameLog);
  counters_.psram_total = psramTotalBytes();
  counters_.psram_free = psramFreeBytes();
  const uint8_t segmentedReplyMaxSegments =
      (config_.nameSegmentMaxSegments != 0 ? config_.nameSegmentMaxSegments : GatewayState::kNameSegmentMaxSegments) >
              (config_.firmwareSegmentMaxSegments != 0 ? config_.firmwareSegmentMaxSegments : GatewayState::kNameSegmentMaxSegments)
          ? (config_.nameSegmentMaxSegments != 0 ? config_.nameSegmentMaxSegments : GatewayState::kNameSegmentMaxSegments)
          : (config_.firmwareSegmentMaxSegments != 0 ? config_.firmwareSegmentMaxSegments : GatewayState::kNameSegmentMaxSegments);
  if (!state_.begin(counters_.psram_total != 0,
                    segmentedReplyMaxSegments,
                    config_.nameSegmentPayloadMaxLen != 0 ? config_.nameSegmentPayloadMaxLen : GatewayState::kNameSegmentPayloadMaxLen)) {
    return false;
  }

  if (!canTransport_.begin(config.canTxPin, config.canRxPin, config.canBitrate,
                           config.canListenOnly)) {
    return false;
  }
  if (!discovery_.begin(config.udpPort)) {
    return false;
  }

  tcpServer_.begin(config.tcpPort);
  const uint32_t normalSpacingUs =
      config.canTxMinSpacingUsNormal != 0 ? config.canTxMinSpacingUsNormal :
      (config.canTxMinSpacingUs != 0 ? config.canTxMinSpacingUs : kDefaultCanTxMinSpacingUsNormal);
  const uint32_t writeSpacingUs =
      config.canTxMinSpacingUsWrite != 0 ? config.canTxMinSpacingUsWrite : kDefaultCanTxMinSpacingUsWrite;
  const uint32_t scanSpacingUs =
      config.canTxMinSpacingUsScan != 0 ? config.canTxMinSpacingUsScan : kDefaultCanTxMinSpacingUsScan;
  scheduler_.begin(canTransport_,
                   normalSpacingUs,
                   writeSpacingUs,
                   scanSpacingUs,
                   config.identWindowMs != 0 ? config.identWindowMs : 550,
                   config.identQuietHoldMs != 0 ? config.identQuietHoldMs : 75,
                   config.enableWriteRetry,
                   config.writeAckTimeoutMs != 0 ? config.writeAckTimeoutMs : 500,
                   debugCli_);
  scheduler_.setCanTxObserver(&R70GatewayV2::handleSchedulerCanTx, this);
  state_.updateDebugStats(counters_);
  canTxQueueHighWater_ = scheduler_.queueHighWatermark();
  canTxQueueLowWater_ = scheduler_.queueLowWatermark();
  started_ = true;
  tcpBackpressureActive_ = false;
  tcpBackpressureBurstLimited_ = false;
  tcpBackpressureTotalLimited_ = false;
  tcpBackpressureStartMs_ = 0;
  tcpBackpressureWindowStartMs_ = 0;
  tcpBackpressureWindowUsedMs_ = 0;
  lastStateSummaryMs_ = 0;
  lastSummaryCanTxFrames_ = 0;
  lastSummaryCanRxFrames_ = 0;
  lastSummaryTcpRxFrames_ = 0;
  lastSummaryTcpTxFrames_ = 0;
  lastSummaryMuteWriteTxCount_ = 0;
  lastSummaryMuteWriteAckCount_ = 0;
  lastSummaryMuteWriteTimeoutCount_ = 0;
  lastSummaryRx441_ = 0;
  lastSummaryRx442_ = 0;
  lastLongRunSummaryMs_ = millis();
  lastLongRunQueueFullTotal_ = counters_.can_tx_queue_full_total;
  lastLongRunTcpBackpressureActiveMs_ = counters_.tcp_backpressure_active_ms;
  lastLongRunCacheRepliesSent_ = counters_.cache_replies_sent;
  lastLongRunCanTxFrames_ = counters_.can_tx_frames;
  lastLongRunCanRxFrames_ = counters_.can_rx_frames;
  lastRuntimeBaselineSummaryMs_ = 0;
  runtimeStableCandidateSinceMs_ = 0;
  runtimeStableEnteredMs_ = 0;
  longRunNormalOldestMin_ = 0;
  longRunNormalOldestMax_ = 0;
  longRunNormalOldestSum_ = 0;
  longRunNormalOldestSamples_ = 0;
  longRunNormalDepthMax_ = 0;
  longRunCanTxDepthMax_ = 0;
  lastAutoGroupTimeoutSnapshotMs_ = 0;
  lastPollMicros_ = 0;
  lastTcpPollMicros_ = 0;
  lastCanPollMicros_ = 0;
  loopMaxUsWindow_ = 0;
  loopSumUsWindow_ = 0;
  loopSamplesWindow_ = 0;
  canPollMaxGapUsWindow_ = 0;
  tcpPollMaxGapUsWindow_ = 0;
  maxSerialPrintBurstMsWindow_ = 0;
  serialBytesPrintedEstimateWindow_ = 0;
  canResponseLatencySumMs_ = 0;
  canResponseLatencyCount_ = 0;
  lastPrintedCanResponseOver1000Ms_ = 0;
  canResponseOver1000GrowthStreak_ = 0;
  lastIdentTxMs_ = 0;
  lastTcpConnected_ = false;
  r1ConnectedSinceMs_ = 0;
  bootstrapProtectionUntilMs_ = 0;
  bootstrapCompleteStableSinceMs_ = 0;
  bootstrapRecoveryWindowStartedMs_ = 0;
  bootstrapRecoverySentInWindow_ = 0;
  lastActiveAmpCount_ = state_.activeAmpCount();
  firstAmpDiscoveryMs_ = 0;
  lastAmpDiscoveryMs_ = 0;
  lastActiveAmpCountChangeMs_ = 0;
  lastRuntimePendingTcpReplies_ = 0;
  lastRuntimePendingCanReads_ = 0;
  lastRuntimeMissingStartupBlockers_ = 0;
  lastRuntimeBootstrapNodesIncomplete_ = 0;
  lastRuntimeR1TimeoutEventsSnapshot_ = 0;
  runtimeStableEntered_ = false;
  gentleStatusShapingRuntimeEnabled_ = ENABLE_GENTLE_STATUS_SHAPING != 0;
  bootstrapProtectionRuntimeEnabled_ = ENABLE_BOOTSTRAP_PROTECTION != 0;
  bootstrapRetryRuntimeEnabled_ = ENABLE_BOOTSTRAP_RETRY != 0;
  segmentedDampeningRuntimeEnabled_ = ENABLE_SEGMENTED_DAMPENING != 0;
  fanoutAttachRuntimeEnabled_ = ENABLE_FANOUT_ATTACH != 0;
  postBootstrapStatusCacheRuntimeEnabled_ = ENABLE_POST_BOOTSTRAP_STATUS_CACHE != 0;
  gentleStatusShapingHasEverBeenActive_ = false;
  gentleStatusShapingLastEnabled_ = false;
  statefulStatusCacheHasEverBeenActive_ = false;
  statefulStatusCacheLastEnabled_ = false;
  lastR1TimeoutEvents_ = 0;
  r1TimeoutEventsAtGentleStatusShapingEnable_ = 0;
  r1TimeoutEventsAtStatefulCacheEnable_ = 0;
  lastChannelIncompleteCount_ = 0;
  lastSubnetServiceGapCount_ = 0;
  statefulStatusCacheDisabledReasonCode_ = kStatefulStatusCacheFlagOff;
  lastSlowOver500Ms_ = 0;
  fairnessLastNodeKey_ = 0xFFFFu;
  fairnessCurrentNodeStreak_ = 0;
  fairnessLastObjectKey_ = 0xFFFFu;
  fairnessCurrentObjectStreak_ = 0;
  slowCanWindow_ = {};
  clearSoftInflightCollapseEntries();
  clearGentleStatusShapingEntries();
  clearDeferredSegmentedRequests();
  clearDeferredIdentRequests();
  clearStatefulStatusCache();
  resetStatefulStatusCacheGroupWindows();
  clearSemanticMirror();
  for (uint16_t index = 0; index < GatewayState::kMaxAddressSlots; ++index) {
    lastNodeCanTxMs_[index] = 0;
  }
  for (uint8_t index = 0; index < kSlowObjectBucketCapacity; ++index) {
    objectFairnessStates_[index] = {};
  }
  for (uint8_t index = 0; index < DebugCounters::kSubnetCount; ++index) {
    lastSummaryIdentRxCountBySubnet_[index] = 0;
  }
  char line[256] = {};
#if R70_EXPERIMENTAL_DEBUG_CODE
  debugCli_.logText("baseline_tag=0.2");
  debugCli_.logText("BASELINE_NAME=beta_0_7_stable_1");
#if R70_BASELINE_0_2_SAFE
  debugCli_.logText("scheduler_mode=SAFE_TRANSPARENT_0_2");
#else
  debugCli_.logText("scheduler_mode=UNSPECIFIED");
#endif
  snprintf(line, sizeof(line), "selected_profile=%s",
           gatewayProfileName(config_.profile));
  debugCli_.logText(line);
  snprintf(line, sizeof(line), "psram_total=%lu psram_free=%lu",
           static_cast<unsigned long>(counters_.psram_total),
           static_cast<unsigned long>(counters_.psram_free));
  debugCli_.logText(line);
#endif
  snprintf(line, sizeof(line),
           "cache_location=%s max_address_slots=%u max_active_amps=%u scalar_fields_per_amp=%u segment_cache_per_amp=%u bytes_allocated=%lu fallback_mode=%s",
           state_.usingPsram() ? "psram" : "internal",
           static_cast<unsigned>(GatewayState::kMaxAddressSlots),
           static_cast<unsigned>(state_.maxActiveAmps()),
           static_cast<unsigned>(state_.fieldCacheCapacityPerAmp()),
           static_cast<unsigned>(state_.segmentCachePerAmp()),
           static_cast<unsigned long>(state_.bytesAllocated()),
           state_.fallbackMode() ? "yes" : "no");
#if R70_EXPERIMENTAL_DEBUG_CODE
  debugCli_.logText(line);
#endif
  snprintf(line,
           sizeof(line),
           "REGRESSION_FLAGS ENABLE_GENTLE_STATUS_SHAPING=%u ENABLE_BOOTSTRAP_PROTECTION=%u ENABLE_BOOTSTRAP_RETRY=%u ENABLE_SEGMENTED_DAMPENING=%u ENABLE_FANOUT_ATTACH=%u ENABLE_POST_BOOTSTRAP_STATUS_CACHE=%u",
           static_cast<unsigned>(gentleStatusShapingRuntimeEnabled_ ? 1u : 0u),
           static_cast<unsigned>(bootstrapProtectionRuntimeEnabled_ ? 1u : 0u),
           static_cast<unsigned>(bootstrapRetryRuntimeEnabled_ ? 1u : 0u),
           static_cast<unsigned>(segmentedDampeningRuntimeEnabled_ ? 1u : 0u),
           static_cast<unsigned>(fanoutAttachRuntimeEnabled_ ? 1u : 0u),
           static_cast<unsigned>(postBootstrapStatusCacheRuntimeEnabled_ ? 1u : 0u));
  debugCli_.logText(line);
  if (counters_.psram_total != 0 && !state_.usingPsram()) {
    snprintf(line, sizeof(line),
             "PSRAM_STATE_CACHE_ALLOC_FAILED fallback_active_amps=%u fallback_scalar_fields_per_amp=%u",
             static_cast<unsigned>(state_.maxActiveAmps()),
             static_cast<unsigned>(state_.fieldCacheCapacityPerAmp()));
    debugCli_.logText(line);
  }
#if R70_EXPERIMENTAL_DEBUG_CODE
  snprintf(line, sizeof(line), "queue_capacities write=%u ident=%u normal=%u",
           static_cast<unsigned>(HotWarmColdQueues::kWriteCapacity),
           static_cast<unsigned>(HotWarmColdQueues::kIdentCapacity),
           static_cast<unsigned>(HotWarmColdQueues::kNormalReadCapacity));
  debugCli_.logText(line);
  snprintf(line, sizeof(line),
           "enableNameSegmentCacheReplies=%s nameSegmentTtlMs=%lu nameSegmentMaxSegments=%u nameSegmentPayloadMaxLen=%u segment_cache_capacity=%lu segment_cache_used=%lu name_cache_slots_capacity=%lu name_cache_slots_used=%lu",
           config_.enableNameSegmentCacheReplies ? "true" : "false",
           static_cast<unsigned long>(nameSegmentCacheTtlMs(config_)),
           static_cast<unsigned>(state_.nameSegmentMaxSegments()),
           static_cast<unsigned>(state_.nameSegmentPayloadMaxLen()),
           static_cast<unsigned long>(counters_.segment_cache_capacity),
           static_cast<unsigned long>(counters_.segment_cache_used),
           static_cast<unsigned long>(counters_.name_cache_slots_capacity),
           static_cast<unsigned long>(counters_.name_cache_slots_used));
  debugCli_.logText(line);
  snprintf(line, sizeof(line),
           "enableFirmwareSegmentCache=%s firmwareSegmentTtlMs=%lu firmwareSegmentMaxSegments=%u firmware_segment_cache_slots_capacity=%lu firmware_segment_cache_slots_used=%lu",
           config_.enableFirmwareSegmentCache ? "true" : "false",
           static_cast<unsigned long>(firmwareSegmentCacheTtlMs(config_)),
           static_cast<unsigned>(config_.firmwareSegmentMaxSegments != 0 ? config_.firmwareSegmentMaxSegments : GatewayState::kNameSegmentMaxSegments),
           static_cast<unsigned long>(counters_.firmware_segment_cache_slots_capacity),
           static_cast<unsigned long>(counters_.firmware_segment_cache_slots_used));
  debugCli_.logText(line);
  snprintf(line, sizeof(line),
           "enableUnknown0111ExactCache=%s unknown0111TtlMs=%lu enableUnknown0111ExactCacheDebug=%s",
           config_.enableUnknown0111ExactCache ? "true" : "false",
           static_cast<unsigned long>(config_.unknown0111TtlMs != 0 ? config_.unknown0111TtlMs : 500),
           config_.enableUnknown0111ExactCacheDebug ? "true" : "false");
  debugCli_.logText(line);
  snprintf(line, sizeof(line),
           "enableCriticalStatusCache=%s watchdogStatusTtlMs=%lu ispTtlMs=%lu criticalStatusTtlMs=%lu enableCriticalStatusTtlExperiment=%s criticalStatusTtlExperimentMs=%lu criticalStatusTtlPreset=%lu enableConfigScalarTtlExperiment=%s configScalarTtlMs=%lu configScalarTtlExperimentMs=%lu configScalarTtlPreset=%lu enableStableSetupCache=%s enableSpeakerIdCache=%s enableInputModeCache=%s enableDataLinkPresetCache=%s enableSmallStatusCacheExperiment=%s",
           config_.enableCriticalStatusCache ? "true" : "false",
           static_cast<unsigned long>(config_.watchdogStatusTtlMs != 0 ? config_.watchdogStatusTtlMs : 1000),
           static_cast<unsigned long>(config_.ispTtlMs != 0 ? config_.ispTtlMs : (config_.criticalStatusTtlMs != 0 ? config_.criticalStatusTtlMs : 750)),
           static_cast<unsigned long>(config_.criticalStatusTtlMs != 0 ? config_.criticalStatusTtlMs : 500),
           config_.enableCriticalStatusTtlExperiment ? "true" : "false",
           static_cast<unsigned long>(config_.criticalStatusTtlExperimentMs != 0 ? config_.criticalStatusTtlExperimentMs : 1000),
           static_cast<unsigned long>(config_.criticalStatusTtlPreset != 0 ? config_.criticalStatusTtlPreset : (config_.enableCriticalStatusTtlExperiment ? (config_.criticalStatusTtlMs != 0 ? config_.criticalStatusTtlMs : 750) : 500)),
           config_.enableConfigScalarTtlExperiment ? "true" : "false",
           static_cast<unsigned long>(config_.configScalarTtlMs != 0 ? config_.configScalarTtlMs : config_.cacheTtlDynamicMs),
           static_cast<unsigned long>(config_.configScalarTtlExperimentMs != 0 ? config_.configScalarTtlExperimentMs : 1500),
           static_cast<unsigned long>(config_.configScalarTtlPreset != 0 ? config_.configScalarTtlPreset : (config_.enableConfigScalarTtlExperiment ? (config_.configScalarTtlMs != 0 ? config_.configScalarTtlMs : config_.cacheTtlDynamicMs) : config_.cacheTtlDynamicMs)),
           config_.enableStableSetupCache ? "true" : "false",
           config_.enableSpeakerIdCache ? "true" : "false",
           config_.enableInputModeCache ? "true" : "false",
           config_.enableDataLinkPresetCache ? "true" : "false",
           config_.enableSmallStatusCacheExperiment ? "true" : "false");
  debugCli_.logText(line);
  snprintf(line, sizeof(line),
           "enableVerboseFrameLog=%s enableTimeoutCorrelation=%s enableAutoGroupTimeoutSnapshot=%s enablePerNodeRecentRequestRing=%s enableLowRateSummary=%s",
           config_.enableVerboseFrameLog ? "true" : "false",
           config_.enableTimeoutCorrelation ? "true" : "false",
           config_.enableAutoGroupTimeoutSnapshot ? "true" : "false",
           config_.enablePerNodeRecentRequestRing ? "true" : "false",
           config_.enableLowRateSummary ? "true" : "false");
  debugCli_.logText(line);
#endif
  debugCli_.logText("R70_EMULATOR_V2_READY");
  return true;
}

void R70GatewayV2::poll() {
  if (!started_) {
    return;
  }

  const uint32_t pollStartUs = micros();
  if (lastPollMicros_ != 0) {
    const uint32_t loopGapUs = pollStartUs - lastPollMicros_;
    if (loopGapUs > loopMaxUsWindow_) {
      loopMaxUsWindow_ = loopGapUs;
    }
    loopSumUsWindow_ += loopGapUs;
    ++loopSamplesWindow_;
  }
  lastPollMicros_ = pollStartUs;
  const uint32_t nowMs = millis();
  expireSoftInflightCollapse(nowMs);
  pollCli(nowMs);
  state_.tick(nowMs, counters_);
  discovery_.poll(config_.identity, counters_, debugCli_);
  if (static_cast<int32_t>(nowMs - (tcpBackpressureWindowStartMs_ + 1000)) >= 0) {
    tcpBackpressureWindowStartMs_ = nowMs;
    tcpBackpressureWindowUsedMs_ = 0;
    tcpBackpressureTotalLimited_ = false;
  }
  const uint16_t queueDepthBeforeTcp = scheduler_.queueDepth();
  bool allowTcpRead = true;
  if (tcpBackpressureActive_ && queueDepthBeforeTcp <= canTxQueueLowWater_) {
    tcpBackpressureActive_ = false;
    const uint32_t activeMs = nowMs - tcpBackpressureStartMs_;
    counters_.tcp_backpressure_active_ms += activeMs;
    tcpBackpressureWindowUsedMs_ += activeMs;
    ++counters_.tcp_backpressure_off_count;
    debugCli_.logTcpBackpressureOff(queueDepthBeforeTcp, canTxQueueLowWater_, activeMs);
    tcpBackpressureStartMs_ = 0;
    tcpBackpressureBurstLimited_ = false;
  }
  if (queueDepthBeforeTcp < canTxQueueHighWater_) {
    tcpBackpressureBurstLimited_ = false;
  }
  if (config_.enableEmergencyTcpBackpressure && queueDepthBeforeTcp >= canTxQueueHighWater_) {
    if (!tcpBackpressureActive_) {
      if (config_.maxTcpBackpressureTotalPerSecondMs != 0 &&
          tcpBackpressureWindowUsedMs_ >= config_.maxTcpBackpressureTotalPerSecondMs) {
        if (!tcpBackpressureTotalLimited_) {
          ++counters_.tcp_backpressure_limited_count;
          debugCli_.logTcpBackpressureLimited("max_total_per_second");
          tcpBackpressureTotalLimited_ = true;
        }
      } else {
        tcpBackpressureActive_ = true;
        tcpBackpressureStartMs_ = nowMs;
        ++counters_.tcp_backpressure_on_count;
        debugCli_.logTcpBackpressureOn(queueDepthBeforeTcp, canTxQueueHighWater_);
      }
    }
    if (tcpBackpressureActive_) {
      const uint32_t burstMs = nowMs - tcpBackpressureStartMs_;
      const uint32_t activeTotalMs = tcpBackpressureWindowUsedMs_ + burstMs;
      const bool burstExceeded =
          config_.maxTcpBackpressureBurstMs != 0 &&
          burstMs >= config_.maxTcpBackpressureBurstMs;
      const bool totalExceeded =
          config_.maxTcpBackpressureTotalPerSecondMs != 0 &&
          activeTotalMs >= config_.maxTcpBackpressureTotalPerSecondMs;
      if (burstExceeded || totalExceeded) {
        tcpBackpressureActive_ = false;
        counters_.tcp_backpressure_active_ms += burstMs;
        tcpBackpressureWindowUsedMs_ += burstMs;
        ++counters_.tcp_backpressure_off_count;
        debugCli_.logTcpBackpressureOff(queueDepthBeforeTcp, canTxQueueLowWater_, burstMs);
        ++counters_.tcp_backpressure_limited_count;
        debugCli_.logTcpBackpressureLimited(burstExceeded ? "max_burst_ms" : "max_total_per_second");
        if (burstExceeded) {
          tcpBackpressureBurstLimited_ = true;
        }
        if (totalExceeded) {
          tcpBackpressureTotalLimited_ = true;
        }
        tcpBackpressureStartMs_ = 0;
      } else {
        allowTcpRead = false;
      }
    }
  }
  counters_.tcp_backpressure_active = tcpBackpressureActive_ ? 1u : 0u;
  const uint32_t tcpPollUs = micros();
  if (lastTcpPollMicros_ != 0) {
    const uint32_t tcpGapUs = tcpPollUs - lastTcpPollMicros_;
    if (tcpGapUs > tcpPollMaxGapUsWindow_) {
      tcpPollMaxGapUsWindow_ = tcpGapUs;
    }
  }
  lastTcpPollMicros_ = tcpPollUs;
  const bool tcpConnectedBeforePoll = tcpServer_.connected();
  tcpServer_.poll(counters_, debugCli_, allowTcpRead);
  const bool tcpConnectedAfterPoll = tcpServer_.connected();
  if (tcpConnectedAfterPoll != lastTcpConnected_ || tcpConnectedBeforePoll != tcpConnectedAfterPoll) {
    clearGentleStatusShapingEntries();
    clearSoftInflightCollapseEntries();
    clearDeferredSegmentedRequests();
    clearDeferredIdentRequests();
    clearStatefulStatusCache();
    clearSemanticMirror();
    clearPresenceRounds();
  }
  if (!tcpConnectedBeforePoll && tcpConnectedAfterPoll) {
    r1ConnectedSinceMs_ = nowMs;
    restartBootstrapPhase(nowMs, true);
  } else if (!tcpConnectedAfterPoll) {
    r1ConnectedSinceMs_ = 0;
  }
  lastTcpConnected_ = tcpConnectedAfterPoll;

  for (uint8_t index = 0; index < 4; ++index) {
    TcpEnvelope envelope = {};
    uint8_t raw[R70Frame::kMaxFrameLength] = {};
    uint8_t rawLength = 0;
    if (!tcpServer_.nextFrame(envelope, raw, rawLength, counters_)) {
      break;
    }

    if (envelope.checksumChecked && !envelope.checksumValid) {
      debugCli_.logChecksumMismatch(envelope);
    }

    NormalizedRequest request = {};
    if (!codec::normalizeTcpEnvelope(envelope, request)) {
      ++counters_.tcp_frames_unknown;
      ++counters_.tcp_unknown_frames;
      debugCli_.logTcpFrame("TCP_CONTROL_UNKNOWN ", raw, rawLength);
      continue;
    }

    if (request.tcpWrappedCan) {
      RequestTrace* trace = 0;
      state_.noteTcpRxRequestForNode(request.canId.subnet, request.canId.device, request.fieldId, nowMs);
      noteBootstrapRequestObserved(request, nowMs);
      noteStartupBlockerRequestObserved(request, nowMs);
      if (isReadFunction(request.admin.functionCode)) {
        // 0111 belongs to the R1-facing alive/status cohort. Never defer or
        // collapse it ahead of its byte-exact cache path below: doing so can
        // leave the dedicated 0111 pending accounting open even after a
        // generic mirror reply was sent.
        const bool exact0111Read = isUnknown0111ReadRequest(request);
        if (!exact0111Read && gentleStatusShapingFeatureEnabled() &&
            tryDeferGentleStatusShaping(request, nowMs)) {
          noteObjectTcpRead(request, nowMs);
          ++counters_.allowlist_hits;
          continue;
        }
        const uint8_t duplicatePendingCount = countPendingExactDuplicates(request);
        const bool burstRepeat = hasRecentExactBurst(request, nowMs, kSoftInflightBurstWindowMs);
        const bool segmentedActiveRepeat = isNameSegmentReadRequest(request) && duplicatePendingCount != 0;
        noteObjectTcpRead(request, nowMs);
        noteExpensiveObjectRequest(request, nowMs, duplicatePendingCount, burstRepeat, segmentedActiveRepeat);
        if (!exact0111Read && fanoutAttachFeatureEnabled() &&
            tryAttachSoftInflightCollapse(request, nowMs)) {
          ++counters_.allowlist_hits;
          continue;
        }
        trace = recordRequestTrace(request,
                                   nowMs,
                                   pendingTcpReplyCount(),
                                   scheduler_.normalQueueDepth(),
                                   scheduler_.queueDepth(),
                                   duplicatePendingCount);
        noteSemanticFamilyRequested(request, nowMs);
        if (exact0111Read) {
          state_.noteUnknown0111Request(request.canId.subnet, request.canId.device, nowMs);
          ++counters_.unknown_0111_read_requests;
          ++counters_.unknown_0111_pending;
        }
        if (!exact0111Read && tryHandleSemanticMirrorRequest(request, trace, nowMs)) {
          continue;
        }
        if (!exact0111Read && tryHandlePostBootstrapStatusCacheRequest(request, trace, nowMs)) {
          continue;
        }
      }
      if (isFirmwareSegmentReadRequest(request)) {
        ++counters_.firmware_segment_read_requests;
        Frame cachedReplyFrame = {};
        uint32_t cacheAgeMs = 0;
        NameSegmentCacheMissReason candidateReason = NameSegmentCacheMissReason::NoField;
        if (state_.buildNameSegmentCacheReply(request,
                                              0,
                                              nowMs,
                                              cachedReplyFrame,
                                              cacheAgeMs,
                                              candidateReason)) {
          ++counters_.firmware_segment_cache_candidate_hits_possible;
        }
      }
      if (isStableSetupReadRequest(request)) {
        if (isSpeakerIdField(request.fieldId)) {
          ++counters_.speaker_id_read_requests;
        } else if (isInputModeField(request.fieldId)) {
          ++counters_.input_mode_read_requests;
        } else if (isDataLinkPresetField(request.fieldId)) {
          ++counters_.datalink_preset_read_requests;
        }
        if (isDataLinkPresetField(request.fieldId)) {
          Frame cachedReplyFrame = {};
          uint32_t cacheAgeMs = 0;
          bool cacheStale = false;
          if (state_.buildObservedFrameCacheReply(request, 0, nowMs, cachedReplyFrame, cacheAgeMs, cacheStale)) {
            ++counters_.datalink_preset_cache_candidate_hits_possible;
          }
        }
      }
      if (isUnknown0111ReadRequest(request) &&
          isWriteFunction(request.admin.functionCode)) {
        state_.invalidateUnknown0111Cache(request.canId.subnet, request.canId.device);
        ++counters_.unknown_0111_cache_invalidations;
      }
      if (isNameSegmentWriteRequest(request)) {
        state_.invalidateNameSegmentCache(request.canId.subnet,
                                          request.canId.device,
                                          request.fieldId,
                                          "write",
                                          counters_,
                                          &debugCli_);
        debugCli_.logNameCacheInvalidate(request.canId.subnet, request.canId.device,
                                         request.fieldId, "write");
      }
      if (isStableSetupWriteRequest(request)) {
        state_.invalidateFieldCache(request.canId.subnet, request.canId.device, request.fieldId);
        ++counters_.stable_setup_cache_invalidations;
        if (isSpeakerIdField(request.fieldId)) {
          ++counters_.speaker_id_cache_invalidations;
        } else if (isInputModeField(request.fieldId)) {
          ++counters_.input_mode_cache_invalidations;
        }
        debugCli_.logStableSetupCacheInvalidate(request.canId.subnet,
                                                request.canId.device,
                                                request.fieldId,
                                                "write");
      }
      if (request.valid &&
          isWriteFunction(request.admin.functionCode) &&
          request.canId.device != 0) {
        invalidateStatefulStatusCache(request.canId.subnet,
                                      request.canId.device,
                                      request.objectAddress.channelNumber);
        if (!updateSemanticMirrorOptimisticallyForWrite(request, nowMs)) {
          invalidateSemanticMirrorForWrite(request, "write");
        }
      }
      if (request.valid &&
          isWriteFunction(request.admin.functionCode) &&
          request.admin.telegramType == TelegramType::Unstructured &&
          isConfigScalarTtlField(request.fieldId)) {
        state_.invalidateFieldCache(request.canId.subnet, request.canId.device, request.fieldId);
      }
      if (isFirmwareSegmentReadRequest(request)) {
        NameSegmentCacheMissReason missReason = NameSegmentCacheMissReason::Disabled;
        Frame cachedReply = {};
        uint32_t cacheAgeMs = 0;
        if (config_.enableFirmwareSegmentCache &&
            state_.buildNameSegmentCacheReply(request,
                                              firmwareSegmentCacheTtlMs(config_),
                                              nowMs,
                                              cachedReply,
                                              cacheAgeMs,
                                              missReason,
                                              kNodeFreshMirrorServeMaxAgeMs,
                                              true)) {
          uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
          uint8_t tcpFrameLength = 0;
          if (R70Frame::encodeWrappedCanFrame(cachedReply, tcpFrame, sizeof(tcpFrame), tcpFrameLength) &&
              tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
            ++counters_.cache_hits;
            ++counters_.cache_replies_sent;
            ++counters_.firmware_segment_cache_replies;
            ++counters_.allowlist_hits;
            if (shouldLogLowRateCounter(counters_.firmware_segment_cache_replies)) {
              debugCli_.logFirmwareSegmentCacheReply(request.canId.subnet,
                                                     request.canId.device,
                                                     request.objectAddress.segmentNumber,
                                                     cachedReply,
                                                     cacheAgeMs);
            }
            debugCli_.logTcpTxWrappedCan(cachedReply, tcpFrame, tcpFrameLength);
            state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
            noteSemanticFamilyUpdated(request, nowMs, 3u, true);
            noteStartupBlockerReplyForwarded(request, nowMs);
            markTraceCacheReply(trace, nowMs);
            noteObjectCacheReply(request, nowMs);
            continue;
          }
        } else {
          ++counters_.firmware_segment_cache_miss;
          if (missReason == NameSegmentCacheMissReason::TtlExpired) {
            ++counters_.firmware_segment_cache_stale;
            noteObjectCacheStale(request);
          } else {
            noteObjectCacheMiss(request);
          }
          ++counters_.cache_misses;
        }
      } else if (isNameSegmentReadRequest(request)) {
        ++counters_.name_segment_cache_read_requests;
        NameSegmentCacheMissReason missReason = NameSegmentCacheMissReason::Disabled;
        Frame cachedReply = {};
        uint32_t cacheAgeMs = 0;
        if (config_.enableNameSegmentCacheReplies &&
            state_.buildNameSegmentCacheReply(request,
                                              nameSegmentCacheTtlMs(config_),
                                              nowMs,
                                              cachedReply,
                                              cacheAgeMs,
                                              missReason,
                                              kNodeFreshMirrorServeMaxAgeMs,
                                              true)) {
          uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
          uint8_t tcpFrameLength = 0;
          if (R70Frame::encodeWrappedCanFrame(cachedReply, tcpFrame, sizeof(tcpFrame), tcpFrameLength) &&
              tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
            ++counters_.cache_hits;
            ++counters_.cache_replies_sent;
            ++counters_.name_segment_cache_replies;
            ++counters_.cache_hit_names;
            ++counters_.allowlist_hits;
            if (shouldLogLowRateCounter(counters_.name_segment_cache_replies)) {
              debugCli_.logNameSegmentCacheReply(request.canId.subnet,
                                                 request.canId.device,
                                                 request.fieldId,
                                                 request.objectAddress.segmentNumber,
                                                 cachedReply,
                                                 cacheAgeMs);
            }
            debugCli_.logTcpTxWrappedCan(cachedReply, tcpFrame, tcpFrameLength);
            state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
            noteSemanticFamilyUpdated(request, nowMs, 3u, true);
            noteStartupBlockerReplyForwarded(request, nowMs);
            markTraceCacheReply(trace, nowMs);
            noteObjectCacheReply(request, nowMs);
            continue;
          }
        } else {
          if (!config_.enableNameSegmentCacheReplies) {
            missReason = NameSegmentCacheMissReason::Disabled;
          }
          noteNameSegmentMiss(counters_, missReason);
          ++counters_.cache_misses;
          if (shouldLogLowRateCounter(counters_.name_segment_cache_miss)) {
            debugCli_.logNameSegmentCacheMiss(request.canId.subnet,
                                              request.canId.device,
                                              request.fieldId,
                                              request.objectAddress.segmentNumber,
                                              nameSegmentMissReasonText(missReason));
          }
          if (missReason == NameSegmentCacheMissReason::TtlExpired) {
            noteObjectCacheStale(request);
          } else {
            noteObjectCacheMiss(request);
          }
        }
      } else if (isUnknown0111ReadRequest(request)) {
        Frame cachedReply = {};
        uint32_t cacheAgeMs = 0;
        bool cacheStale = false;
        if (config_.enableUnknown0111ExactCache &&
            state_.buildUnknown0111CacheReply(request.canId.subnet,
                                              request.canId.device,
                                              config_.unknown0111TtlMs != 0 ? config_.unknown0111TtlMs : 500,
                                              nowMs,
                                              cachedReply,
                                              cacheAgeMs,
                                              cacheStale,
                                              kNodeFreshMirrorServeMaxAgeMs)) {
          uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
          uint8_t tcpFrameLength = 0;
          if (R70Frame::encodeWrappedCanFrame(cachedReply, tcpFrame, sizeof(tcpFrame), tcpFrameLength) &&
              tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
            ++counters_.cache_hits;
            ++counters_.cache_replies_sent;
            ++counters_.unknown_0111_cache_replies;
            ++counters_.allowlist_hits;
            debugCli_.logTcpTxWrappedCan(cachedReply, tcpFrame, tcpFrameLength);
            state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
            noteSemanticFamilyUpdated(request, nowMs, 3u, true);
            noteStartupBlockerReplyForwarded(request, nowMs);
            state_.noteUnknown0111TcpTx(request.canId.subnet,
                                        request.canId.device,
                                        unknown0111Value(cachedReply),
                                        nowMs);
            ++counters_.unknown_0111_tcp_tx;
            if (counters_.unknown_0111_pending != 0) {
              --counters_.unknown_0111_pending;
            }
            markTraceCacheReply(trace, nowMs);
            noteObjectCacheReply(request, nowMs);
            if (config_.enableVerboseFrameLog &&
                config_.enableUnknown0111ExactCacheDebug &&
                shouldLogLowRateCounter(counters_.unknown_0111_cache_replies)) {
              char line[160] = {};
              snprintf(line, sizeof(line),
                       "T=%lums UNKNOWN_0111_CACHE_REPLY node=%u.%02u age_ms=%lu data=%02X %02X %02X %02X",
                       static_cast<unsigned long>(nowMs),
                       static_cast<unsigned>(request.canId.subnet),
                       static_cast<unsigned>(request.canId.device),
                       static_cast<unsigned long>(cacheAgeMs),
                       static_cast<unsigned>(cachedReply.data[0]),
                       static_cast<unsigned>(cachedReply.data[1]),
                       static_cast<unsigned>(cachedReply.data[2]),
                       static_cast<unsigned>(cachedReply.length >= 4 ? cachedReply.data[3] : 0));
              debugCli_.logText(line);
            }
            queueUnknown0111Refresh(request, nowMs);
            continue;
          }
        } else {
          ++counters_.unknown_0111_cache_miss;
          ++counters_.cache_misses;
          if (cacheStale) {
            ++counters_.unknown_0111_cache_stale;
            markTraceStaleAtRequest(trace);
            noteObjectCacheStale(request);
            if (config_.enableVerboseFrameLog &&
                config_.enableUnknown0111ExactCacheDebug &&
                shouldLogLowRateCounter(counters_.unknown_0111_cache_stale)) {
              char line[160] = {};
              snprintf(line, sizeof(line),
                       "T=%lums UNKNOWN_0111_CACHE_STALE node=%u.%02u age_ms=%lu ttl_ms=%lu",
                       static_cast<unsigned long>(nowMs),
                       static_cast<unsigned>(request.canId.subnet),
                       static_cast<unsigned>(request.canId.device),
                       static_cast<unsigned long>(cacheAgeMs),
                       static_cast<unsigned long>(config_.unknown0111TtlMs != 0 ? config_.unknown0111TtlMs : 500));
              debugCli_.logText(line);
            }
          } else {
            noteObjectCacheMiss(request);
            if (config_.enableVerboseFrameLog &&
                config_.enableUnknown0111ExactCacheDebug &&
                shouldLogLowRateCounter(counters_.unknown_0111_cache_miss)) {
              char line[160] = {};
              snprintf(line, sizeof(line),
                       "T=%lums UNKNOWN_0111_CACHE_MISS node=%u.%02u reason=not_learned",
                       static_cast<unsigned long>(nowMs),
                       static_cast<unsigned>(request.canId.subnet),
                       static_cast<unsigned>(request.canId.device));
              debugCli_.logText(line);
            }
          }
        }
      } else if (isStableSetupReadRequest(request)) {
        const bool cacheEnabled = isStableSetupCacheEnabled(config_, request.fieldId);
        Frame cachedReply = {};
        uint32_t cacheAgeMs = 0;
        bool cacheStale = false;
        if (cacheEnabled &&
            state_.buildObservedFrameCacheReply(request,
                                                stableSetupTtlMs(config_, request.fieldId),
                                                nowMs,
                                                cachedReply,
                                                cacheAgeMs,
                                                cacheStale,
                                                kNodeFreshMirrorServeMaxAgeMs)) {
          uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
          uint8_t tcpFrameLength = 0;
          if (R70Frame::encodeWrappedCanFrame(cachedReply, tcpFrame, sizeof(tcpFrame), tcpFrameLength) &&
              tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
            ++counters_.cache_hits;
            ++counters_.cache_replies_sent;
            ++counters_.stable_setup_cache_replies;
            ++counters_.allowlist_hits;
            if (isSpeakerIdField(request.fieldId)) {
              ++counters_.speaker_id_cache_replies;
            } else if (isInputModeField(request.fieldId)) {
              ++counters_.input_mode_cache_replies;
            } else if (isDataLinkPresetField(request.fieldId)) {
              ++counters_.datalink_preset_cache_replies;
            }
            noteCacheHitByField(request.fieldId, counters_);
            debugCli_.logStableSetupCacheReply(request.canId.subnet,
                                               request.canId.device,
                                               request.fieldId,
                                               cachedReply,
                                               cacheAgeMs);
            debugCli_.logTcpTxWrappedCan(cachedReply, tcpFrame, tcpFrameLength);
            state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
            noteSemanticFamilyUpdated(request, nowMs, 3u, true);
            noteStartupBlockerReplyForwarded(request, nowMs);
            markTraceCacheReply(trace, nowMs);
            noteObjectCacheReply(request, nowMs);
            continue;
          }
        } else if (cacheEnabled) {
          ++counters_.stable_setup_cache_miss;
          ++counters_.cache_misses;
          if (cacheStale) {
            ++counters_.stable_setup_cache_stale;
            noteObjectCacheStale(request);
            if (isSpeakerIdField(request.fieldId)) {
              ++counters_.speaker_id_cache_stale;
            } else if (isInputModeField(request.fieldId)) {
              ++counters_.input_mode_cache_stale;
            }
          } else {
            noteObjectCacheMiss(request);
            if (isSpeakerIdField(request.fieldId)) {
              ++counters_.speaker_id_cache_miss;
            } else if (isInputModeField(request.fieldId)) {
              ++counters_.input_mode_cache_miss;
            }
          }
        }
      } else if (isReadFunction(request.admin.functionCode) && !canAttemptConfirmedReadCache(config_, request)) {
        ++counters_.cache_lookup_skipped_label_only;
      } else if (canAttemptConfirmedReadCache(config_, request)) {
        const uint32_t ttlMs = confirmedReadCacheTtlMs(config_, request);
        Frame cachedReply = {};
        uint32_t cacheAgeMs = 0;
        bool cacheStale = false;
        if (state_.buildCacheReply(request,
                                   ttlMs,
                                   nowMs,
                                   cachedReply,
                                   cacheAgeMs,
                                   cacheStale,
                                   kNodeFreshMirrorServeMaxAgeMs)) {
          uint8_t tcpFrame[R70Frame::kMaxFrameLength] = {};
          uint8_t tcpFrameLength = 0;
          if (R70Frame::encodeWrappedCanFrame(cachedReply, tcpFrame, sizeof(tcpFrame), tcpFrameLength) &&
              tcpServer_.sendFrame(tcpFrame, tcpFrameLength, &counters_)) {
            ++counters_.cache_hits;
            ++counters_.cache_replies_sent;
            ++counters_.allowlist_hits;
            if (isConfigScalarTtlField(request.fieldId)) {
              ++counters_.config_scalar_cache_replies;
            }
            if (isWatchdogStatusCacheField(request.fieldId)) {
              ++counters_.watchdog_status_cache_replies;
            }
            if (isCriticalStatusCacheField(request.fieldId)) {
              ++counters_.critical_status_cache_replies;
            }
            noteCacheHitByField(request.fieldId, counters_);
            debugCli_.logCacheReply(request, cachedReply, cacheAgeMs, ttlMs);
            debugCli_.logTcpTxWrappedCan(cachedReply, tcpFrame, tcpFrameLength);
            state_.noteCacheReplyForNode(request.canId.subnet, request.canId.device, nowMs);
            noteSemanticFamilyUpdated(request, nowMs, 3u, true);
            noteStartupBlockerReplyForwarded(request, nowMs);
            markTraceCacheReply(trace, nowMs);
            noteObjectCacheReply(request, nowMs);
            continue;
          }
        } else if (cacheStale) {
          markTraceStaleAtRequest(trace);
          ++counters_.cache_reply_stale;
          if (isConfigScalarTtlField(request.fieldId)) {
            ++counters_.config_scalar_cache_stale;
          }
          if (isWatchdogStatusCacheField(request.fieldId)) {
            ++counters_.watchdog_status_cache_stale;
          }
          if (isCriticalStatusCacheField(request.fieldId)) {
            ++counters_.critical_status_cache_stale;
          }
          noteObjectCacheStale(request);
        } else {
          ++counters_.cache_reply_ineligible;
          if (isConfigScalarTtlField(request.fieldId)) {
            ++counters_.config_scalar_cache_miss;
          }
          if (isWatchdogStatusCacheField(request.fieldId)) {
            ++counters_.watchdog_status_cache_miss;
          }
          if (isCriticalStatusCacheField(request.fieldId)) {
            ++counters_.critical_status_cache_misses;
          }
          noteObjectCacheMiss(request);
        }
        ++counters_.cache_misses;
      }
      if (isSegmentedReadRequest(request) && tryQueueDeferredSegmentedRequest(request, nowMs)) {
        continue;
      }
      if (tryQueueDeferredIdentRequest(request, nowMs)) {
        continue;
      }
      const char* failureReason = 0;
      const uint32_t coalescedBefore = counters_.read_coalesced_duplicate_count;
      bool bootstrapPriorityRequest = false;
      if (bootstrapProtectionFeatureEnabled() &&
          request.canId.device != 0 &&
          isReadFunction(request.admin.functionCode)) {
        AmpState* ampState = state_.amp(request.canId.subnet, request.canId.device, false);
        uint8_t bootstrapIndex = 0;
        if (ampState != 0 &&
            ampState->used &&
            (bootstrapProtectionActive(nowMs) || powerOnBootstrapActive(*ampState, nowMs)) &&
            classifyBootstrapReply(request, bootstrapIndex) &&
            (bootstrapRequiredMask(*ampState) & bootstrapMaskFor(bootstrapIndex)) != 0) {
          bootstrapPriorityRequest = true;
        }
      }
      if (!bootstrapPriorityRequest && isStartupFreshnessPriorityRequest(request, nowMs)) {
        bootstrapPriorityRequest = true;
      }
      const bool enqueued =
          bootstrapPriorityRequest
              ? scheduler_.enqueueBootstrapRetry(request, nowMs, counters_, failureReason)
              : scheduler_.enqueue(request, 0, false, nowMs, counters_, failureReason);
      if (enqueued) {
        ++counters_.allowlist_hits;
        debugCli_.logTcpCanTx(request);
        markTraceForwardedToCan(trace);
        if (fanoutAttachFeatureEnabled() || segmentedDampeningFeatureEnabled()) {
          noteSoftInflightPrimary(request, nowMs);
        }
        if (isUnknown0111ReadRequest(request)) {
          state_.noteUnknown0111CanTx(request.canId.subnet, request.canId.device, nowMs);
          ++counters_.unknown_0111_can_tx;
        }
        if (counters_.read_coalesced_duplicate_count != coalescedBefore) {
          noteObjectCoalesced(request);
          noteExpensiveObjectCoalesced(request);
        } else if (isReadFunction(request.admin.functionCode)) {
          noteObjectCanTxForwarded(request);
        }
      } else {
        debugCli_.logTcpCanTxFail(request, failureReason);
        if (ActiveShapingCadenceState* cadence = activeShapingCadenceState(request)) {
          if (cadence->lastCanTxMs == 0) {
            *cadence = {};
          }
        }
        if (failureReason != 0 && strcmp(failureReason, "queue_full") == 0) {
          noteObjectQueueFull(request);
        }
      }
      continue;
    }

    if (request.mapping != 0) {
      if (request.mapping->isSpecOnly) {
        ++counters_.tcp_labeled_spec_only;
      }
      if (request.mapping->isPdfXlsOnly) {
        ++counters_.tcp_labeled_pdfxls_only;
      }
    }

    if (config_.enableVerboseFrameLog) {
      debugCli_.logCanFrame("TCP_REQ", request.frame, &request);
    }

    if (isReadFunction(request.admin.functionCode)) {
      ++counters_.cache_misses;
      const char* failureReason = 0;
      if (runtimeReadable(request.mapping) &&
          scheduler_.enqueue(request, request.tcpNode, true, nowMs, counters_, failureReason)) {
        ++counters_.allowlist_hits;
      } else if (!runtimeReadable(request.mapping)) {
        ++counters_.allowlist_misses;
      } else {
        debugCli_.logTcpCanTxFail(request, failureReason);
      }
      continue;
    }

    if (isWriteFunction(request.admin.functionCode)) {
      const char* failureReason = 0;
      if (runtimeWritable(request.mapping) &&
          scheduler_.enqueue(request, request.tcpNode, false, nowMs, counters_, failureReason)) {
        ++counters_.allowlist_hits;
      } else if (!runtimeWritable(request.mapping)) {
        ++counters_.allowlist_misses;
      } else {
        debugCli_.logTcpCanTxFail(request, failureReason);
      }
      continue;
    }

    ++counters_.allowlist_misses;
  }

  const uint8_t activeAmpCountNow = state_.activeAmpCount();
  if (activeAmpCountNow != lastActiveAmpCount_) {
    resetRuntimeStabilityTracking();
    lastActiveAmpCountChangeMs_ = nowMs;
    if (firstAmpDiscoveryMs_ == 0) {
      firstAmpDiscoveryMs_ = nowMs;
    }
    bootstrapProtectionUntilMs_ =
        bootstrapProtectionFeatureEnabled() ? (nowMs + kBootstrapProtectionMs) : 0;
    bootstrapCompleteStableSinceMs_ = 0;
    bootstrapRecoveryWindowStartedMs_ = 0;
    bootstrapRecoverySentInWindow_ = 0;
    clearGentleStatusShapingEntries();
    clearSoftInflightCollapseEntries();
    clearDeferredSegmentedRequests();
    clearDeferredIdentRequests();
    clearStatefulStatusCache();
    clearSemanticMirror();
  }
  if (activeAmpCountNow > lastActiveAmpCount_) {
    lastAmpDiscoveryMs_ = nowMs;
    for (uint8_t ampIndex = 0; ampIndex < activeAmpCountNow; ++ampIndex) {
      const AmpState* activeAmp = state_.activeAmpByIndex(ampIndex);
      if (activeAmp == 0) {
        continue;
      }
      AmpState* ampState = state_.amp(activeAmp->subnet, activeAmp->device, false);
      if (ampState != 0 && ampState->bootstrapStartedMs == 0) {
        ampState->bootstrapStartedMs = nowMs;
      }
    }
  }
  lastActiveAmpCount_ = activeAmpCountNow;

  if (gentleStatusShapingFeatureEnabled()) {
    processGentleStatusShaping(nowMs);
  } else {
    clearGentleStatusShapingEntries();
  }

  const uint32_t canPollUs = micros();
  if (lastCanPollMicros_ != 0) {
    const uint32_t canGapUs = canPollUs - lastCanPollMicros_;
    if (canGapUs > canPollMaxGapUsWindow_) {
      canPollMaxGapUsWindow_ = canGapUs;
    }
  }
  lastCanPollMicros_ = canPollUs;
  for (uint8_t index = 0; index < 8; ++index) {
    Frame frame = {};
    if (!canTransport_.receive(frame, 0)) {
      break;
    }

    ++counters_.can_rx_frames;
    if (frame.canId == 0x441) {
      ++counters_.can_rx_id_0x441;
    } else if (frame.canId == 0x442) {
      ++counters_.can_rx_id_0x442;
    }
    NormalizedRequest reply = {};
    if (!codec::normalizeCanFrame(frame, RequestOrigin::Can, reply)) {
      if (frame.length >= 3) {
        const uint16_t rawObjectKey =
            static_cast<uint16_t>((static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2]);
        noteSemanticCandidate(static_cast<uint8_t>((frame.canId >> 6) & 0x07u),
                              static_cast<uint8_t>(frame.canId & 0x3Fu),
                              0xFFu,
                              rawObjectKey,
                              frame.data,
                              frame.length,
                              nowMs,
                              false,
                              "not_normalized");
        if (isSemanticMirrorFocusObjectKey(rawObjectKey)) {
          ++semanticMirrorWindow_.learnSkips;
          if (shouldLogLowRateCounter(semanticMirrorWindow_.learnSkips)) {
            char line[192] = {};
            snprintf(line,
                     sizeof(line),
                     "MIRROR_LEARN_SKIP node=%u.%02u object=%04X raw_object=%04X reason=not_normalized",
                     static_cast<unsigned>((frame.canId >> 6) & 0x07u),
                     static_cast<unsigned>(frame.canId & 0x3Fu),
                     static_cast<unsigned>(semanticMirrorDisplayObjectKey(rawObjectKey)),
                     static_cast<unsigned>(rawObjectKey));
            debugCli_.logText(line);
          }
        }
      }
      noteCanBusFrame(nowMs, 0xFFu, false, frame.length);
      if (config_.enableVerboseFrameLog) {
        debugCli_.logCanFrame("CAN_RX_UNKNOWN", frame, 0);
      }
      continue;
    }

    noteCanBusFrame(nowMs, reply.canId.subnet, false, frame.length);

    logFocusedLiveMeter("can_reply", reply, reply.frame, nowMs);

    notePresenceReply(reply, nowMs);

    if (config_.enableVerboseFrameLog) {
      debugCli_.logCanFrame("CAN_RX", frame, &reply);
    }
    state_.noteCanRxForNode(reply.canId.subnet, reply.canId.device, nowMs);
    AmpState* refreshAmp = isUnknown0111CanReply(reply)
                              ? state_.amp(reply.canId.subnet, reply.canId.device, false) : 0;
    const bool exact0111RefreshReply = refreshAmp != 0 && refreshAmp->unknown0111RefreshPending;
    const int8_t liveIndex = liveRefreshIndex(reply);
    AmpState* liveAmp = liveIndex >= 0 ? state_.amp(reply.canId.subnet, reply.canId.device, false) : 0;
    AmpState::LiveRefresh* liveTrack = liveAmp != 0 ? &liveAmp->liveRefresh[liveIndex] : 0;
    ExtraLiveRefresh* extraLiveEntry = 0;
    if (liveTrack == 0 && isExtraLiveRefreshField(reply.fieldId)) {
      for (uint8_t i = 0; i < kExtraLiveRefreshCount; ++i) {
        ExtraLiveRefresh& entry = extraLiveRefresh_[i];
        if (entry.used && entry.subnet == reply.canId.subnet && entry.device == reply.canId.device &&
            entry.fieldId == reply.fieldId) {
          extraLiveEntry = &entry;
          liveTrack = &entry.track;
          break;
        }
      }
    }
    const bool liveRefreshReply = liveTrack != 0 && liveTrack->pending && liveTrack->sent;
    if (liveRefreshReply) {
      liveTrack->pending = false;
      liveTrack->sent = false;
      liveTrack->recovery.completed();
      ++liveTrack->completed;
    }
    if (exact0111RefreshReply) {
      refreshAmp->unknown0111RefreshPending = false;
      refreshAmp->unknown0111Recovery.completed();
      ++refreshAmp->unknown0111RefreshDoneCount;
    } else if (!liveRefreshReply) {
      markTraceRealCanRx(reply, nowMs);
    }
    noteBootstrapReply(reply, nowMs);
    confirmPowerOnFromNodeActivity(reply, nowMs);
    if (reply.valid) {
      if (isSpeakerIdField(reply.fieldId)) {
        ++counters_.speaker_id_can_rx;
      } else if (isInputModeField(reply.fieldId)) {
        ++counters_.input_mode_can_rx;
      } else if (isDataLinkPresetField(reply.fieldId)) {
        ++counters_.datalink_preset_can_rx;
      }
    }
    if (reply.valid &&
        reply.fieldId == FieldId::fixed_firmware_version &&
        reply.admin.valid &&
        reply.admin.functionCode == FunctionCode::ReadObject &&
        reply.admin.telegramType == TelegramType::Segmented) {
      ++counters_.firmware_segment_can_rx;
    }
    if (isUnknown0111CanReply(reply)) {
      const AmpState* ampState0111 = state_.amp(reply.canId.subnet, reply.canId.device);
      const bool hadUnknown0111Cache = ampState0111 != 0 && ampState0111->unknown0111CacheValid;
      state_.noteUnknown0111CanRx(reply.canId.subnet,
                                  reply.canId.device,
                                  reply.frame,
                                  nowMs);
      ++counters_.unknown_0111_can_rx;
      if (!hadUnknown0111Cache) {
        ++counters_.unknown_0111_cache_learned;
        if (config_.enableVerboseFrameLog &&
            config_.enableUnknown0111ExactCacheDebug &&
            shouldLogLowRateCounter(counters_.unknown_0111_cache_learned)) {
          char line[160] = {};
          snprintf(line, sizeof(line),
                   "T=%lums UNKNOWN_0111_LEARN node=%u.%02u data=%02X %02X %02X %02X",
                   static_cast<unsigned long>(nowMs),
                   static_cast<unsigned>(reply.canId.subnet),
                   static_cast<unsigned>(reply.canId.device),
                   static_cast<unsigned>(reply.frame.data[0]),
                   static_cast<unsigned>(reply.frame.data[1]),
                   static_cast<unsigned>(reply.frame.data[2]),
                   static_cast<unsigned>(reply.frame.length >= 4 ? reply.frame.data[3] : 0));
          debugCli_.logText(line);
        }
      }
    }
    if (isWriteFunction(reply.admin.functionCode) && isNameSegmentCacheField(reply.fieldId)) {
      state_.invalidateNameSegmentCache(reply.canId.subnet,
                                        reply.canId.device,
                                        reply.fieldId,
                                        "write",
                                        counters_,
                                        &debugCli_);
      debugCli_.logNameCacheInvalidate(reply.canId.subnet, reply.canId.device,
                                       reply.fieldId, "write");
    }
    if (reply.valid && isWriteFunction(reply.admin.functionCode) && isStableSetupField(reply.fieldId)) {
      state_.invalidateFieldCache(reply.canId.subnet, reply.canId.device, reply.fieldId);
      ++counters_.stable_setup_cache_invalidations;
      if (isSpeakerIdField(reply.fieldId)) {
        ++counters_.speaker_id_cache_invalidations;
      } else if (isInputModeField(reply.fieldId)) {
        ++counters_.input_mode_cache_invalidations;
      }
      debugCli_.logStableSetupCacheInvalidate(reply.canId.subnet,
                                              reply.canId.device,
                                              reply.fieldId,
                                              "write");
    }
    if (reply.valid &&
        isWriteFunction(reply.admin.functionCode) &&
        reply.admin.telegramType == TelegramType::Unstructured &&
        isConfigScalarTtlField(reply.fieldId)) {
      state_.invalidateFieldCache(reply.canId.subnet, reply.canId.device, reply.fieldId);
    }
    if (reply.valid &&
        isWriteFunction(reply.admin.functionCode) &&
        reply.canId.device != 0) {
      invalidateStatefulStatusCache(reply.canId.subnet,
                                    reply.canId.device,
                                    reply.objectAddress.channelNumber);
      confirmSemanticMirrorWriteAck(reply, nowMs);
      // Power/mute were already written through when the TCP request arrived.
      // Do not invalidate that responsive value again on the CAN write ACK.
      if (reply.fieldId != FieldId::settings_power &&
          reply.fieldId != FieldId::config_mute_a &&
          reply.fieldId != FieldId::config_mute_b) {
        invalidateSemanticMirrorForWrite(reply, "write");
      }
    }
    state_.updateCacheFromCan(reply, nowMs, false, counters_, &debugCli_);
    noteStatefulStatusCacheCanReply(reply, nowMs);
    scheduler_.observeCanRx(reply, nowMs, counters_);
    const uint8_t statefulCacheWaiters = releaseStatefulStatusCacheWaiters(reply, nowMs);
    const uint8_t collapsedWaiters =
        fanoutAttachFeatureEnabled() ? releaseSoftInflightCollapseWaiters(reply, nowMs) : 0u;
    const uint8_t shapedWaiters =
        gentleStatusShapingFeatureEnabled() ? releaseGentleStatusShapingWaiters(reply, nowMs) : 0u;
    const bool semanticMirrorRefreshReply = isSemanticMirrorBackgroundRefreshReply(reply);
    AmpState* object0100Amp = normalizedObjectKey(reply) == 0x0100u
                                 ? state_.amp(reply.canId.subnet, reply.canId.device, false)
                                 : 0;
    const bool triggeredObject0100HeartbeatReply =
        object0100Amp != 0 && object0100Amp->object0100HeartbeatQueuedMs != 0;
    if (triggeredObject0100HeartbeatReply) {
      object0100Amp->object0100HeartbeatQueuedMs = 0;
    }
    // A real waiting request (e.g. after cache expiry) may consume this reply;
    // otherwise the already-answered background read must not emit a second TCP reply.
    const RequestTrace* waiting0111 = (exact0111RefreshReply || liveRefreshReply) ? findMatchingPendingTrace(reply) : 0;
    const bool cacheOnly0111Reply = (exact0111RefreshReply || liveRefreshReply) &&
        (waiting0111 == 0 || waiting0111->pendingState == TracePendingState::TimeoutCleanup);
    bool pushedLiveStatus = false;
    if (extraLiveEntry != 0 && isRemoteHoldLiveField(reply.fieldId)) {
      const bool changed = extraLiveEntry->pushedValueValid &&
          (extraLiveEntry->lastPushedValue.length != reply.frame.length ||
           memcmp(extraLiveEntry->lastPushedValue.data, reply.frame.data, reply.frame.length) != 0);
      const bool recentlyVisible = extraLiveEntry->track.lastR1RequestMs != 0 &&
          nowMs - extraLiveEntry->track.lastR1RequestMs <= 2000u;
      const bool cooldownReady = extraLiveEntry->lastPushMs == 0 || nowMs - extraLiveEntry->lastPushMs >= 100u;
      const bool globalRateReady = lastLiveStatusPushMs_ == 0 || nowMs - lastLiveStatusPushMs_ >= 20u;
      extraLiveEntry->lastPushedValue = reply.frame;
      extraLiveEntry->pushedValueValid = true;
      if (changed && recentlyVisible && cooldownReady && globalRateReady && cacheOnly0111Reply &&
          forwardCanFrameToTcp(tcpServer_, state_, debugCli_, counters_, reply, nowMs)) {
        extraLiveEntry->lastPushMs = nowMs;
        lastLiveStatusPushMs_ = nowMs;
        ++liveStatusPushSent_;
        pushedLiveStatus = true;
        char line[192] = {};
        snprintf(line, sizeof(line),
                 "LIVE_STATUS_PUSH node=%u.%02u field=%s object=%04X value=%02X age_since_r1_ms=%lu sent=%lu rate_limited=%lu",
                 static_cast<unsigned>(reply.canId.subnet), static_cast<unsigned>(reply.canId.device),
                 fieldIdName(reply.fieldId), static_cast<unsigned>(normalizedObjectKey(reply)),
                 static_cast<unsigned>(reply.frame.length > 3 ? reply.frame.data[3] : 0u),
                 static_cast<unsigned long>(nowMs - extraLiveEntry->track.lastR1RequestMs),
                 static_cast<unsigned long>(liveStatusPushSent_),
                 static_cast<unsigned long>(liveStatusPushRateLimited_));
        debugCli_.logText(line);
      } else if (changed && recentlyVisible && cooldownReady && !globalRateReady && cacheOnly0111Reply) {
        ++liveStatusPushRateLimited_;
      }
    }
    bool pushedObject0100Heartbeat = false;
    if ((semanticMirrorRefreshReply || triggeredObject0100HeartbeatReply) &&
        normalizedObjectKey(reply) == 0x0100u) {
      recordAliveDiagnosticCanReply(reply, nowMs);
      ++r1Object0100HeartbeatCanRx_;
      if (forwardCanFrameToTcp(tcpServer_, state_, debugCli_, counters_, reply, nowMs)) {
        ++r1Object0100HeartbeatTcpTx_;
        pushedObject0100Heartbeat = true;
      } else {
        ++r1Object0100HeartbeatTcpFailed_;
      }
      if (shouldLogLowRateCounter(r1Object0100HeartbeatCanRx_)) {
        char line[224] = {};
        snprintf(line, sizeof(line),
                 "R1_0100_HEARTBEAT node=%u.%02u can_tx=%lu can_rx=%lu tcp_tx=%lu tcp_failed=%lu real_can_reply=1",
                 static_cast<unsigned>(reply.canId.subnet),
                 static_cast<unsigned>(reply.canId.device),
                 static_cast<unsigned long>(r1Object0100HeartbeatCanTx_),
                 static_cast<unsigned long>(r1Object0100HeartbeatCanRx_),
                 static_cast<unsigned long>(r1Object0100HeartbeatTcpTx_),
                 static_cast<unsigned long>(r1Object0100HeartbeatTcpFailed_));
        debugCli_.logText(line);
      }
    }
    const uint8_t forwardCopies =
        (semanticMirrorRefreshReply || triggeredObject0100HeartbeatReply || cacheOnly0111Reply)
            ? 0u
            : static_cast<uint8_t>(1u + (statefulCacheWaiters > 0 ? statefulCacheWaiters : 0u) +
                                   (collapsedWaiters > 0 ? collapsedWaiters : 0u) +
                                   (shapedWaiters > 0 ? shapedWaiters : 0u));
    bool forwardedReply = false;
    for (uint8_t copyIndex = 0; copyIndex < forwardCopies; ++copyIndex) {
      if (!forwardCanFrameToTcp(tcpServer_, state_, debugCli_, counters_, reply, nowMs)) {
        continue;
      }
      forwardedReply = true;
    }
    forwardedReply = forwardedReply || pushedLiveStatus || pushedObject0100Heartbeat;
    if (forwardedReply) {
      markTraceTcpTx(reply, TraceSource::RealCan, nowMs);
      noteSemanticFamilyUpdated(reply, nowMs, 2u, true);
      noteStartupBlockerReplyForwarded(reply, nowMs);
      if (isUnknown0111CanReply(reply)) {
        state_.noteUnknown0111TcpTx(reply.canId.subnet,
                                    reply.canId.device,
                                    unknown0111Value(reply.frame),
                                    nowMs);
        ++counters_.unknown_0111_tcp_tx;
        if (counters_.unknown_0111_pending != 0) {
          --counters_.unknown_0111_pending;
        }
      }
    }
    if (semanticMirrorRefreshReply) {
      noteSemanticFamilyUpdated(reply, nowMs, 4u, false);
    }
    if (!isIdentPresenceReply(reply) &&
        (reply.mapping == 0 || reply.fieldId == FieldId::Unknown ||
         isSemanticCandidateStatusObjectKey(normalizedObjectKey(reply)))) {
      noteSemanticCandidate(reply.canId.subnet,
                            reply.canId.device,
                            reply.objectAddress.channelNumber,
                            normalizedObjectKey(reply),
                            reply.frame.data,
                            reply.frame.length,
                            nowMs,
                            forwardedReply,
                            reply.fieldId == FieldId::Unknown ? "field_unknown" : "status_container");
    }
    noteSemanticMirrorCanReply(reply, nowMs, forwardedReply);

    uint8_t tcpNode = 0;
    state_.completePending(reply, tcpNode);
  }

  updateRegressionCounters(nowMs);
  processTraceTimeoutCleanup(nowMs);
  processTimeoutWindowDump(nowMs);
  processPresenceRetries(nowMs);
  processBootstrapRecovery(nowMs);
  logBootstrapIncompleteNodes(nowMs);
  pollPlanner_.tick(state_, nowMs, counters_);
  processSemanticMirrorFreshness(nowMs);
  processLocalControlRefresh(nowMs);
  processIspRefresh(nowMs);
  if (!processDeferredSegmentedRequests(nowMs)) {
    processDeferredIdentRequests(nowMs);
  }
  scheduler_.tick(state_, nowMs, counters_);
  if (static_cast<int32_t>(nowMs - (lastStateSummaryMs_ + 1000)) >= 0) {
    lastStateSummaryMs_ = nowMs;
    counters_.psram_total = psramTotalBytes();
    counters_.psram_free = psramFreeBytes();
    state_.updateDebugStats(counters_);
    updateBootstrapCounters(nowMs);
    if (lastRuntimeBaselineSummaryMs_ == 0 ||
        static_cast<int32_t>(nowMs - (lastRuntimeBaselineSummaryMs_ + kRuntimeBaselineSummaryMs)) >= 0) {
      logRuntimeBaselineSummary(nowMs);
    }
    updateRegressionCounters(nowMs);
    evaluateRegressionGuards(nowMs);
    const uint32_t canTxFramesDelta = counters_.can_tx_frames - lastSummaryCanTxFrames_;
    const uint32_t canRxFramesDelta = counters_.can_rx_frames - lastSummaryCanRxFrames_;
    const uint32_t tcpRxFramesDelta = counters_.tcp_rx_frames - lastSummaryTcpRxFrames_;
    const uint32_t tcpTxFramesDelta = counters_.tcp_tx_frames - lastSummaryTcpTxFrames_;
    counters_.can_tx_per_sec = canTxFramesDelta;
    counters_.can_rx_per_sec = canRxFramesDelta;
    counters_.tcp_rx_per_sec = tcpRxFramesDelta;
    counters_.tcp_tx_per_sec = tcpTxFramesDelta;
    logSemanticHealth(nowMs);
    logSemanticCandidateSummary(nowMs);
    const uint32_t tcpBackpressureActiveMs =
        counters_.tcp_backpressure_active_ms +
        (tcpBackpressureActive_ ? nowMs - tcpBackpressureStartMs_ : 0);
    const uint32_t normalOldestAgeMs = counters_.normal_read_queue_oldest_age_ms;
    if (longRunNormalOldestSamples_ == 0) {
      longRunNormalOldestMin_ = normalOldestAgeMs;
      longRunNormalOldestMax_ = normalOldestAgeMs;
    } else {
      if (normalOldestAgeMs < longRunNormalOldestMin_) {
        longRunNormalOldestMin_ = normalOldestAgeMs;
      }
      if (normalOldestAgeMs > longRunNormalOldestMax_) {
        longRunNormalOldestMax_ = normalOldestAgeMs;
      }
    }
    longRunNormalOldestSum_ += normalOldestAgeMs;
    ++longRunNormalOldestSamples_;
    if (counters_.normal_read_queue_depth > longRunNormalDepthMax_) {
      longRunNormalDepthMax_ = counters_.normal_read_queue_depth;
    }
    if (counters_.can_tx_queue_depth > longRunCanTxDepthMax_) {
      longRunCanTxDepthMax_ = counters_.can_tx_queue_depth;
    }
    if (config_.enableLowRateSummary && counters_.normal_read_queue_oldest_age_ms > 1500) {
      debugCli_.logNormalQueueLatency(counters_.normal_read_queue_oldest_age_ms,
                                      static_cast<uint16_t>(counters_.normal_read_queue_depth),
                                      true);
    } else if (config_.enableLowRateSummary && counters_.normal_read_queue_oldest_age_ms > 500) {
      debugCli_.logNormalQueueLatency(counters_.normal_read_queue_oldest_age_ms,
                                      static_cast<uint16_t>(counters_.normal_read_queue_depth),
                                      false);
    }
    for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
      const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
      if (ampState == 0 || !ampState->used || ampState->device == 0) {
        continue;
      }
      const uint32_t replyGapMs = r1ReplyAgeMs(*ampState, nowMs);
      if (ampState->subnet < DebugCounters::kSubnetCount &&
          replyGapMs > slowCanWindow_.subnetReplyGapMaxMs[ampState->subnet]) {
        slowCanWindow_.subnetReplyGapMaxMs[ampState->subnet] = replyGapMs;
      }
      if (replyGapMs > slowCanWindow_.worstDeviceReplyGapMs) {
        slowCanWindow_.worstDeviceReplyGapMs = replyGapMs;
        slowCanWindow_.worstDeviceReplyGapSubnet = ampState->subnet;
        slowCanWindow_.worstDeviceReplyGapDevice = ampState->device;
      }
      uint8_t gapLevel = 0;
      if (replyGapMs >= 1000u) {
        gapLevel = 3u;
      } else if (replyGapMs >= 750u) {
        gapLevel = 2u;
      } else if (replyGapMs >= 500u) {
        gapLevel = 1u;
      }
      AmpState* mutableAmp = state_.amp(ampState->subnet, ampState->device, false);
      if (mutableAmp == 0) {
        continue;
      }
      if (gapLevel > mutableAmp->lastServiceGapLevel) {
        for (uint8_t level = static_cast<uint8_t>(mutableAmp->lastServiceGapLevel + 1u);
             level <= gapLevel;
             ++level) {
          ++slowCanWindow_.missedServiceWindowCount;
          char line[192] = {};
          const uint16_t thresholdMs = level == 1u ? 500u : (level == 2u ? 750u : 1000u);
          snprintf(line, sizeof(line),
                   "T=%lums DEVICE_SERVICE_GAP node=%u.%02u threshold_ms=%u reply_gap_ms=%lu last_can_rx_age_ms=%lu last_cache_reply_age_ms=%lu",
                   static_cast<unsigned long>(nowMs),
                   static_cast<unsigned>(mutableAmp->subnet),
                   static_cast<unsigned>(mutableAmp->device),
                   static_cast<unsigned>(thresholdMs),
                   static_cast<unsigned long>(replyGapMs),
                   mutableAmp->lastCanRxMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - mutableAmp->lastCanRxMs),
                   mutableAmp->lastCacheReplyMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - mutableAmp->lastCacheReplyMs));
          debugCli_.logText(line);
        }
      }
      mutableAmp->lastServiceGapLevel = gapLevel;
    }
    if (config_.enableTimeoutCorrelation) {
      rebuildWatchdogState(nowMs);
      counters_.unknown_0111_pending = 0;
      for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
        const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
        if (ampState != 0) {
          counters_.unknown_0111_pending += ampState->unknown0111Pending;
        }
      }
    } else {
      counters_.unknown_0111_pending = 0;
    }
    counters_.loop_max_us = loopMaxUsWindow_;
    counters_.loop_avg_us =
        loopSamplesWindow_ != 0 ? static_cast<uint32_t>(loopSumUsWindow_ / loopSamplesWindow_) : 0;
    counters_.can_poll_max_gap_us = canPollMaxGapUsWindow_;
    counters_.tcp_poll_max_gap_us = tcpPollMaxGapUsWindow_;
    counters_.max_serial_print_burst_ms = maxSerialPrintBurstMsWindow_;
    counters_.serial_bytes_printed_delta = serialBytesPrintedEstimateWindow_;
    counters_.timeout_correlation_enabled = config_.enableTimeoutCorrelation ? 1u : 0u;
    counters_.verbose_frame_log_enabled = config_.enableVerboseFrameLog ? 1u : 0u;
    if (config_.enableLowRateSummary) {
      const uint32_t serialBurstStartMs = millis();
      debugCli_.logStateSummary(counters_,
                                nowMs,
                                canTxFramesDelta,
                                canRxFramesDelta,
                                tcpRxFramesDelta,
                                tcpTxFramesDelta,
                                tcpBackpressureActiveMs,
                                scheduler_.currentSpacingUs(nowMs));
      serialBytesPrintedEstimateWindow_ += 2048;
      const uint32_t burstMs = millis() - serialBurstStartMs;
      if (burstMs > maxSerialPrintBurstMsWindow_) {
        maxSerialPrintBurstMsWindow_ = burstMs;
      }
    }
    if (static_cast<int32_t>(nowMs - (lastLongRunSummaryMs_ + 60000)) >= 0) {
      const uint32_t durationMs = nowMs - lastLongRunSummaryMs_;
      counters_.normal_read_queue_oldest_age_ms_min = longRunNormalOldestSamples_ != 0 ? longRunNormalOldestMin_ : 0;
      counters_.normal_read_queue_oldest_age_ms_max = longRunNormalOldestSamples_ != 0 ? longRunNormalOldestMax_ : 0;
      counters_.normal_read_queue_oldest_age_ms_avg =
          longRunNormalOldestSamples_ != 0
              ? static_cast<uint32_t>(longRunNormalOldestSum_ / longRunNormalOldestSamples_)
              : 0;
      counters_.normal_read_queue_depth_max = longRunNormalDepthMax_;
      counters_.can_tx_queue_depth_max = longRunCanTxDepthMax_;
      counters_.queue_full_total_delta = counters_.can_tx_queue_full_total - lastLongRunQueueFullTotal_;
      counters_.tcp_backpressure_active_ms_delta = tcpBackpressureActiveMs - lastLongRunTcpBackpressureActiveMs_;
      counters_.cache_replies_sent_delta = counters_.cache_replies_sent - lastLongRunCacheRepliesSent_;
      counters_.can_tx_frames_delta_long_run = counters_.can_tx_frames - lastLongRunCanTxFrames_;
      counters_.can_rx_frames_delta_long_run = counters_.can_rx_frames - lastLongRunCanRxFrames_;
      if (config_.enableLowRateSummary) {
        const uint32_t serialBurstStartMs = millis();
        debugCli_.logLongRunPerfSummary(counters_, nowMs, durationMs / 1000u);
        serialBytesPrintedEstimateWindow_ += 256;
        const uint32_t burstMs = millis() - serialBurstStartMs;
        if (burstMs > maxSerialPrintBurstMsWindow_) {
          maxSerialPrintBurstMsWindow_ = burstMs;
        }
      }
      lastLongRunSummaryMs_ = nowMs;
      lastLongRunQueueFullTotal_ = counters_.can_tx_queue_full_total;
      lastLongRunTcpBackpressureActiveMs_ = tcpBackpressureActiveMs;
      lastLongRunCacheRepliesSent_ = counters_.cache_replies_sent;
      lastLongRunCanTxFrames_ = counters_.can_tx_frames;
      lastLongRunCanRxFrames_ = counters_.can_rx_frames;
      longRunNormalOldestMin_ = 0;
      longRunNormalOldestMax_ = 0;
      longRunNormalOldestSum_ = 0;
      longRunNormalOldestSamples_ = 0;
      longRunNormalDepthMax_ = 0;
      longRunCanTxDepthMax_ = 0;
    }
    if (config_.enableLowRateSummary && R70_EXPERIMENTAL_DEBUG_CODE) {
      logTopObjectStats();
      serialBytesPrintedEstimateWindow_ += 16u * 128u;
    }
    if (config_.enableTimeoutCorrelation) {
      uint8_t groupRiskSubnets[8] = {};
      uint8_t groupRiskDevices[8] = {};
      uint8_t groupRiskCount = 0;
      struct NodeAgeTopEntry {
        const AmpState* amp;
        uint32_t lastTcpTxAgeMs;
        uint32_t lastRealCanRxAgeMs;
        uint32_t lastCacheReplyAgeMs;
        uint32_t lastCanTxPollAgeMs;
      };
      NodeAgeTopEntry top[8] = {};
      for (uint8_t ampIndex = 0; ampIndex < state_.activeAmpCount(); ++ampIndex) {
        const AmpState* ampState = state_.activeAmpByIndex(ampIndex);
        if (ampState == 0 || !ampState->used || ampState->device == 0) {
          continue;
        }
        if (ampState->lastTcpTxToR1Ms == 0) {
          continue;
        }
        const uint32_t lastTcpTxAgeMs = nowMs - ampState->lastTcpTxToR1Ms;
        const uint32_t lastRealCanRxAgeMs =
            ampState->lastCanRxMs == 0 ? 0 : (nowMs - ampState->lastCanRxMs);
        const uint32_t lastCacheReplyAgeMs =
            ampState->lastCacheReplyMs == 0 ? 0 : (nowMs - ampState->lastCacheReplyMs);
        const uint32_t lastCanTxPollAgeMs =
            ampState->lastCanTxPollMs == 0 ? 0 : (nowMs - ampState->lastCanTxPollMs);
        if (lastTcpTxAgeMs > 2000) {
          state_.noteTimeoutRisk(ampState->subnet, ampState->device);
          debugCli_.logNodeTimeoutRisk(ampState->subnet,
                                       ampState->device,
                                       nowMs,
                                       lastTcpTxAgeMs,
                                       lastRealCanRxAgeMs,
                                       lastCacheReplyAgeMs,
                                       lastCanTxPollAgeMs);
        }
        if (ampState->lastTcpRxRequestMs != 0 &&
            ampState->lastTcpRxRequestMs > ampState->lastTcpTxToR1Ms) {
          const uint32_t requestAgeMs = nowMs - ampState->lastTcpRxRequestMs;
          if (requestAgeMs > 1500) {
            debugCli_.logNodeRequestWithoutReply(ampState->subnet,
                                                 ampState->device,
                                                 nowMs,
                                                 ampState->lastTcpRxRequestField,
                                                 requestAgeMs);
          }
        }
        const bool suspicious =
            ampState->oldestPendingFieldAgeMs > kWatchdogWarnAgeMs ||
            ampState->maxRequestToTcpTxLatencyMs > kWatchdogWarnAgeMs ||
            lastTcpTxAgeMs > kWatchdogWarnAgeMs ||
            (ampState->lastUnknown0111RequestMs != 0 &&
             ampState->lastUnknown0111TcpTxMs < ampState->lastUnknown0111RequestMs &&
             (nowMs - ampState->lastUnknown0111RequestMs) > kWatchdogWarnAgeMs);
        if (suspicious) {
          char line[256] = {};
          snprintf(line, sizeof(line),
                   "T=%lums NODE_WATCHDOG_RISK node=%u.%02u last_tcp_tx_age_ms=%lu last_real_can_rx_age_ms=%lu last_cache_reply_age_ms=%lu oldest_pending_age_ms=%lu pending_count=%u max_req_to_tx_latency_ms=%lu last_0111_req_age_ms=%lu last_0111_rx_age_ms=%lu last_0111_tx_age_ms=%lu",
                   static_cast<unsigned long>(nowMs),
                   static_cast<unsigned>(ampState->subnet),
                   static_cast<unsigned>(ampState->device),
                   static_cast<unsigned long>(lastTcpTxAgeMs),
                   static_cast<unsigned long>(lastRealCanRxAgeMs),
                   static_cast<unsigned long>(lastCacheReplyAgeMs),
                   static_cast<unsigned long>(ampState->oldestPendingFieldAgeMs),
                   static_cast<unsigned>(ampState->pendingFieldCount),
                   static_cast<unsigned long>(ampState->maxRequestToTcpTxLatencyMs),
                   ampState->lastUnknown0111RequestMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - ampState->lastUnknown0111RequestMs),
                   ampState->lastUnknown0111RxMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - ampState->lastUnknown0111RxMs),
                   ampState->lastUnknown0111TcpTxMs == 0 ? 0ul : static_cast<unsigned long>(nowMs - ampState->lastUnknown0111TcpTxMs));
          debugCli_.logText(line);
        }
        const bool autoSnapshot =
            ampState->oldestPendingFieldAgeMs > kWatchdogCriticalAgeMs ||
            ampState->unknown0111Pending != 0 &&
                ampState->lastUnknown0111RequestMs != 0 &&
                (nowMs - ampState->lastUnknown0111RequestMs) > kWatchdogCriticalAgeMs ||
            lastTcpTxAgeMs > kWatchdogCriticalAgeMs;
        const bool groupRisk =
            ampState->oldestPendingFieldAgeMs > kWatchdogWarnAgeMs ||
            lastTcpTxAgeMs > kWatchdogWarnAgeMs ||
            ampState->maxRequestToTcpTxLatencyMs > kWatchdogWarnAgeMs ||
            (ampState->unknown0111Pending != 0 &&
             ampState->lastUnknown0111RequestMs != 0 &&
             (nowMs - ampState->lastUnknown0111RequestMs) > kWatchdogWarnAgeMs);
        if (groupRisk && groupRiskCount < 8) {
          groupRiskSubnets[groupRiskCount] = ampState->subnet;
          groupRiskDevices[groupRiskCount] = ampState->device;
          ++groupRiskCount;
        }
        if (config_.enableAutoGroupTimeoutSnapshot &&
            autoSnapshot && (ampState->lastAutoTimeoutSnapshotMs == 0 ||
                             static_cast<int32_t>(nowMs - (ampState->lastAutoTimeoutSnapshotMs +
                                                           kAutoSnapshotRateLimitMs)) >= 0)) {
          AmpState* mutableAmp = state_.amp(ampState->subnet, ampState->device, false);
          if (mutableAmp != 0) {
            mutableAmp->lastAutoTimeoutSnapshotMs = nowMs;
          }
          logTimeoutCorrelation(ampState->subnet, ampState->device, nowMs,
                                "AUTO_TIMEOUT_RISK_SNAPSHOT");
        }
        for (uint8_t slot = 0; slot < 8; ++slot) {
          if (top[slot].amp == 0 || lastTcpTxAgeMs > top[slot].lastTcpTxAgeMs) {
            for (int8_t shift = 7; shift > static_cast<int8_t>(slot); --shift) {
              top[shift] = top[shift - 1];
            }
            top[slot].amp = ampState;
            top[slot].lastTcpTxAgeMs = lastTcpTxAgeMs;
            top[slot].lastRealCanRxAgeMs = lastRealCanRxAgeMs;
            top[slot].lastCacheReplyAgeMs = lastCacheReplyAgeMs;
            top[slot].lastCanTxPollAgeMs = lastCanTxPollAgeMs;
            break;
          }
        }
      }
      if (config_.enableAutoGroupTimeoutSnapshot &&
          groupRiskCount >= 2 &&
          (lastAutoGroupTimeoutSnapshotMs_ == 0 ||
           static_cast<int32_t>(nowMs - (lastAutoGroupTimeoutSnapshotMs_ + kAutoSnapshotRateLimitMs)) >= 0)) {
        lastAutoGroupTimeoutSnapshotMs_ = nowMs;
        logGroupTimeoutCorrelation(groupRiskSubnets,
                                   groupRiskDevices,
                                   groupRiskCount,
                                   nowMs,
                                   "AUTO_GROUP_TIMEOUT_RISK");
      }
      for (uint8_t slot = 0; slot < 8; ++slot) {
        if (top[slot].amp == 0) {
          continue;
        }
        debugCli_.logNodeReplyAgeTop(top[slot].amp->subnet,
                                     top[slot].amp->device,
                                     static_cast<uint8_t>(slot + 1),
                                     nowMs,
                                     top[slot].lastTcpTxAgeMs,
                                     top[slot].lastRealCanRxAgeMs,
                                     top[slot].lastCacheReplyAgeMs,
                                     top[slot].lastCanTxPollAgeMs,
                                     top[slot].amp->tcpTxCount,
                                     top[slot].amp->realCanRxCount,
                                     top[slot].amp->cacheReplyCount,
                                     top[slot].amp->canTxPollCount);
        serialBytesPrintedEstimateWindow_ += 160;
      }
      logUnknown0111Summary(nowMs);
      serialBytesPrintedEstimateWindow_ += 8u * 120u;
    }
    if (config_.enableLowRateSummary) {
      for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
        const uint32_t identRxDelta =
            counters_.ident_rx_count_by_subnet[subnet] - lastSummaryIdentRxCountBySubnet_[subnet];
        debugCli_.logIdentSummary(
            subnet,
            identRxDelta,
            subnet == 1 ? counters_.can_rx_id_0x441 - lastSummaryRx441_ : 0,
            subnet == 1 ? counters_.can_rx_id_0x442 - lastSummaryRx442_ : 0);
        serialBytesPrintedEstimateWindow_ += 80;
        lastSummaryIdentRxCountBySubnet_[subnet] = counters_.ident_rx_count_by_subnet[subnet];
      }
    } else {
      for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
        lastSummaryIdentRxCountBySubnet_[subnet] = counters_.ident_rx_count_by_subnet[subnet];
      }
    }
    lastSummaryCanTxFrames_ = counters_.can_tx_frames;
    lastSummaryCanRxFrames_ = counters_.can_rx_frames;
    lastSummaryTcpRxFrames_ = counters_.tcp_rx_frames;
    lastSummaryTcpTxFrames_ = counters_.tcp_tx_frames;
    lastSummaryMuteWriteTxCount_ = counters_.mute_write_tx_count;
    lastSummaryMuteWriteAckCount_ = counters_.mute_write_ack_count;
    lastSummaryMuteWriteTimeoutCount_ = counters_.mute_write_timeout_count;
    lastSummaryRx441_ = counters_.can_rx_id_0x441;
    lastSummaryRx442_ = counters_.can_rx_id_0x442;
    loopMaxUsWindow_ = 0;
    loopSumUsWindow_ = 0;
    loopSamplesWindow_ = 0;
    canPollMaxGapUsWindow_ = 0;
    tcpPollMaxGapUsWindow_ = 0;
    maxSerialPrintBurstMsWindow_ = 0;
    serialBytesPrintedEstimateWindow_ = 0;
  }

  const uint32_t loopDurationUs = micros() - pollStartUs;
  if (loopDurationUs > loopMaxUsWindow_) {
    loopMaxUsWindow_ = loopDurationUs;
  }
}

void R70GatewayV2::restartNetworkServices() {
  if (!started_) {
    return;
  }
  discovery_.end();
  const bool discoveryStarted = discovery_.begin(config_.udpPort);
  tcpServer_.begin(config_.tcpPort);
  lastTcpConnected_ = false;
  if (log_ != 0) {
    log_->print(F("NETWORK_SERVICES_RESTART discovery="));
    log_->print(discoveryStarted ? 1 : 0);
    log_->print(F(" tcp_port="));
    log_->println(config_.tcpPort);
  }
}

const DebugCounters& R70GatewayV2::counters() const {
  return counters_;
}

DebugCounters& R70GatewayV2::mutableCounters() {
  return counters_;
}

bool R70GatewayV2::tcpControlConnected() const {
  return tcpServer_.connected();
}

bool R70GatewayV2::enqueueLocalPrimaryMasterFrame(const Frame& frame, const char*& failureReason) {
  failureReason = 0;
  if (!started_) {
    failureReason = "gateway_not_started";
    return false;
  }

  NormalizedRequest request = {};
  if (!codec::normalizeCanFrame(frame, RequestOrigin::Can, request)) {
    failureReason = "normalize_failed";
    return false;
  }
  return scheduler_.enqueue(request, 0, false, millis(), counters_, failureReason);
}

bool R70GatewayV2::sendDirectPrimaryMasterFrame(const Frame& frame,
                                                uint32_t timeoutMs,
                                                const char*& failureReason) {
  failureReason = 0;
  if (!started_) {
    failureReason = "gateway_not_started";
    return false;
  }
  if (frame.canId > 0x7FF || frame.length == 0 || frame.length > kMaxFrameDataLength) {
    failureReason = "invalid_frame";
    return false;
  }
  if (!canTransport_.send(frame, timeoutMs)) {
    failureReason = "transport_send_false";
    return false;
  }
  ++counters_.can_tx_frames;
  return true;
}

void R70GatewayV2::printCounters(Print& out) const {
  if (counters_.can_response_over_1000ms > lastPrintedCanResponseOver1000Ms_) {
    if (canResponseOver1000GrowthStreak_ < 0xFFu) {
      ++canResponseOver1000GrowthStreak_;
    }
  } else {
    canResponseOver1000GrowthStreak_ = 0;
  }
  lastPrintedCanResponseOver1000Ms_ = counters_.can_response_over_1000ms;

  DebugCli printer;
  printer.begin(out);
  printer.printCounters(counters_);
  const uint32_t nowMs = millis();
  printStatefulStatusCacheActivationStatus(out, nowMs);
  printStatefulStatusCacheGroupSummaries(out);
  out.print("STATEFUL_CACHE_REASON text=");
  out.println(statefulStatusCacheDisabledReasonName(statefulStatusCacheDisabledReasonCode_));
  printR1FreshnessSummary(out, nowMs);
  printSlowCanSummary(out);
  const_cast<R70GatewayV2*>(this)->resetStatefulStatusCacheSummaryWindow();

  const bool warnNormalQueue = counters_.normal_read_queue_highwater > 128;
  const bool warnCanTxQueue = counters_.can_tx_queue_highwater > 128;
  const bool warnLatency = counters_.max_can_response_latency_ms > 1500;
  const bool warnOver1000Growth = canResponseOver1000GrowthStreak_ >= 3;
  if (!warnNormalQueue && !warnCanTxQueue && !warnLatency && !warnOver1000Growth) {
    return;
  }

  out.print("SCALING_WARN");
  if (warnNormalQueue) {
    out.print(" normal_q_hwm=");
    out.print(counters_.normal_read_queue_highwater);
  }
  if (warnCanTxQueue) {
    out.print(" can_tx_q_hwm=");
    out.print(counters_.can_tx_queue_highwater);
  }
  if (warnLatency) {
    out.print(" max_can_response_latency_ms=");
    out.print(counters_.max_can_response_latency_ms);
  }
  if (warnOver1000Growth) {
    out.print(" can_response_over_1000ms=");
    out.print(counters_.can_response_over_1000ms);
    out.print(" growth_intervals=");
    out.print(canResponseOver1000GrowthStreak_);
  }
  out.println();
}

void R70GatewayV2::printBootstrapDiagnostics(Print& out) const {
  DebugCli printer;
  printer.begin(out);
  printer.printBootstrapDiagnostics(counters_);
}

GatewayState& R70GatewayV2::state() {
  return state_;
}

const GatewayConfig& R70GatewayV2::config() const {
  return config_;
}

}  // namespace r70v2
}  // namespace dbcan
