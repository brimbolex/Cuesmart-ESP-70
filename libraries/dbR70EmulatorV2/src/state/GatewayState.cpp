#include "GatewayState.h"

#include <stdlib.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_heap_caps.h>
#endif

#include "../protocol/CanObjectCodec.h"

namespace dbcan {
namespace r70v2 {

namespace {

uint8_t dataOffset(const NormalizedRequest& request) {
  switch (request.admin.telegramType) {
    case TelegramType::Unstructured:
      return 3;
    case TelegramType::Segmented:
      return 4;
    case TelegramType::File:
      return 4;
    case TelegramType::FileSegmented:
      return 6;
    default:
      return request.frame.length;
  }
}

bool cacheAllowedFromBehavior(BehaviorConfidence confidence) {
  return confidence == BehaviorConfidence::SeenOnCan ||
         confidence == BehaviorConfidence::ConfirmedCapture ||
         confidence == BehaviorConfidence::ConfirmedRoundTrip;
}

bool isNameSegmentField(FieldId fieldId) {
  return fieldId == FieldId::settings_device_name ||
         fieldId == FieldId::config_name_a ||
         fieldId == FieldId::config_name_b ||
         fieldId == FieldId::error_text ||
         fieldId == FieldId::cherr_error_text_a ||
         fieldId == FieldId::cherr_error_text_b;
}

bool isFirmwareSegmentField(FieldId fieldId) {
  return fieldId == FieldId::fixed_firmware_version;
}

void* allocBytes(size_t size, bool psram) {
#if defined(ARDUINO_ARCH_ESP32)
  const uint32_t caps = psram ? (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
                              : (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  return heap_caps_malloc(size, caps);
#else
  (void)psram;
  return malloc(size);
#endif
}

void freeBytes(void* ptr) {
  if (ptr == 0) {
    return;
  }
#if defined(ARDUINO_ARCH_ESP32)
  heap_caps_free(ptr);
#else
  free(ptr);
#endif
}

}  // namespace

GatewayState::GatewayState()
    : fieldCachePool_(0),
      segmentSlotPool_(0),
      nameSegmentCachePool_(0),
      fieldCacheCapacityPerAmp_(0),
      maxActiveAmps_(0),
      activeAmpCount_(0),
      usingPsram_(false),
      fallbackMode_(false),
      bytesAllocated_(0),
      recentHead_(0),
      nameSegmentMaxSegments_(kNameSegmentMaxSegments),
      nameSegmentPayloadMaxLen_(kNameSegmentPayloadMaxLen) {
  for (uint8_t index = 0; index < kPreferredMaxActiveAmps; ++index) {
    amps_[index].caches = 0;
    amps_[index].cacheCapacity = 0;
    amps_[index].used = false;
  }
  reset();
}

GatewayState::~GatewayState() {
  releaseStorage();
}

bool GatewayState::begin(bool preferPsram,
                         uint8_t nameSegmentMaxSegments,
                         uint8_t nameSegmentPayloadMaxLen) {
  releaseStorage();
  nameSegmentMaxSegments_ =
      nameSegmentMaxSegments > kNameSegmentMaxSegments ? kNameSegmentMaxSegments : nameSegmentMaxSegments;
  if (nameSegmentMaxSegments_ == 0) {
    nameSegmentMaxSegments_ = 1;
  }
  nameSegmentPayloadMaxLen_ =
      nameSegmentPayloadMaxLen > kNameSegmentPayloadMaxLen ? kNameSegmentPayloadMaxLen
                                                           : nameSegmentPayloadMaxLen;
  if (nameSegmentPayloadMaxLen_ == 0) {
    nameSegmentPayloadMaxLen_ = kNameSegmentPayloadMaxLen;
  }

  struct Attempt {
    uint8_t ampCount;
    uint16_t fieldCapacityPerAmp;
    bool psram;
    bool fallback;
  };

  const Attempt attempts[] = {
      {kPreferredMaxActiveAmps, kPreferredScalarFieldsPerAmp, preferPsram, false},
      {kFallbackMaxActiveAmps, kFallbackScalarFieldsPerAmp, false, true},
  };

  for (uint8_t attemptIndex = 0; attemptIndex < (sizeof(attempts) / sizeof(attempts[0])); ++attemptIndex) {
    const Attempt& attempt = attempts[attemptIndex];
    FieldCache* fieldPool = static_cast<FieldCache*>(
        allocBytes(static_cast<size_t>(attempt.ampCount) * attempt.fieldCapacityPerAmp * sizeof(FieldCache),
                   attempt.psram));
    SegmentedAssembler::Slot* segmentSlots = static_cast<SegmentedAssembler::Slot*>(
        allocBytes(static_cast<size_t>(attempt.ampCount) * kSegmentCachePerAmp *
                       sizeof(SegmentedAssembler::Slot),
                   attempt.psram));
    NameSegmentCacheEntry* nameSegmentPool = static_cast<NameSegmentCacheEntry*>(
        allocBytes(static_cast<size_t>(attempt.ampCount) * kSegmentedReplyCacheFieldCount *
                       nameSegmentMaxSegments_ * sizeof(NameSegmentCacheEntry),
                   attempt.psram));
    if (fieldPool == 0 || segmentSlots == 0 || nameSegmentPool == 0) {
      freeBytes(fieldPool);
      freeBytes(segmentSlots);
      freeBytes(nameSegmentPool);
      continue;
    }

    fieldCachePool_ = fieldPool;
    segmentSlotPool_ = segmentSlots;
    nameSegmentCachePool_ = nameSegmentPool;
    fieldCacheCapacityPerAmp_ = attempt.fieldCapacityPerAmp;
    maxActiveAmps_ = attempt.ampCount;
    activeAmpCount_ = 0;
    usingPsram_ = attempt.psram;
    fallbackMode_ = attempt.fallback;
    bytesAllocated_ =
        static_cast<uint32_t>(static_cast<size_t>(attempt.ampCount) * attempt.fieldCapacityPerAmp *
                                  sizeof(FieldCache) +
                              static_cast<size_t>(attempt.ampCount) * kSegmentCachePerAmp *
                                  sizeof(SegmentedAssembler::Slot) +
                              static_cast<size_t>(attempt.ampCount) * kSegmentedReplyCacheFieldCount *
                                  nameSegmentMaxSegments_ * sizeof(NameSegmentCacheEntry));
    segmented_.begin(segmentSlots,
                     static_cast<uint16_t>(attempt.ampCount * kSegmentCachePerAmp));
    for (uint8_t index = 0; index < kPreferredMaxActiveAmps; ++index) {
      if (index < maxActiveAmps_) {
        amps_[index].bind(fieldCachePool_ + static_cast<size_t>(index) * fieldCacheCapacityPerAmp_,
                          fieldCacheCapacityPerAmp_);
      } else {
        amps_[index].caches = 0;
        amps_[index].cacheCapacity = 0;
        amps_[index].used = false;
      }
    }
    reset();
    return true;
  }

  return false;
}

void GatewayState::releaseStorage() {
  freeBytes(fieldCachePool_);
  freeBytes(segmentSlotPool_);
  freeBytes(nameSegmentCachePool_);
  fieldCachePool_ = 0;
  segmentSlotPool_ = 0;
  nameSegmentCachePool_ = 0;
  fieldCacheCapacityPerAmp_ = 0;
  maxActiveAmps_ = 0;
  activeAmpCount_ = 0;
  usingPsram_ = false;
  fallbackMode_ = false;
  bytesAllocated_ = 0;
  nameSegmentMaxSegments_ = kNameSegmentMaxSegments;
  nameSegmentPayloadMaxLen_ = kNameSegmentPayloadMaxLen;
  segmented_.begin(0, 0);
  for (uint8_t index = 0; index < kPreferredMaxActiveAmps; ++index) {
    amps_[index].caches = 0;
    amps_[index].cacheCapacity = 0;
    amps_[index].used = false;
  }
}

uint16_t GatewayState::addressIndex(uint8_t subnet, uint8_t device) {
  return static_cast<uint16_t>((static_cast<uint16_t>(subnet) << 6) | (device & 0x3F));
}

void GatewayState::reset() {
  for (uint16_t index = 0; index < kMaxAddressSlots; ++index) {
    addressToAmpIndex_[index] = 0xFF;
  }
  for (uint8_t index = 0; index < kPreferredMaxActiveAmps; ++index) {
    amps_[index].clear();
  }
  activeAmpCount_ = 0;
  for (uint8_t index = 0; index < kMaxPendingRequests; ++index) {
    pending_[index].clear();
  }
  for (uint8_t index = 0; index < kMaxRecentRequests; ++index) {
    recent_[index] = {};
  }
  recentHead_ = 0;
  segmented_.clear();
  clearNameSegmentEntries();
}

void GatewayState::tick(uint32_t nowMs, DebugCounters& counters) {
  for (uint8_t index = 0; index < kMaxPendingRequests; ++index) {
    if (pending_[index].expired(nowMs)) {
      pending_[index].clear();
    }
  }
  counters.segmented_timeout += segmented_.expire(nowMs);
}

void GatewayState::updateDebugStats(DebugCounters& counters) const {
  counters.active_amp_count = activeAmpCount_;
  counters.field_cache_capacity = static_cast<uint32_t>(maxActiveAmps_) * fieldCacheCapacityPerAmp_;
  counters.segment_cache_capacity = segmented_.capacity();
  counters.segment_cache_used = segmented_.usedCount();
  counters.segment_cache_insert_fail = segmented_.insertFailCount();
  counters.name_cache_slots_capacity =
      static_cast<uint32_t>(maxActiveAmps_) * kNameSegmentFieldCount * nameSegmentMaxSegments_;
  counters.firmware_segment_cache_slots_capacity =
      static_cast<uint32_t>(maxActiveAmps_) * kFirmwareSegmentFieldCount * nameSegmentMaxSegments_;

  uint32_t usedCaches = 0;
  uint32_t usedNameCacheSlots = 0;
  uint32_t usedFirmwareCacheSlots = 0;
  for (uint8_t ampIndex = 0; ampIndex < activeAmpCount_; ++ampIndex) {
    const AmpState& ampState = amps_[ampIndex];
    if (!ampState.used || ampState.caches == 0) {
      continue;
    }
    for (uint16_t cacheIndex = 0; cacheIndex < ampState.cacheCapacity; ++cacheIndex) {
      if (ampState.caches[cacheIndex].valid) {
        ++usedCaches;
      }
    }
    for (uint8_t fieldIndex = 0; fieldIndex < kSegmentedReplyCacheFieldCount; ++fieldIndex) {
      for (uint8_t segmentIndex = 0; segmentIndex < nameSegmentMaxSegments_; ++segmentIndex) {
        const size_t offset = (static_cast<size_t>(ampIndex) * kSegmentedReplyCacheFieldCount + fieldIndex) *
                                  nameSegmentMaxSegments_ +
                              segmentIndex;
        if (nameSegmentCachePool_ != 0 && nameSegmentCachePool_[offset].valid) {
          if (fieldIndex < kNameSegmentFieldCount) {
            ++usedNameCacheSlots;
          } else {
            ++usedFirmwareCacheSlots;
          }
        }
      }
    }
  }
  counters.field_cache_used = usedCaches;
  counters.name_cache_slots_used = usedNameCacheSlots;
  counters.firmware_segment_cache_slots_used = usedFirmwareCacheSlots;
}

int8_t GatewayState::nameFieldIndex(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::settings_device_name:
      return 0;
    case FieldId::config_name_a:
      return 1;
    case FieldId::config_name_b:
      return 2;
    case FieldId::error_text:
      return 3;
    case FieldId::cherr_error_text_a:
      return 4;
    case FieldId::cherr_error_text_b:
      return 5;
    case FieldId::fixed_firmware_version:
      return 6;
    default:
      return -1;
  }
}

uint8_t GatewayState::ampIndex(uint8_t subnet, uint8_t device) const {
  const uint16_t slot = addressIndex(subnet, device);
  return addressToAmpIndex_[slot];
}

GatewayState::NameSegmentCacheEntry* GatewayState::nameSegmentEntry(uint8_t ampIndexValue,
                                                                    FieldId fieldId,
                                                                    uint8_t segmentNumber) {
  const int8_t fieldIndex = nameFieldIndex(fieldId);
  if (nameSegmentCachePool_ == 0 ||
      ampIndexValue >= maxActiveAmps_ ||
      fieldIndex < 0 ||
      segmentNumber >= nameSegmentMaxSegments_) {
    return 0;
  }
  const size_t offset =
      (static_cast<size_t>(ampIndexValue) * kSegmentedReplyCacheFieldCount + static_cast<size_t>(fieldIndex)) *
          nameSegmentMaxSegments_ +
      segmentNumber;
  return &nameSegmentCachePool_[offset];
}

const GatewayState::NameSegmentCacheEntry* GatewayState::nameSegmentEntry(uint8_t ampIndexValue,
                                                                          FieldId fieldId,
                                                                          uint8_t segmentNumber) const {
  const int8_t fieldIndex = nameFieldIndex(fieldId);
  if (nameSegmentCachePool_ == 0 ||
      ampIndexValue >= maxActiveAmps_ ||
      fieldIndex < 0 ||
      segmentNumber >= nameSegmentMaxSegments_) {
    return 0;
  }
  const size_t offset =
      (static_cast<size_t>(ampIndexValue) * kSegmentedReplyCacheFieldCount + static_cast<size_t>(fieldIndex)) *
          nameSegmentMaxSegments_ +
      segmentNumber;
  return &nameSegmentCachePool_[offset];
}

void GatewayState::clearNameSegmentEntries() {
  if (nameSegmentCachePool_ == 0) {
    return;
  }
  const size_t capacity =
      static_cast<size_t>(maxActiveAmps_) * kSegmentedReplyCacheFieldCount * nameSegmentMaxSegments_;
  for (size_t index = 0; index < capacity; ++index) {
    nameSegmentCachePool_[index] = {};
  }
}

void GatewayState::clearNameSegmentEntriesForAmp(uint8_t ampIndexValue) {
  if (nameSegmentCachePool_ == 0 || ampIndexValue >= maxActiveAmps_) {
    return;
  }
  for (uint8_t fieldIndex = 0; fieldIndex < kSegmentedReplyCacheFieldCount; ++fieldIndex) {
    for (uint8_t segmentIndex = 0; segmentIndex < nameSegmentMaxSegments_; ++segmentIndex) {
      const size_t offset = (static_cast<size_t>(ampIndexValue) * kSegmentedReplyCacheFieldCount + fieldIndex) *
                                nameSegmentMaxSegments_ +
                            segmentIndex;
      nameSegmentCachePool_[offset] = {};
    }
  }
}

void GatewayState::clearNameSegmentFieldForAmp(uint8_t ampIndexValue, FieldId fieldId) {
  const int8_t fieldIndex = nameFieldIndex(fieldId);
  if (nameSegmentCachePool_ == 0 || ampIndexValue >= maxActiveAmps_ || fieldIndex < 0) {
    return;
  }
  for (uint8_t segmentIndex = 0; segmentIndex < nameSegmentMaxSegments_; ++segmentIndex) {
    const size_t offset =
        (static_cast<size_t>(ampIndexValue) * kSegmentedReplyCacheFieldCount + static_cast<size_t>(fieldIndex)) *
            nameSegmentMaxSegments_ +
        segmentIndex;
    nameSegmentCachePool_[offset] = {};
  }
}

bool GatewayState::storeNameSegmentFromCan(const NormalizedRequest& reply,
                                           uint32_t nowMs,
                                           DebugCounters& counters) {
  if (!reply.valid ||
      (!isNameSegmentField(reply.fieldId) && !isFirmwareSegmentField(reply.fieldId)) ||
      !isSegmentedType(reply.admin.telegramType) ||
      !reply.objectAddress.hasSegment) {
    return false;
  }

  AmpState* ampState = amp(reply.canId.subnet, reply.canId.device, true);
  if (ampState == 0) {
    return false;
  }

  const uint8_t mappedAmpIndex = ampIndex(reply.canId.subnet, reply.canId.device);
  NameSegmentCacheEntry* entry =
      nameSegmentEntry(mappedAmpIndex, reply.fieldId, reply.objectAddress.segmentNumber);
  if (entry == 0) {
    return false;
  }

  const bool wasValid = entry->valid;
  entry->valid = true;
  entry->lastUpdateMs = nowMs;
  entry->length = reply.frame.length > nameSegmentPayloadMaxLen_ ? nameSegmentPayloadMaxLen_
                                                                 : reply.frame.length;
  for (uint8_t index = 0; index < entry->length; ++index) {
    entry->payload[index] = reply.frame.data[index];
  }
  for (uint8_t index = entry->length; index < kNameSegmentPayloadMaxLen; ++index) {
    entry->payload[index] = 0;
  }
  if (!wasValid) {
    if (isNameSegmentField(reply.fieldId)) {
      ++counters.name_segments_learned;
    } else if (isFirmwareSegmentField(reply.fieldId)) {
      ++counters.firmware_segment_cache_learned;
    }
  }
  return true;
}

AmpState* GatewayState::amp(uint8_t subnet, uint8_t device, bool createIfMissing) {
  const uint16_t slot = addressIndex(subnet, device);
  const uint8_t mapped = addressToAmpIndex_[slot];
  if (mapped != 0xFF && mapped < maxActiveAmps_) {
    return &amps_[mapped];
  }

  if (!createIfMissing || activeAmpCount_ >= maxActiveAmps_) {
    return 0;
  }

  const uint8_t ampIndex = activeAmpCount_++;
  addressToAmpIndex_[slot] = ampIndex;
  AmpState& ampState = amps_[ampIndex];
  ampState.clear();
  ampState.used = true;
  ampState.subnet = subnet;
  ampState.device = device;
  ampState.tcpNode = static_cast<uint8_t>((subnet << 6) | (device & 0x3F));
  clearNameSegmentEntriesForAmp(ampIndex);
  return &ampState;
}

const AmpState* GatewayState::amp(uint8_t subnet, uint8_t device) const {
  const uint16_t slot = addressIndex(subnet, device);
  const uint8_t mapped = addressToAmpIndex_[slot];
  if (mapped == 0xFF || mapped >= maxActiveAmps_) {
    return 0;
  }
  return &amps_[mapped];
}

bool GatewayState::shouldSuppressPending(const NormalizedRequest& request) const {
  if (!request.valid || request.mapping == 0 || !request.mapping->allowPollSuppress ||
      request.mapping->behaviorConfidence != BehaviorConfidence::ConfirmedRoundTrip) {
    return false;
  }

  for (uint8_t index = 0; index < kMaxPendingRequests; ++index) {
    if (pending_[index].used && codec::sameLogicalShape(pending_[index].request, request)) {
      return true;
    }
  }
  return false;
}

bool GatewayState::shouldSuppressRecent(const NormalizedRequest& request, uint32_t nowMs) const {
  if (!request.valid || request.mapping == 0 || !request.mapping->allowPollSuppress ||
      request.mapping->behaviorConfidence != BehaviorConfidence::ConfirmedRoundTrip) {
    return false;
  }

  for (uint8_t index = 0; index < kMaxRecentRequests; ++index) {
    const RecentRequest& recent = recent_[index];
    if (!recent.used || static_cast<int32_t>(nowMs - (recent.sentMs + 250)) >= 0) {
      continue;
    }
    if (codec::sameLogicalShape(recent.request, request)) {
      return true;
    }
  }
  return false;
}

void GatewayState::rememberSentRequest(const NormalizedRequest& request, uint32_t nowMs) {
  RecentRequest& recent = recent_[recentHead_];
  recent.used = true;
  recent.sentMs = nowMs;
  recent.request = request;
  recentHead_ = static_cast<uint8_t>((recentHead_ + 1) % kMaxRecentRequests);
}

void GatewayState::addPending(const NormalizedRequest& request,
                              uint8_t tcpNode,
                              uint32_t nowMs,
                              uint32_t timeoutMs) {
  for (uint8_t index = 0; index < kMaxPendingRequests; ++index) {
    if (!pending_[index].used) {
      pending_[index].used = true;
      pending_[index].tcpNode = tcpNode;
      pending_[index].startedMs = nowMs;
      pending_[index].expiresMs = nowMs + timeoutMs;
      pending_[index].request = request;
      return;
    }
  }
}

bool GatewayState::completePending(const NormalizedRequest& reply, uint8_t& tcpNode) {
  tcpNode = 0;
  for (uint8_t index = 0; index < kMaxPendingRequests; ++index) {
    if (pending_[index].matchesReply(reply)) {
      tcpNode = pending_[index].tcpNode;
      pending_[index].clear();
      return true;
    }
  }
  return false;
}

bool GatewayState::updateCacheFromCan(const NormalizedRequest& reply,
                                      uint32_t nowMs,
                                      bool fromWrite,
                                      DebugCounters& counters,
                                      const DebugCli* debugCli) {
  if (!reply.valid || reply.mapping == 0 || !reply.mapping->allowDecode ||
      !cacheAllowedFromBehavior(reply.mapping->behaviorConfidence)) {
    return false;
  }

  AmpState* ampState = amp(reply.canId.subnet, reply.canId.device, true);
  if (ampState == 0) {
    ++counters.field_cache_insert_fail;
    return false;
  }
  ampState->lastSeenMs = nowMs;
  ampState->lastCanRxMs = nowMs;

  const uint8_t offset = dataOffset(reply);
  if (offset >= reply.frame.length) {
    return false;
  }

  const uint8_t length = static_cast<uint8_t>(reply.frame.length - offset);
  FieldCache* cache = ampState->cache(reply.fieldId);
  if (cache == 0) {
    ++counters.field_cache_insert_fail;
    return false;
  }

  if (isSegmentedType(reply.admin.telegramType)) {
    storeNameSegmentFromCan(reply, nowMs, counters);
    SegmentedResult segmented = {};
    if (!segmented_.ingest(reply, nowMs, segmented)) {
      return false;
    }
    if (!segmented.completed) {
      ++counters.segmented_started;
      return false;
    }
    cache->set(reply.fieldId, segmented.bytes, segmented.length, nowMs);
    ++counters.segmented_completed;
    if (debugCli != 0 && isNameSegmentField(reply.fieldId)) {
      ++counters.name_assembled_count;
      debugCli->logNameAssembled(reply.canId.subnet, reply.canId.device, reply.fieldId,
                                 segmented.bytes, segmented.length);
    }
  } else {
    // 0502/2602/4602 are the amplifier's persisted "description of last
    // error" fields.  A change in the live status or error number does not
    // invalidate that history: captures show the status returning to OK while
    // the last-error text deliberately remains available.
    cache->set(reply.fieldId, reply.frame.data + offset, length, nowMs);
    cache->setReplyFrame(reply.frame);
  }

  if (fromWrite) {
    ++counters.cache_updates_from_write;
  } else {
    ++counters.cache_updates_from_can;
  }
  return true;
}

bool GatewayState::updateCacheFromWrite(const NormalizedRequest& request,
                                        uint32_t nowMs,
                                        DebugCounters& counters) {
  if (!request.valid || request.mapping == 0 || !request.mapping->allowWrite) {
    return false;
  }

  AmpState* ampState = amp(request.canId.subnet, request.canId.device, true);
  if (ampState == 0) {
    ++counters.field_cache_insert_fail;
    return false;
  }
  ampState->lastSeenMs = nowMs;

  const uint8_t offset = dataOffset(request);
  if (offset >= request.frame.length) {
    return false;
  }

  FieldCache* cache = ampState->cache(request.fieldId);
  if (cache == 0) {
    ++counters.field_cache_insert_fail;
    return false;
  }
  cache->set(request.fieldId, request.frame.data + offset,
             static_cast<uint8_t>(request.frame.length - offset), nowMs);
  ++counters.cache_updates_from_write;
  return true;
}

bool GatewayState::cacheLookup(const NormalizedRequest& request, FieldCache& cache) const {
  const AmpState* ampState = amp(request.canId.subnet, request.canId.device);
  if (ampState == 0) {
    return false;
  }
  const FieldCache* found = ampState->cache(request.fieldId);
  if (found == 0 || !found->valid) {
    return false;
  }
  cache = *found;
  return true;
}

bool GatewayState::buildCacheReply(const NormalizedRequest& request,
                                   uint32_t ttlMs,
                                   uint32_t nowMs,
                                   Frame& replyFrame,
                                   uint32_t& ageMs,
                                   bool& stale,
                                   uint32_t nodeFreshMaxAgeMs) const {
  replyFrame = {};
  ageMs = 0;
  stale = false;
  if (!request.valid || request.mapping == 0 || !request.mapping->allowCacheReply ||
      request.mapping->behaviorConfidence != BehaviorConfidence::ConfirmedRoundTrip) {
    return false;
  }

  FieldCache cache = {};
  if (!cacheLookup(request, cache)) {
    return false;
  }
  if (!cache.hasReplyFrame) {
    return false;
  }
  const uint32_t valueAgeMs = nowMs - cache.lastUpdateMs;
  const AmpState* ampState = amp(request.canId.subnet, request.canId.device);
  const uint32_t nodeCanAgeMs =
      ampState != 0 && ampState->lastCanRxMs != 0 && ampState->lastCanRxMs <= nowMs
          ? nowMs - ampState->lastCanRxMs
          : 0xFFFFFFFFu;
  ageMs = nodeFreshMaxAgeMs != 0 && nodeCanAgeMs <= nodeFreshMaxAgeMs && nodeCanAgeMs < valueAgeMs
              ? nodeCanAgeMs
              : valueAgeMs;
  if (ttlMs != 0 && ageMs >= ttlMs) {
    stale = true;
    return false;
  }
  replyFrame = cache.replyFrame;
  return true;
}

bool GatewayState::buildObservedFrameCacheReply(const NormalizedRequest& request,
                                                uint32_t ttlMs,
                                                uint32_t nowMs,
                                                Frame& replyFrame,
                                                uint32_t& ageMs,
                                                bool& stale,
                                                uint32_t nodeFreshMaxAgeMs) const {
  replyFrame = {};
  ageMs = 0;
  stale = false;
  if (!request.valid) {
    return false;
  }

  FieldCache cache = {};
  if (!cacheLookup(request, cache) || !cache.hasReplyFrame) {
    return false;
  }
  const uint32_t valueAgeMs = nowMs - cache.lastUpdateMs;
  const AmpState* ampState = amp(request.canId.subnet, request.canId.device);
  const uint32_t nodeCanAgeMs =
      ampState != 0 && ampState->lastCanRxMs != 0 && ampState->lastCanRxMs <= nowMs
          ? nowMs - ampState->lastCanRxMs
          : 0xFFFFFFFFu;
  ageMs = nodeFreshMaxAgeMs != 0 && nodeCanAgeMs <= nodeFreshMaxAgeMs && nodeCanAgeMs < valueAgeMs
              ? nodeCanAgeMs
              : valueAgeMs;
  if (ttlMs != 0 && ageMs >= ttlMs) {
    stale = true;
    return false;
  }
  replyFrame = cache.replyFrame;
  return true;
}

bool GatewayState::buildNameSegmentCacheReply(const NormalizedRequest& request,
                                              uint32_t ttlMs,
                                              uint32_t nowMs,
                                              Frame& replyFrame,
                                              uint32_t& ageMs,
                                              NameSegmentCacheMissReason& missReason,
                                              uint32_t nodeFreshMaxAgeMs,
                                              bool requireCompleteSet) const {
  replyFrame = {};
  ageMs = 0;
  missReason = NameSegmentCacheMissReason::NoField;
  if (!request.valid ||
      !isReadFunction(request.admin.functionCode) ||
      !isSegmentedType(request.admin.telegramType) ||
      !request.objectAddress.hasSegment) {
    return false;
  }

  const int8_t fieldIndex = nameFieldIndex(request.fieldId);
  if (fieldIndex < 0) {
    missReason = NameSegmentCacheMissReason::NoField;
    return false;
  }

  const AmpState* ampState = amp(request.canId.subnet, request.canId.device);
  const uint8_t mappedAmpIndex = ampIndex(request.canId.subnet, request.canId.device);
  if (mappedAmpIndex == 0xFF || mappedAmpIndex >= maxActiveAmps_) {
    missReason = NameSegmentCacheMissReason::NoSlot;
    return false;
  }

  const NameSegmentCacheEntry* entry =
      nameSegmentEntry(mappedAmpIndex, request.fieldId, request.objectAddress.segmentNumber);
  if (entry == 0 || !entry->valid || entry->length == 0) {
    missReason = NameSegmentCacheMissReason::NoSegment;
    return false;
  }

  if (requireCompleteSet) {
    for (uint8_t segment = 0; segment < nameSegmentMaxSegments_; ++segment) {
      const NameSegmentCacheEntry* completeEntry =
          nameSegmentEntry(mappedAmpIndex, request.fieldId, segment);
      if (completeEntry == 0 || !completeEntry->valid || completeEntry->length == 0) {
        missReason = NameSegmentCacheMissReason::NoSegment;
        return false;
      }
    }
  }

  const uint32_t valueAgeMs = nowMs - entry->lastUpdateMs;
  const uint32_t nodeCanAgeMs =
      ampState != 0 && ampState->lastCanRxMs != 0 && ampState->lastCanRxMs <= nowMs
          ? nowMs - ampState->lastCanRxMs
          : 0xFFFFFFFFu;
  ageMs = nodeFreshMaxAgeMs != 0 && nodeCanAgeMs <= nodeFreshMaxAgeMs && nodeCanAgeMs < valueAgeMs
              ? nodeCanAgeMs
              : valueAgeMs;
  if (ttlMs != 0 && ageMs >= ttlMs) {
    missReason = NameSegmentCacheMissReason::TtlExpired;
    return false;
  }

  replyFrame.canId =
      static_cast<uint16_t>(0x400u | (static_cast<uint16_t>(request.canId.subnet) << 6) | request.canId.device);
  replyFrame.length = entry->length;
  for (uint8_t index = 0; index < entry->length; ++index) {
    replyFrame.data[index] = entry->payload[index];
  }
  missReason = NameSegmentCacheMissReason::None;
  return true;
}

bool GatewayState::buildUnknown0111CacheReply(uint8_t subnet,
                                              uint8_t device,
                                              uint32_t ttlMs,
                                              uint32_t nowMs,
                                              Frame& replyFrame,
                                              uint32_t& ageMs,
                                              bool& stale,
                                              uint32_t nodeFreshMaxAgeMs) const {
  replyFrame = {};
  ageMs = 0;
  stale = false;
  const AmpState* ampState = amp(subnet, device);
  if (ampState == 0 || !ampState->unknown0111CacheValid) {
    return false;
  }
  const uint32_t valueAgeMs = nowMs - ampState->unknown0111CacheUpdateMs;
  const uint32_t nodeCanAgeMs =
      ampState->lastCanRxMs != 0 && ampState->lastCanRxMs <= nowMs
          ? nowMs - ampState->lastCanRxMs
          : 0xFFFFFFFFu;
  ageMs = nodeFreshMaxAgeMs != 0 && nodeCanAgeMs <= nodeFreshMaxAgeMs && nodeCanAgeMs < valueAgeMs
              ? nodeCanAgeMs
              : valueAgeMs;
  if (ttlMs != 0 && ageMs >= ttlMs) {
    stale = true;
    return false;
  }
  replyFrame = ampState->unknown0111CacheFrame;
  return true;
}

void GatewayState::invalidateNameSegmentCache(uint8_t subnet,
                                              uint8_t device,
                                              FieldId fieldId,
                                              const char*,
                                              DebugCounters& counters,
                                              const DebugCli*) {
  if (!isNameSegmentField(fieldId)) {
    return;
  }
  const uint8_t mappedAmpIndex = ampIndex(subnet, device);
  if (mappedAmpIndex != 0xFF && mappedAmpIndex < maxActiveAmps_) {
    clearNameSegmentFieldForAmp(mappedAmpIndex, fieldId);
  }
  ++counters.name_cache_invalidations;
}

void GatewayState::invalidateFieldCache(uint8_t subnet, uint8_t device, FieldId fieldId) {
  AmpState* ampState = amp(subnet, device, false);
  if (ampState == 0) {
    return;
  }
  FieldCache* cache = ampState->cache(fieldId);
  if (cache != 0) {
    cache->clear();
  }
}

void GatewayState::invalidateAllCachedValuesForNode(uint8_t subnet, uint8_t device) {
  AmpState* ampState = amp(subnet, device, false);
  if (ampState == 0) {
    return;
  }
  for (uint16_t index = 0; index < ampState->cacheCapacity; ++index) {
    ampState->caches[index].clear();
  }
  ampState->unknown0111CacheValid = false;
  ampState->unknown0111CacheUpdateMs = 0;
  ampState->unknown0111CacheFrame = {};
  for (uint8_t index = 0; index < AmpState::kLiveRefreshCount; ++index) {
    ampState->liveRefresh[index].learned = false;
    ampState->liveRefresh[index].pending = false;
    ampState->liveRefresh[index].sent = false;
  }
  ampState->unknown0111RefreshPending = false;
  ampState->object0100HeartbeatQueuedMs = 0;
  ampState->object0100HeartbeatLastTriggerMs = 0;
  const uint8_t mappedAmpIndex = ampIndex(subnet, device);
  if (mappedAmpIndex != 0xFF && mappedAmpIndex < maxActiveAmps_) {
    clearNameSegmentEntriesForAmp(mappedAmpIndex);
  }
}

void GatewayState::invalidateUnknown0111Cache(uint8_t subnet, uint8_t device) {
  AmpState* ampState = amp(subnet, device, false);
  if (ampState == 0) {
    return;
  }
  ampState->unknown0111CacheValid = false;
  ampState->unknown0111CacheUpdateMs = 0;
  ampState->unknown0111CacheFrame = {};
}

bool GatewayState::usingPsram() const {
  return usingPsram_;
}

bool GatewayState::fallbackMode() const {
  return fallbackMode_;
}

uint8_t GatewayState::activeAmpCount() const {
  return activeAmpCount_;
}

const AmpState* GatewayState::activeAmpByIndex(uint8_t index) const {
  if (index >= activeAmpCount_) {
    return 0;
  }
  const AmpState& ampState = amps_[index];
  return ampState.used ? &ampState : 0;
}

uint8_t GatewayState::maxActiveAmps() const {
  return maxActiveAmps_;
}

uint16_t GatewayState::fieldCacheCapacityPerAmp() const {
  return fieldCacheCapacityPerAmp_;
}

uint8_t GatewayState::segmentCachePerAmp() const {
  return kSegmentCachePerAmp;
}

uint16_t GatewayState::nameCacheSlotsCapacity() const {
  return static_cast<uint16_t>(kNameSegmentFieldCount * nameSegmentMaxSegments_);
}

uint8_t GatewayState::nameSegmentMaxSegments() const {
  return nameSegmentMaxSegments_;
}

uint8_t GatewayState::nameSegmentPayloadMaxLen() const {
  return nameSegmentPayloadMaxLen_;
}

uint32_t GatewayState::bytesAllocated() const {
  return bytesAllocated_;
}

void GatewayState::noteTcpTxToNode(uint8_t subnet, uint8_t device, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastTcpTxToR1Ms = nowMs;
  ampState->r1TimeoutActive = false;
  ampState->lastServiceGapLevel = 0;
  ++ampState->tcpTxCount;
}

void GatewayState::noteCanRxForNode(uint8_t subnet, uint8_t device, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastSeenMs = nowMs;
  ampState->lastCanRxMs = nowMs;
  ++ampState->realCanRxCount;
}

void GatewayState::noteCacheReplyForNode(uint8_t subnet, uint8_t device, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastTcpTxToR1Ms = nowMs;
  ampState->lastCacheReplyMs = nowMs;
  ampState->r1TimeoutActive = false;
  ampState->lastServiceGapLevel = 0;
  ++ampState->tcpTxCount;
  ++ampState->cacheReplyCount;
}

void GatewayState::noteCanTxPollForNode(uint8_t subnet, uint8_t device, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastCanTxPollMs = nowMs;
  ++ampState->canTxPollCount;
}

void GatewayState::noteTcpRxRequestForNode(uint8_t subnet, uint8_t device, FieldId fieldId, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastTcpRxRequestMs = nowMs;
  ampState->lastTcpRxRequestField = fieldId;
  ampState->r1TimeoutActive = false;
}

void GatewayState::noteTimeoutRisk(uint8_t subnet, uint8_t device) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, false);
  if (ampState == 0) {
    return;
  }
  ++ampState->timeoutRiskCount;
}

void GatewayState::noteUnknown0111Request(uint8_t subnet, uint8_t device, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastUnknown0111RequestMs = nowMs;
  ++ampState->unknown0111Requests;
  ++ampState->unknown0111Pending;
}

void GatewayState::noteUnknown0111CanTx(uint8_t subnet, uint8_t device, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastCanTxPollMs = nowMs;
  ++ampState->unknown0111CanTx;
}

void GatewayState::noteUnknown0111CanRx(uint8_t subnet, uint8_t device, const Frame& frame, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastUnknown0111RxMs = nowMs;
  ampState->unknown0111LastValue = frame.length >= 4 ? frame.data[3] : 0;
  ampState->unknown0111CacheValid = true;
  ampState->unknown0111CacheUpdateMs = nowMs;
  ampState->unknown0111CacheFrame = frame;
  ++ampState->unknown0111CanRx;
}

void GatewayState::noteUnknown0111TcpTx(uint8_t subnet, uint8_t device, uint8_t value, uint32_t nowMs) {
  if (device == 0) {
    return;
  }
  AmpState* ampState = amp(subnet, device, true);
  if (ampState == 0) {
    return;
  }
  ampState->lastTcpTxToR1Ms = nowMs;
  ampState->lastUnknown0111TcpTxMs = nowMs;
  ampState->unknown0111LastValue = value;
  ++ampState->unknown0111TcpTx;
  if (ampState->unknown0111Pending != 0) {
    --ampState->unknown0111Pending;
  }
}

}  // namespace r70v2
}  // namespace dbcan
