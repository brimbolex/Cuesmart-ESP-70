#include "SegmentedAssembler.h"

namespace dbcan {
namespace r70v2 {

namespace {

uint8_t segmentedDataOffset(const NormalizedRequest& reply) {
  switch (reply.admin.telegramType) {
    case TelegramType::Segmented:
      return 4;
    case TelegramType::FileSegmented:
      return 6;
    default:
      return reply.frame.length;
  }
}

}  // namespace

SegmentedAssembler::SegmentedAssembler()
    : slots_(0),
      capacity_(0),
      insertFailCount_(0) {}

void SegmentedAssembler::begin(void* storage, uint16_t capacity) {
  slots_ = static_cast<Slot*>(storage);
  capacity_ = capacity;
  insertFailCount_ = 0;
  clear();
}

void SegmentedAssembler::clear() {
  if (slots_ == 0) {
    return;
  }
  for (uint16_t index = 0; index < capacity_; ++index) {
    slots_[index] = {};
  }
}

SegmentedAssembler::Slot* SegmentedAssembler::slotFor(const NormalizedRequest& reply) {
  if (slots_ == 0) {
    return 0;
  }

  Slot* empty = 0;
  for (uint16_t index = 0; index < capacity_; ++index) {
    Slot& slot = slots_[index];
    if (slot.used &&
        slot.subnet == reply.canId.subnet &&
        slot.device == reply.canId.device &&
        slot.fieldId == reply.fieldId &&
        slot.recordNumber == reply.objectAddress.recordNumber) {
      return &slot;
    }
    if (!slot.used && empty == 0) {
      empty = &slot;
    }
  }

  if (empty != 0) {
    *empty = {};
    empty->used = true;
    empty->subnet = reply.canId.subnet;
    empty->device = reply.canId.device;
    empty->fieldId = reply.fieldId;
    empty->recordNumber = reply.objectAddress.recordNumber;
    return empty;
  }

  ++insertFailCount_;
  return 0;
}

bool SegmentedAssembler::ingest(const NormalizedRequest& reply, uint32_t nowMs, SegmentedResult& result) {
  result = {};
  if (!reply.valid || !isSegmentedType(reply.admin.telegramType) || !reply.objectAddress.hasSegment) {
    return false;
  }

  Slot* slot = slotFor(reply);
  if (slot == 0) {
    return false;
  }

  slot->lastUpdateMs = nowMs;
  if (reply.objectAddress.segmentNumber > slot->highestSegmentSeen) {
    slot->highestSegmentSeen = reply.objectAddress.segmentNumber;
  }
  if (reply.objectAddress.segmentNumber < 16) {
    slot->seenMask = static_cast<uint16_t>(slot->seenMask |
                                           (1u << reply.objectAddress.segmentNumber));
  }

  const uint8_t dataOffset = segmentedDataOffset(reply);
  if (dataOffset >= reply.frame.length) {
    return false;
  }

  const uint8_t chunkLength = static_cast<uint8_t>(reply.frame.length - dataOffset);
  const uint8_t writeOffset = static_cast<uint8_t>(reply.objectAddress.segmentNumber * chunkLength);
  for (uint8_t index = 0; index < chunkLength; ++index) {
    const uint8_t target = static_cast<uint8_t>(writeOffset + index);
    if (target >= sizeof(slot->bytes)) {
      break;
    }
    slot->bytes[target] = reply.frame.data[dataOffset + index];
    if (target + 1 > slot->length) {
      slot->length = static_cast<uint8_t>(target + 1);
    }
  }

  const uint16_t neededMask = static_cast<uint16_t>((1u << (slot->highestSegmentSeen + 1)) - 1u);
  if ((slot->seenMask & neededMask) != neededMask || (slot->seenMask & 0x01u) == 0u) {
    return true;
  }

  result.completed = true;
  result.length = slot->length;
  for (uint8_t index = 0; index < slot->length; ++index) {
    result.bytes[index] = slot->bytes[index];
  }
  *slot = {};
  return true;
}

uint8_t SegmentedAssembler::expire(uint32_t nowMs) {
  if (slots_ == 0) {
    return 0;
  }
  uint8_t expired = 0;
  for (uint16_t index = 0; index < capacity_; ++index) {
    Slot& slot = slots_[index];
    if (!slot.used) {
      continue;
    }
    if (static_cast<int32_t>(nowMs - (slot.lastUpdateMs + kTimeoutMs)) >= 0) {
      slot = {};
      ++expired;
    }
  }
  return expired;
}

uint16_t SegmentedAssembler::usedCount() const {
  if (slots_ == 0) {
    return 0;
  }
  uint16_t used = 0;
  for (uint16_t index = 0; index < capacity_; ++index) {
    if (slots_[index].used) {
      ++used;
    }
  }
  return used;
}

uint16_t SegmentedAssembler::capacity() const {
  return capacity_;
}

uint32_t SegmentedAssembler::insertFailCount() const {
  return insertFailCount_;
}

}  // namespace r70v2
}  // namespace dbcan
