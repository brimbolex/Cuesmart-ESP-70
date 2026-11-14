#include "HotWarmColdQueues.h"

namespace dbcan {
namespace r70v2 {

template <uint16_t Capacity>
void HotWarmColdQueues::clearRing(Ring<Capacity>& ring) {
  ring = {};
}

template <uint16_t Capacity>
bool HotWarmColdQueues::enqueueRing(Ring<Capacity>& ring, const QueueEntry& entry) {
  if (ring.count >= Capacity) {
    return false;
  }
  ring.entries[ring.tail] = entry;
  ring.tail = static_cast<uint16_t>((ring.tail + 1) % Capacity);
  ++ring.count;
  return true;
}

template <uint16_t Capacity>
bool HotWarmColdQueues::enqueueFrontRing(Ring<Capacity>& ring, const QueueEntry& entry) {
  if (ring.count >= Capacity) {
    return false;
  }
  ring.head = static_cast<uint16_t>((ring.head + Capacity - 1) % Capacity);
  ring.entries[ring.head] = entry;
  ++ring.count;
  return true;
}

template <uint16_t Capacity>
bool HotWarmColdQueues::dequeueRing(Ring<Capacity>& ring, QueueEntry& entry) {
  if (ring.count == 0) {
    return false;
  }
  entry = ring.entries[ring.head];
  ring.head = static_cast<uint16_t>((ring.head + 1) % Capacity);
  --ring.count;
  return true;
}

bool HotWarmColdQueues::sameFrame(const NormalizedRequest& left, const NormalizedRequest& right) {
  if (!left.valid || !right.valid ||
      left.frame.canId != right.frame.canId ||
      left.frame.length != right.frame.length) {
    return false;
  }
  for (uint8_t index = 0; index < left.frame.length; ++index) {
    if (left.frame.data[index] != right.frame.data[index]) {
      return false;
    }
  }
  return true;
}

template <uint16_t Capacity>
bool HotWarmColdQueues::containsExactRing(const Ring<Capacity>& ring, const NormalizedRequest& request) {
  for (uint16_t index = 0; index < ring.count; ++index) {
    const uint16_t slot = static_cast<uint16_t>((ring.head + index) % Capacity);
    if (sameFrame(ring.entries[slot].request, request)) {
      return true;
    }
  }
  return false;
}

template <uint16_t Capacity>
uint32_t HotWarmColdQueues::oldestAgeMsRing(const Ring<Capacity>& ring, uint32_t nowMs) {
  if (ring.count == 0) {
    return 0;
  }
  const QueueEntry& entry = ring.entries[ring.head];
  return nowMs - entry.enqueuedMs;
}

template <uint16_t Capacity>
bool HotWarmColdQueues::ringContainsOtherSubnet(const Ring<Capacity>& ring, uint8_t subnet) {
  for (uint16_t index = 0; index < ring.count; ++index) {
    const uint16_t slot = static_cast<uint16_t>((ring.head + index) % Capacity);
    if (ring.entries[slot].request.canId.subnet != subnet) {
      return true;
    }
  }
  return false;
}

template <uint16_t Capacity>
bool HotWarmColdQueues::dequeueRingFirstOtherSubnet(Ring<Capacity>& ring, uint8_t subnet, QueueEntry& entry) {
  if (ring.count == 0) {
    return false;
  }
  uint16_t selectedOffset = Capacity;
  for (uint16_t offset = 0; offset < ring.count; ++offset) {
    const uint16_t slot = static_cast<uint16_t>((ring.head + offset) % Capacity);
    if (ring.entries[slot].request.canId.subnet != subnet) {
      selectedOffset = offset;
      break;
    }
  }
  if (selectedOffset >= ring.count) {
    return false;
  }

  const uint16_t selectedSlot = static_cast<uint16_t>((ring.head + selectedOffset) % Capacity);
  entry = ring.entries[selectedSlot];
  for (uint16_t offset = selectedOffset; (offset + 1u) < ring.count; ++offset) {
    const uint16_t fromSlot = static_cast<uint16_t>((ring.head + offset + 1u) % Capacity);
    const uint16_t toSlot = static_cast<uint16_t>((ring.head + offset) % Capacity);
    ring.entries[toSlot] = ring.entries[fromSlot];
  }
  ring.tail = static_cast<uint16_t>((ring.tail + Capacity - 1u) % Capacity);
  --ring.count;
  return true;
}

void HotWarmColdQueues::clear() {
  clearRing(write_);
  clearRing(maintenance_);
  clearRing(ident_);
  clearRing(normalRead_);
}

bool HotWarmColdQueues::enqueue(QueueClass queueClass, const QueueEntry& entry) {
  if (queueClass == QueueClass::Write) {
    return enqueueRing(write_, entry);
  }
  if (queueClass == QueueClass::Maintenance) {
    return enqueueRing(maintenance_, entry);
  }
  if (queueClass == QueueClass::Ident) {
    return enqueueRing(ident_, entry);
  }
  return enqueueRing(normalRead_, entry);
}

bool HotWarmColdQueues::enqueueFront(QueueClass queueClass, const QueueEntry& entry) {
  if (queueClass == QueueClass::Write) {
    return enqueueFrontRing(write_, entry);
  }
  if (queueClass == QueueClass::Maintenance) {
    return enqueueFrontRing(maintenance_, entry);
  }
  if (queueClass == QueueClass::Ident) {
    return enqueueFrontRing(ident_, entry);
  }
  return enqueueFrontRing(normalRead_, entry);
}

bool HotWarmColdQueues::peek(QueueEntry& entry, QueueClass& queueClass) const {
  if (write_.count > 0) {
    entry = write_.entries[write_.head];
    queueClass = QueueClass::Write;
    return true;
  }
  if (maintenance_.count > 0) {
    entry = maintenance_.entries[maintenance_.head];
    queueClass = QueueClass::Maintenance;
    return true;
  }
  if (ident_.count > 0) {
    entry = ident_.entries[ident_.head];
    queueClass = QueueClass::Ident;
    return true;
  }
  if (normalRead_.count > 0) {
    entry = normalRead_.entries[normalRead_.head];
    queueClass = QueueClass::NormalRead;
    return true;
  }
  return false;
}

bool HotWarmColdQueues::dequeue(QueueEntry& entry, QueueClass* queueClass) {
  if (dequeueRing(write_, entry)) {
    if (queueClass != 0) {
      *queueClass = QueueClass::Write;
    }
    return true;
  }
  if (dequeueRing(maintenance_, entry)) {
    if (queueClass != 0) {
      *queueClass = QueueClass::Maintenance;
    }
    return true;
  }
  if (dequeueRing(ident_, entry)) {
    if (queueClass != 0) {
      *queueClass = QueueClass::Ident;
    }
    return true;
  }
  if (dequeueRing(normalRead_, entry)) {
    if (queueClass != 0) {
      *queueClass = QueueClass::NormalRead;
    }
    return true;
  }
  return false;
}

uint16_t HotWarmColdQueues::size() const {
  return static_cast<uint16_t>(write_.count + maintenance_.count + ident_.count + normalRead_.count);
}

uint16_t HotWarmColdQueues::writeSize() const {
  return write_.count;
}

uint16_t HotWarmColdQueues::maintenanceSize() const {
  return maintenance_.count;
}

uint16_t HotWarmColdQueues::identSize() const {
  return ident_.count;
}

uint16_t HotWarmColdQueues::normalReadSize() const {
  return normalRead_.count;
}

bool HotWarmColdQueues::normalReadContainsOtherSubnet(uint8_t subnet) const {
  return ringContainsOtherSubnet(normalRead_, subnet);
}

bool HotWarmColdQueues::dequeueNormalReadFirstOtherSubnet(uint8_t subnet, QueueEntry& entry) {
  return dequeueRingFirstOtherSubnet(normalRead_, subnet, entry);
}

bool HotWarmColdQueues::containsExact(QueueClass queueClass, const NormalizedRequest& request) const {
  if (queueClass == QueueClass::Write) {
    return containsExactRing(write_, request);
  }
  if (queueClass == QueueClass::Maintenance) {
    return containsExactRing(maintenance_, request);
  }
  if (queueClass == QueueClass::Ident) {
    return containsExactRing(ident_, request);
  }
  return containsExactRing(normalRead_, request);
}

uint32_t HotWarmColdQueues::oldestAgeMs(QueueClass queueClass, uint32_t nowMs) const {
  if (queueClass == QueueClass::Write) {
    return oldestAgeMsRing(write_, nowMs);
  }
  if (queueClass == QueueClass::Maintenance) {
    return oldestAgeMsRing(maintenance_, nowMs);
  }
  if (queueClass == QueueClass::Ident) {
    return oldestAgeMsRing(ident_, nowMs);
  }
  return oldestAgeMsRing(normalRead_, nowMs);
}

}  // namespace r70v2
}  // namespace dbcan
