#ifndef DB_R70_V2_HOT_WARM_COLD_QUEUES_H
#define DB_R70_V2_HOT_WARM_COLD_QUEUES_H

#include <stdint.h>

#include "../protocol/DbCanMapping.h"

namespace dbcan {
namespace r70v2 {

enum class QueueClass : uint8_t {
  Write,
  Maintenance,
  Ident,
  NormalRead
};

struct QueueEntry {
  NormalizedRequest request;
  uint8_t tcpNode;
  bool expectReply;
  uint32_t enqueuedMs;
};

class HotWarmColdQueues {
 public:
  static const uint16_t kWriteCapacity = 128;
  static const uint16_t kMaintenanceCapacity = 64;
  static const uint16_t kIdentCapacity = 64;
  static const uint16_t kNormalReadCapacity = 512;
  static const uint16_t kTotalCapacity =
      kWriteCapacity + kMaintenanceCapacity + kIdentCapacity + kNormalReadCapacity;

  void clear();
  bool enqueue(QueueClass queueClass, const QueueEntry& entry);
  bool enqueueFront(QueueClass queueClass, const QueueEntry& entry);
  bool peek(QueueEntry& entry, QueueClass& queueClass) const;
  bool dequeue(QueueEntry& entry, QueueClass* queueClass = 0);
  uint16_t size() const;
  uint16_t writeSize() const;
  uint16_t maintenanceSize() const;
  uint16_t identSize() const;
  uint16_t normalReadSize() const;
  bool normalReadContainsOtherSubnet(uint8_t subnet) const;
  bool dequeueNormalReadFirstOtherSubnet(uint8_t subnet, QueueEntry& entry);
  bool containsExact(QueueClass queueClass, const NormalizedRequest& request) const;
  uint32_t oldestAgeMs(QueueClass queueClass, uint32_t nowMs) const;

 private:
  template <uint16_t Capacity>
  struct Ring {
    QueueEntry entries[Capacity];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
  };

  Ring<kWriteCapacity> write_;
  Ring<kMaintenanceCapacity> maintenance_;
  Ring<kIdentCapacity> ident_;
  Ring<kNormalReadCapacity> normalRead_;

  template <uint16_t Capacity>
  static void clearRing(Ring<Capacity>& ring);
  template <uint16_t Capacity>
  static bool enqueueRing(Ring<Capacity>& ring, const QueueEntry& entry);
  template <uint16_t Capacity>
  static bool enqueueFrontRing(Ring<Capacity>& ring, const QueueEntry& entry);
  template <uint16_t Capacity>
  static bool dequeueRing(Ring<Capacity>& ring, QueueEntry& entry);
  static bool sameFrame(const NormalizedRequest& left, const NormalizedRequest& right);
  template <uint16_t Capacity>
  static bool containsExactRing(const Ring<Capacity>& ring, const NormalizedRequest& request);
  template <uint16_t Capacity>
  static uint32_t oldestAgeMsRing(const Ring<Capacity>& ring, uint32_t nowMs);
  template <uint16_t Capacity>
  static bool ringContainsOtherSubnet(const Ring<Capacity>& ring, uint8_t subnet);
  template <uint16_t Capacity>
  static bool dequeueRingFirstOtherSubnet(Ring<Capacity>& ring, uint8_t subnet, QueueEntry& entry);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_HOT_WARM_COLD_QUEUES_H
