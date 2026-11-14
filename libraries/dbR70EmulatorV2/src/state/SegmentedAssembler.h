#ifndef DB_R70_V2_SEGMENTED_ASSEMBLER_H
#define DB_R70_V2_SEGMENTED_ASSEMBLER_H

#include <stdint.h>

#include "../protocol/DbCanMapping.h"

namespace dbcan {
namespace r70v2 {

struct SegmentedResult {
  bool completed;
  uint8_t length;
  uint8_t bytes[kMaxFieldValueBytes];
};

class SegmentedAssembler {
 public:
  static const uint32_t kTimeoutMs = 2000;

  struct Slot {
    bool used;
    uint8_t subnet;
    uint8_t device;
    FieldId fieldId;
    uint16_t recordNumber;
    uint8_t highestSegmentSeen;
    uint16_t seenMask;
    uint8_t length;
    uint32_t lastUpdateMs;
    uint8_t bytes[kMaxFieldValueBytes];
  };

  SegmentedAssembler();

  void begin(void* storage, uint16_t capacity);
  void clear();
  bool ingest(const NormalizedRequest& reply, uint32_t nowMs, SegmentedResult& result);
  uint8_t expire(uint32_t nowMs);
  uint16_t usedCount() const;
  uint16_t capacity() const;
  uint32_t insertFailCount() const;

 private:
  Slot* slots_;
  uint16_t capacity_;
  uint32_t insertFailCount_;

  Slot* slotFor(const NormalizedRequest& reply);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_SEGMENTED_ASSEMBLER_H
