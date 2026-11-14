#ifndef DB_R70_V2_FIELD_CACHE_H
#define DB_R70_V2_FIELD_CACHE_H

#include <stdint.h>

#include "../DbCanFrame.h"
#include "../protocol/DbCanMapping.h"

namespace dbcan {
namespace r70v2 {

struct FieldCache {
  FieldId fieldId;
  bool valid;
  uint32_t lastUpdateMs;
  uint8_t valueLength;
  uint8_t value[kMaxFieldValueBytes];
  bool hasReplyFrame;
  Frame replyFrame;

  void clear();
  void set(FieldId id, const uint8_t* bytes, uint8_t length, uint32_t nowMs);
  void setReplyFrame(const Frame& frame);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_FIELD_CACHE_H
