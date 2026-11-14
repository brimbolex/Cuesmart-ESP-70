#include "FieldCache.h"

namespace dbcan {
namespace r70v2 {

void FieldCache::clear() {
  fieldId = FieldId::Unknown;
  valid = false;
  lastUpdateMs = 0;
  valueLength = 0;
  hasReplyFrame = false;
  replyFrame = {};
  for (uint8_t index = 0; index < sizeof(value); ++index) {
    value[index] = 0;
  }
}

void FieldCache::set(FieldId id, const uint8_t* bytes, uint8_t length, uint32_t nowMs) {
  clear();
  fieldId = id;
  valid = true;
  lastUpdateMs = nowMs;
  valueLength = length > sizeof(value) ? sizeof(value) : length;
  for (uint8_t index = 0; index < valueLength; ++index) {
    value[index] = bytes[index];
  }
}

void FieldCache::setReplyFrame(const Frame& frame) {
  replyFrame = frame;
  hasReplyFrame = true;
}

}  // namespace r70v2
}  // namespace dbcan
