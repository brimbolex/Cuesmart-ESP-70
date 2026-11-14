#ifndef DBCAN_FRAME_H
#define DBCAN_FRAME_H

#include <stdint.h>

namespace dbcan {

static const uint8_t kMaxFrameDataLength = 8;
static const uint16_t kMaxStandardCanId = 0x7FF;

struct Frame {
  uint16_t canId;
  uint8_t length;
  uint8_t data[kMaxFrameDataLength];
};

}  // namespace dbcan

#endif  // DBCAN_FRAME_H

