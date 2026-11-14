#ifndef DB_R70_V2_R70_FRAME_H
#define DB_R70_V2_R70_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include "../DbCanFrame.h"

namespace dbcan {
namespace r70v2 {

#ifndef R70_ALLOW_UNVERIFIED_TCP_CAN_FORWARD
#define R70_ALLOW_UNVERIFIED_TCP_CAN_FORWARD 1
#endif

struct TcpEnvelope {
  bool valid;
  uint8_t rawLength;
  uint8_t payloadLength;
  uint8_t payload[16];
  uint8_t checksumByte;
  bool checksumChecked;
  bool checksumValid;
};

class R70Frame {
 public:
  static const uint8_t kStartByte = 0x43;
  static const uint8_t kEndByte = 0x0D;
  static const uint8_t kRequestType = 0x02;
  static const uint8_t kReplyType = 0x04;
  static const uint8_t kPayloadPrefix0 = 0x00;
  static const uint8_t kMaxPayloadLength = 16;
  static const uint8_t kMaxFrameLength = kMaxPayloadLength + 4;

  static bool parse(const uint8_t* bytes, size_t length, TcpEnvelope& envelope);
  static bool encode(const uint8_t* payload,
                     uint8_t payloadLength,
                     uint8_t* out,
                     size_t outCapacity,
                     uint8_t& outLength);
  static bool encodeWrappedCanFrame(const Frame& frame,
                                    uint8_t* out,
                                    size_t outCapacity,
                                    uint8_t& outLength);
  static uint8_t computeR70TcpXorCrc(const uint8_t* bytes, uint8_t length);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_R70_FRAME_H
