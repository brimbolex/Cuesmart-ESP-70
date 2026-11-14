#include "R70Frame.h"

#include "../DbCanFrame.h"

namespace dbcan {
namespace r70v2 {

bool R70Frame::parse(const uint8_t* bytes, size_t length, TcpEnvelope& envelope) {
  envelope = {};
  if (bytes == 0 || length < 7 || length > kMaxFrameLength) {
    return false;
  }
  if (bytes[0] != kStartByte || bytes[length - 1] != kEndByte) {
    return false;
  }

  const uint8_t payloadLength = bytes[1];
  if (payloadLength + 4 != length || payloadLength > kMaxPayloadLength) {
    return false;
  }

  envelope.checksumByte = bytes[length - 2];
  envelope.checksumChecked = true;
  envelope.checksumValid =
      computeR70TcpXorCrc(bytes, static_cast<uint8_t>(length - 2)) == bytes[length - 2];
  if (!envelope.checksumValid && !R70_ALLOW_UNVERIFIED_TCP_CAN_FORWARD) {
    return false;
  }

  envelope.valid = true;
  envelope.rawLength = static_cast<uint8_t>(length);
  envelope.payloadLength = payloadLength;
  for (uint8_t index = 0; index < payloadLength; ++index) {
    envelope.payload[index] = bytes[2 + index];
  }
  return true;
}

bool R70Frame::encode(const uint8_t* payload,
                      uint8_t payloadLength,
                      uint8_t* out,
                      size_t outCapacity,
                      uint8_t& outLength) {
  outLength = 0;
  if (payload == 0 || out == 0 || payloadLength > kMaxPayloadLength ||
      outCapacity < static_cast<size_t>(payloadLength + 4)) {
    return false;
  }

  out[0] = kStartByte;
  out[1] = payloadLength;
  for (uint8_t index = 0; index < payloadLength; ++index) {
    out[2 + index] = payload[index];
  }
  out[payloadLength + 2] = computeR70TcpXorCrc(out, static_cast<uint8_t>(payloadLength + 2));
  out[payloadLength + 3] = kEndByte;
  outLength = static_cast<uint8_t>(payloadLength + 4);
  return true;
}

bool R70Frame::encodeWrappedCanFrame(const Frame& frame,
                                     uint8_t* out,
                                     size_t outCapacity,
                                     uint8_t& outLength) {
  outLength = 0;
  if (out == 0 || frame.length > kMaxFrameDataLength) {
    return false;
  }

  uint8_t payload[kMaxPayloadLength] = {};
  const uint8_t payloadLength = static_cast<uint8_t>(3 + frame.length);
  if (payloadLength > kMaxPayloadLength) {
    return false;
  }

  payload[0] = kPayloadPrefix0;
  payload[1] = static_cast<uint8_t>((frame.canId >> 8) & 0xFF);
  payload[2] = static_cast<uint8_t>(frame.canId & 0xFF);
  for (uint8_t index = 0; index < frame.length; ++index) {
    payload[3 + index] = frame.data[index];
  }

  return encode(payload, payloadLength, out, outCapacity, outLength);
}

uint8_t R70Frame::computeR70TcpXorCrc(const uint8_t* bytes, uint8_t length) {
  uint8_t value = 0;
  for (uint8_t index = 0; index < length; ++index) {
    value = static_cast<uint8_t>(value ^ bytes[index]);
  }
  return value;
}

}  // namespace r70v2
}  // namespace dbcan
