#include "CanObjectCodec.h"

#include "R70PDFlane.h"

namespace dbcan {
namespace r70v2 {
namespace codec {

namespace {

uint8_t headerLength(const AdminParts& admin) {
  switch (admin.telegramType) {
    case TelegramType::Unstructured:
      return 3;
    case TelegramType::Segmented:
      return 4;
    case TelegramType::File:
      return 4;
    case TelegramType::FileSegmented:
      return 6;
    default:
      return 3;
  }
}

}  // namespace

bool decodeCanId(uint16_t canId, CanIdParts& parts) {
  parts = {};
  if (canId > 0x7FF) {
    return false;
  }

  parts.valid = true;
  parts.objectClass = static_cast<uint8_t>((canId >> 9) & 0x03);
  parts.subnet = static_cast<uint8_t>((canId >> 6) & 0x07);
  parts.device = static_cast<uint8_t>(canId & 0x3F);
  return true;
}

bool decodeAdmin(uint8_t adminByte, AdminParts& parts) {
  parts = {};
  const uint8_t functionBits = static_cast<uint8_t>(adminByte & 0x0F);
  if (functionBits > static_cast<uint8_t>(FunctionCode::WriteObject)) {
    return false;
  }

  parts.valid = true;
  parts.errorBit = (adminByte & 0x80) != 0;
  parts.telegramType = static_cast<TelegramType>((adminByte >> 4) & 0x03);
  parts.functionCode = static_cast<FunctionCode>(functionBits);
  return true;
}

void decodeObjectAddress(const Frame& frame, const AdminParts& admin, ObjectAddress& objectAddress) {
  objectAddress = {};
  if (frame.length < 3) {
    return;
  }

  objectAddress.channelNumber = static_cast<uint8_t>((frame.data[1] >> 5) & 0x07);
  objectAddress.boxNumber = static_cast<uint8_t>(frame.data[1] & 0x1F);
  objectAddress.objectNumber = frame.data[2];

  switch (admin.telegramType) {
    case TelegramType::Segmented:
      if (frame.length >= 4) {
        objectAddress.segmentNumber = frame.data[3];
        objectAddress.hasSegment = true;
      }
      break;
    case TelegramType::File:
      if (frame.length >= 4) {
        objectAddress.recordNumber = frame.data[3];
        objectAddress.hasRecord = true;
      }
      break;
    case TelegramType::FileSegmented:
      if (frame.length >= 6) {
        objectAddress.recordNumber =
            static_cast<uint16_t>((static_cast<uint16_t>(frame.data[3]) << 8) | frame.data[4]);
        objectAddress.segmentNumber = frame.data[5];
        objectAddress.hasRecord = true;
        objectAddress.hasSegment = true;
      }
      break;
    case TelegramType::Unstructured:
    default:
      break;
  }
}

bool normalizeCanFrame(const Frame& frame, RequestOrigin origin, NormalizedRequest& request) {
  request = {};
  if (frame.length == 0 || frame.length > kMaxFrameDataLength) {
    return false;
  }

  request.frame = frame;
  request.origin = origin;
  request.tcpWrappedCan = false;

  if (!decodeCanId(frame.canId, request.canId)) {
    return false;
  }

  if (frame.length > 0 && decodeAdmin(frame.data[0], request.admin)) {
    if (frame.length >= 3) {
      decodeObjectAddress(frame, request.admin, request.objectAddress);
      request.mapping = pdflane::findMapping(request.objectAddress);
      request.fieldId = request.mapping != 0 ? request.mapping->fieldId : FieldId::Unknown;
    }
  }
  request.valid = true;
  return true;
}

bool normalizeTcpWrappedCanEnvelope(const TcpEnvelope& envelope, NormalizedRequest& request) {
  request = {};
  if (!envelope.valid || envelope.payloadLength < 3 ||
      envelope.payload[0] != R70Frame::kPayloadPrefix0) {
    return false;
  }

  const uint16_t canId =
      static_cast<uint16_t>(((static_cast<uint16_t>(envelope.payload[1]) << 8) |
                             envelope.payload[2]) & 0x7FFu);
  const uint8_t canLength = static_cast<uint8_t>(envelope.payloadLength - 3);
  if (canLength > kMaxFrameDataLength) {
    return false;
  }

  Frame frame = {};
  frame.canId = canId;
  frame.length = canLength;
  for (uint8_t index = 0; index < canLength; ++index) {
    frame.data[index] = envelope.payload[3 + index];
  }

  if (!normalizeCanFrame(frame, RequestOrigin::Tcp, request)) {
    return false;
  }

  request.tcpWrappedCan = true;
  request.receivedChecksum = envelope.checksumByte;
  request.checksumChecked = false;
  request.checksumValid = envelope.checksumValid;
  return true;
}

bool normalizeTcpEnvelope(const TcpEnvelope& envelope, NormalizedRequest& request) {
  return normalizeTcpWrappedCanEnvelope(envelope, request);
}

bool encodeTcpReply(const NormalizedRequest& request,
                    uint8_t replyNode,
                    uint8_t* out,
                    size_t outCapacity,
                    uint8_t& outLength) {
  outLength = 0;
  if (!request.valid || out == 0) {
    return false;
  }

  uint8_t payload[16] = {};
  const uint8_t canLength = request.frame.length;
  if (canLength + 3 > sizeof(payload)) {
    return false;
  }

  payload[0] = R70Frame::kPayloadPrefix0;
  payload[1] = R70Frame::kReplyType;
  payload[2] = replyNode;
  for (uint8_t index = 0; index < canLength; ++index) {
    payload[3 + index] = request.frame.data[index];
  }
  return R70Frame::encode(payload, static_cast<uint8_t>(canLength + 3), out, outCapacity,
                          outLength);
}

bool sameLogicalShape(const NormalizedRequest& left, const NormalizedRequest& right) {
  if (!left.valid || !right.valid) {
    return false;
  }
  if (left.canId.subnet != right.canId.subnet || left.canId.device != right.canId.device ||
      left.admin.functionCode != right.admin.functionCode ||
      left.admin.telegramType != right.admin.telegramType ||
      left.objectAddress.channelNumber != right.objectAddress.channelNumber ||
      left.objectAddress.boxNumber != right.objectAddress.boxNumber ||
      left.objectAddress.objectNumber != right.objectAddress.objectNumber ||
      left.objectAddress.recordNumber != right.objectAddress.recordNumber ||
      left.objectAddress.segmentNumber != right.objectAddress.segmentNumber) {
    return false;
  }

  const uint8_t leftHeader = headerLength(left.admin);
  const uint8_t rightHeader = headerLength(right.admin);
  if (leftHeader != rightHeader || left.frame.length != right.frame.length) {
    return false;
  }

  for (uint8_t index = leftHeader; index < left.frame.length; ++index) {
    if (left.frame.data[index] != right.frame.data[index]) {
      return false;
    }
  }
  return true;
}

}  // namespace codec
}  // namespace r70v2
}  // namespace dbcan
