#include "DbCanProtocol.h"

namespace dbcan {
namespace protocol {

namespace {

static const uint8_t kChannelOffsetA = 0x20;
static const uint8_t kChannelOffsetB = 0x40;
static const uint8_t kChannelMask = 0x60;
static const uint8_t kObjectBoxMask = 0x1F;

Frame makeEmptyFrame(const Address& address) {
  Frame frame = {};
  frame.canId = requestId(address);
  return frame;
}

uint8_t lowByte(uint16_t value) {
  return static_cast<uint8_t>(value & 0xFF);
}

uint8_t boundedValueLength(uint8_t headerLength, uint8_t valueLength) {
  if (headerLength >= kMaxFrameDataLength) {
    return 0;
  }

  const uint8_t available = kMaxFrameDataLength - headerLength;
  return valueLength < available ? valueLength : available;
}

void appendValue(Frame& frame, const uint8_t* value, uint8_t valueLength) {
  if (value == 0) {
    return;
  }

  const uint8_t count = boundedValueLength(frame.length, valueLength);

  for (uint8_t index = 0; index < count; ++index) {
    frame.data[frame.length++] = value[index];
  }
}

}  // namespace

uint16_t makeCanId(uint8_t objectClass, const Address& address) {
  return static_cast<uint16_t>(((objectClass & 0x03) << 9) |
                               ((address.subnet & 0x07) << 6) |
                               (address.device & 0x3F));
}

uint16_t requestId(const Address& address) {
  return makeCanId(kObjectClassPrimaryMaster, address);
}

uint16_t replyId(const Address& address) {
  return makeCanId(kObjectClassProcessDataFromDevice, address);
}

uint8_t node(const Address& address) {
  return static_cast<uint8_t>(((address.subnet & 0x03) << 6) |
                              (address.device & 0x3F));
}

Address addressFromNode(uint8_t encodedNode) {
  Address address = {};
  address.subnet = static_cast<uint8_t>((encodedNode >> 6) & 0x03);
  address.device = static_cast<uint8_t>(encodedNode & 0x3F);
  return address;
}

bool isReplyFor(const Frame& request, const Frame& reply, const Address& address) {
  if (request.canId != requestId(address) || reply.canId != replyId(address)) {
    return false;
  }

  if (request.length < 3 || reply.length < 3) {
    return false;
  }

  return request.data[1] == reply.data[1] && request.data[2] == reply.data[2];
}

uint8_t makeData1(const ObjectId& objectId, Channel channel) {
  uint8_t data1 = lowByte(objectId.box);

  if (channel == Channel::A) {
    data1 = static_cast<uint8_t>(data1 | kChannelOffsetA);
  } else if (channel == Channel::B) {
    data1 = static_cast<uint8_t>(data1 | kChannelOffsetB);
  }

  return data1;
}

Channel channelFromData1(uint8_t data1) {
  const uint8_t channelBits = data1 & kChannelMask;

  if (channelBits == kChannelOffsetA) {
    return Channel::A;
  }

  if (channelBits == kChannelOffsetB) {
    return Channel::B;
  }

  if (channelBits == 0) {
    return Channel::None;
  }

  return Channel::Unknown;
}

ObjectId objectIdFromData1(uint8_t data1, uint8_t data2) {
  ObjectId objectId = {};
  objectId.box = static_cast<uint16_t>(data1 & kObjectBoxMask);
  objectId.object = data2;
  return objectId;
}

Frame makeReadUnstructured(const Address& address,
                           const ObjectId& objectId,
                           Channel channel) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 3;
  frame.data[0] = kAdminReadUnstructured;
  frame.data[1] = makeData1(objectId, channel);
  frame.data[2] = lowByte(objectId.object);
  return frame;
}

Frame makeWriteUnstructured(const Address& address,
                            const ObjectId& objectId,
                            Channel channel,
                            const uint8_t* value,
                            uint8_t valueLength) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 3;
  frame.data[0] = kAdminWriteUnstructured;
  frame.data[1] = makeData1(objectId, channel);
  frame.data[2] = lowByte(objectId.object);
  appendValue(frame, value, valueLength);
  return frame;
}

Frame makeReadSegmented(const Address& address,
                        const ObjectId& objectId,
                        Channel channel) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 3;
  frame.data[0] = kAdminReadSegmented;
  frame.data[1] = makeData1(objectId, channel);
  frame.data[2] = lowByte(objectId.object);
  return frame;
}

Frame makeWriteSegmented(const Address& address,
                         const ObjectId& objectId,
                         Channel channel,
                         uint8_t segment,
                         const uint8_t* value,
                         uint8_t valueLength) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 4;
  frame.data[0] = kAdminWriteSegmented;
  frame.data[1] = makeData1(objectId, channel);
  frame.data[2] = lowByte(objectId.object);
  frame.data[3] = segment;
  appendValue(frame, value, valueLength);
  return frame;
}

Frame makeReadFile(const Address& address,
                   const ObjectId& objectId,
                   Channel channel,
                   uint8_t record) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 4;
  frame.data[0] = kAdminReadFile;
  frame.data[1] = makeData1(objectId, channel);
  frame.data[2] = lowByte(objectId.object);
  frame.data[3] = record;
  return frame;
}

Frame makeWriteFile(const Address& address,
                    const ObjectId& objectId,
                    Channel channel,
                    uint8_t record,
                    const uint8_t* value,
                    uint8_t valueLength) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 4;
  frame.data[0] = kAdminWriteFile;
  frame.data[1] = makeData1(objectId, channel);
  frame.data[2] = lowByte(objectId.object);
  frame.data[3] = record;
  appendValue(frame, value, valueLength);
  return frame;
}

Frame makePresetLoad(const Address& address, uint8_t preset) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 4;
  frame.data[0] = kAdminPresetCommand;
  frame.data[1] = 0x02;
  frame.data[2] = 0x01;
  frame.data[3] = preset;
  return frame;
}

Frame makePresetSave(const Address& address, uint8_t preset) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 4;
  frame.data[0] = kAdminPresetCommand;
  frame.data[1] = 0x02;
  frame.data[2] = 0x02;
  frame.data[3] = preset;
  return frame;
}

Frame makePresetBackupAndLoad(const Address& address, uint8_t loadPreset, uint8_t backupPreset) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 5;
  frame.data[0] = kAdminPresetCommand;
  frame.data[1] = 0x02;
  frame.data[2] = 0x03;
  frame.data[3] = loadPreset;
  frame.data[4] = backupPreset;
  return frame;
}

Frame makePresetActivePoll(const Address& address, uint8_t preset) {
  Frame frame = makeEmptyFrame(address);
  frame.length = 4;
  frame.data[0] = kAdminReadFile;
  frame.data[1] = 0x10;
  frame.data[2] = 0x01;
  frame.data[3] = preset;
  return frame;
}

bool isPresetActiveReply(const Frame& request, const Frame& reply, bool& active) {
  if (request.length != 4 || reply.length < 5) {
    return false;
  }

  if (request.data[0] != kAdminReadFile ||
      request.data[1] != 0x10 ||
      request.data[2] != 0x01) {
    return false;
  }

  if (reply.data[0] != request.data[0] ||
      reply.data[1] != request.data[1] ||
      reply.data[2] != request.data[2] ||
      reply.data[3] != request.data[3]) {
    return false;
  }

  if (reply.data[4] > 1) {
    return false;
  }

  active = reply.data[4] == 1;
  return true;
}

ResultCode parseFrame(const Frame& frame, ParsedFrame& parsed) {
  if (frame.canId > kMaxStandardCanId || frame.length > kMaxFrameDataLength) {
    return ResultCode::InvalidValue;
  }

  if (frame.length < 3) {
    return ResultCode::InvalidValue;
  }

  parsed = {};
  parsed.admin = frame.data[0];
  parsed.objectId = objectIdFromData1(frame.data[1], frame.data[2]);
  parsed.channel = channelFromData1(frame.data[1]);

  uint8_t valueStart = 3;
  parsed.hasRecord = false;

  if (parsed.admin == kAdminReadFile || parsed.admin == kAdminWriteFile ||
      parsed.admin == kAdminReadFileSegmented ||
      parsed.admin == kAdminWriteFileSegmented ||
      parsed.admin == kAdminWriteSegmented) {
    if (frame.length < 4) {
      return ResultCode::InvalidValue;
    }
    parsed.record = frame.data[3];
    parsed.hasRecord = true;
    valueStart = 4;
  }

  for (uint8_t index = valueStart; index < frame.length && parsed.valueLength < sizeof(parsed.value);
       ++index) {
    parsed.value[parsed.valueLength++] = frame.data[index];
  }

  return ResultCode::Ok;
}

uint8_t xorChecksum(const uint8_t* bytes, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t index = 0; index < length; ++index) {
    checksum = static_cast<uint8_t>(checksum ^ bytes[index]);
  }
  return checksum;
}

uint8_t r70Checksum(const uint8_t* bytes, uint8_t length) {
  return xorChecksum(bytes, length);
}

ResultCode encodeR70Frame(const Frame& frame,
                          const Address& address,
                          uint8_t* out,
                          uint8_t outCapacity,
                          uint8_t& outLength) {
  outLength = 0;

  if (out == 0 || frame.length > kMaxFrameDataLength) {
    return ResultCode::InvalidValue;
  }

  const uint8_t payloadLength = static_cast<uint8_t>(frame.length + 3);
  const uint8_t totalLength = static_cast<uint8_t>(payloadLength + 4);
  if (payloadLength > kMaxR70PayloadLength || outCapacity < totalLength) {
    return ResultCode::InvalidValue;
  }

  out[0] = kR70StartByte;
  out[1] = payloadLength;
  out[2] = kR70PayloadPrefix0;
  out[3] = kR70PayloadPrefix1;
  out[4] = node(address);

  for (uint8_t index = 0; index < frame.length; ++index) {
    out[5 + index] = frame.data[index];
  }

  out[totalLength - 2] = r70Checksum(out, static_cast<uint8_t>(totalLength - 2));
  out[totalLength - 1] = kR70EndByte;
  outLength = totalLength;
  return ResultCode::Ok;
}

ResultCode decodeR70Frame(const uint8_t* bytes,
                          uint8_t length,
                          Frame& frame,
                          Address& address) {
  if (bytes == 0 || length < 7 || bytes[0] != kR70StartByte ||
      bytes[length - 1] != kR70EndByte) {
    return ResultCode::InvalidValue;
  }

  const uint8_t payloadLength = bytes[1];
  if (payloadLength + 4 != length || payloadLength < 3 ||
      payloadLength > kMaxR70PayloadLength) {
    return ResultCode::InvalidValue;
  }

  if (r70Checksum(bytes, static_cast<uint8_t>(length - 2)) != bytes[length - 2]) {
    return ResultCode::Error;
  }

  if (bytes[2] != kR70PayloadPrefix0 || bytes[3] != kR70PayloadPrefix1) {
    return ResultCode::NotSupported;
  }

  address = addressFromNode(bytes[4]);

  frame = {};
  frame.canId = requestId(address);
  frame.length = static_cast<uint8_t>(payloadLength - 3);
  if (frame.length > kMaxFrameDataLength) {
    return ResultCode::InvalidValue;
  }

  for (uint8_t index = 0; index < frame.length; ++index) {
    frame.data[index] = bytes[5 + index];
  }

  return ResultCode::Ok;
}

}  // namespace protocol
}  // namespace dbcan
