#include "DbR70Transport.h"

namespace dbcan {

namespace {

bool timedOut(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

}  // namespace

R70Transport::R70Transport(const Address& address) : address_(address) {}

R70Transport::~R70Transport() {
  end();
}

bool R70Transport::beginDiscovery(uint16_t udpPort) {
  if (udpStarted_) {
    return true;
  }

  udpStarted_ = udp_.begin(udpPort) == 1;
  return udpStarted_;
}

ResultCode R70Transport::sendDiscoveryProbe() {
  if (!udpStarted_ && !beginDiscovery()) {
    return ResultCode::Error;
  }
  return sendR70DiscoveryProbe(udp_);
}

bool R70Transport::waitForDiscovery(IPAddress& remoteIp, uint32_t timeoutMs) {
  if (!udpStarted_ && !beginDiscovery()) {
    return false;
  }

  ResultCode result = sendDiscoveryProbe();
  if (result != ResultCode::Ok) {
    return false;
  }

  R70DiscoveryReply reply = {};
  R70DiscoveryEndpoint endpoint = {};
  result = readR70DiscoveryReply(udp_, reply, timeoutMs, &endpoint);
  if (result != ResultCode::Ok || reply.entryCount == 0) {
    return false;
  }

  remoteIp = reply.entries[0].ip;
  return true;
}

bool R70Transport::connect(const IPAddress& remoteIp, uint16_t tcpPort) {
  client_.stop();
  return client_.connect(remoteIp, tcpPort) == 1;
}

void R70Transport::end() {
  client_.stop();
  if (udpStarted_) {
    udp_.stop();
    udpStarted_ = false;
  }
}

bool R70Transport::isStarted() const {
  return const_cast<EthernetClient&>(client_).connected();
}

ResultCode R70Transport::sendFrame(const Frame& frame, uint32_t timeoutMs) {
  if (!client_.connected()) {
    return ResultCode::NotSupported;
  }

  uint8_t bytes[protocol::kMaxR70FrameLength] = {};
  uint8_t length = 0;
  ResultCode result = protocol::encodeR70Frame(frame, address_, bytes, sizeof(bytes), length);
  if (result != ResultCode::Ok) {
    return result;
  }

  client_.setTimeout(timeoutMs);
  const size_t written = client_.write(bytes, length);
  if (written != length) {
    return ResultCode::Timeout;
  }

  return ResultCode::Ok;
}

ResultCode R70Transport::receiveFrame(Frame& frame, uint32_t timeoutMs) {
  if (!client_.connected()) {
    return ResultCode::NotSupported;
  }

  const uint32_t deadline = millis() + timeoutMs;
  uint8_t start = 0;
  ResultCode result = ResultCode::Timeout;

  while (!timedOut(millis(), deadline)) {
    result = readByte(start, deadline);
    if (result != ResultCode::Ok) {
      return result;
    }
    if (start == protocol::kR70StartByte) {
      break;
    }
  }

  if (start != protocol::kR70StartByte) {
    return ResultCode::Timeout;
  }

  uint8_t lengthByte = 0;
  result = readByte(lengthByte, deadline);
  if (result != ResultCode::Ok) {
    return result;
  }

  const uint8_t totalLength = static_cast<uint8_t>(lengthByte + 4);
  if (totalLength > protocol::kMaxR70FrameLength) {
    return ResultCode::InvalidValue;
  }

  uint8_t bytes[protocol::kMaxR70FrameLength] = {};
  bytes[0] = start;
  bytes[1] = lengthByte;

  for (uint8_t index = 2; index < totalLength; ++index) {
    result = readByte(bytes[index], deadline);
    if (result != ResultCode::Ok) {
      return result;
    }
  }

  Address decodedAddress = {};
  return protocol::decodeR70Frame(bytes, totalLength, frame, decodedAddress);
}

bool R70Transport::sendRawPayload(const uint8_t* payload, uint8_t length, uint32_t timeoutMs) {
  if (payload == 0 && length > 0) {
    return false;
  }

  Frame frame = {};
  frame.canId = protocol::requestId(address_);
  frame.length = length;
  if (frame.length > kMaxFrameDataLength) {
    return false;
  }

  for (uint8_t index = 0; index < frame.length; ++index) {
    frame.data[index] = payload[index];
  }

  return sendFrame(frame, timeoutMs) == ResultCode::Ok;
}

ResultCode R70Transport::isAmpPresetActive(uint8_t encodedNode,
                                           uint8_t presetNo,
                                           bool& active,
                                           uint32_t timeoutMs) {
  active = false;
  const Address requestAddress = protocol::addressFromNode(encodedNode);
  Frame request = protocol::makePresetActivePoll(requestAddress, presetNo);

  ResultCode result = sendFrame(request, timeoutMs);
  if (result != ResultCode::Ok) {
    return result;
  }

  Frame reply = {};
  result = receiveFrame(reply, timeoutMs);
  if (result != ResultCode::Ok) {
    return result;
  }

  if (!protocol::isPresetActiveReply(request, reply, active)) {
    return ResultCode::Error;
  }

  return ResultCode::Ok;
}

ResultCode R70Transport::readByte(uint8_t& value, uint32_t deadlineMs) {
  while (!timedOut(millis(), deadlineMs)) {
    if (!client_.connected()) {
      return ResultCode::Error;
    }
    if (client_.available() > 0) {
      const int next = client_.read();
      if (next < 0) {
        return ResultCode::Error;
      }
      value = static_cast<uint8_t>(next);
      return ResultCode::Ok;
    }
    delay(1);
  }

  return ResultCode::Timeout;
}

}  // namespace dbcan
