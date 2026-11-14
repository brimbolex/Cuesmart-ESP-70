#include "DbAmp.h"

namespace dbcan {

namespace {

const ObjectDescriptor* muteDescriptor(Channel channel) {
  if (channel == Channel::A) {
    return &registry::config_mute_a;
  }
  if (channel == Channel::B) {
    return &registry::config_mute_b;
  }
  return 0;
}

const ObjectDescriptor* delayDescriptor(Channel channel) {
  if (channel == Channel::A) {
    return &registry::config_delay_on_a;
  }
  if (channel == Channel::B) {
    return &registry::config_delay_on_b;
  }
  return 0;
}

const ObjectDescriptor* gainDescriptor(Channel channel) {
  if (channel == Channel::A) {
    return &registry::config_gain_a;
  }
  if (channel == Channel::B) {
    return &registry::config_gain_b;
  }
  return 0;
}

}  // namespace

Amp::Amp(FrameBackend& backend, const Address& address)
    : backend_(backend), address_(address) {}

const Address& Amp::address() const {
  return address_;
}

uint16_t Amp::requestCanId() const {
  return protocol::requestId(address_);
}

uint16_t Amp::replyCanId() const {
  return protocol::replyId(address_);
}

ResultCode Amp::powerOn() {
  return writeBool(registry::settings_power, true);
}

ResultCode Amp::powerOff() {
  return writeBool(registry::settings_power, false);
}

ResultCode Amp::setMute(Channel channel, bool muted) {
  const ObjectDescriptor* descriptor = muteDescriptor(channel);
  if (descriptor == 0) {
    return ResultCode::InvalidValue;
  }
  return writeBool(*descriptor, muted);
}

ResultCode Amp::setDelayEnable(Channel channel, bool enabled) {
  const ObjectDescriptor* descriptor = delayDescriptor(channel);
  if (descriptor == 0) {
    return ResultCode::InvalidValue;
  }
  return writeBool(*descriptor, enabled);
}

ResultCode Amp::setRoutingA(uint8_t routing) {
  if (routing > 2) {
    return ResultCode::InvalidValue;
  }

  return writeObject(registry::config_routing_a, &routing, 1);
}

ResultCode Amp::setGainRaw(Channel channel, int16_t rawValue) {
  const ObjectDescriptor* descriptor = gainDescriptor(channel);
  if (descriptor == 0) {
    return ResultCode::InvalidValue;
  }
  return writeInt16(*descriptor, rawValue);
}

ResultCode Amp::loadPreset(uint8_t preset) {
  return sendFrame(protocol::makePresetLoad(address_, preset));
}

ResultCode Amp::savePreset(uint8_t preset) {
  return sendFrame(protocol::makePresetSave(address_, preset));
}

ResultCode Amp::backupAndLoadPreset(uint8_t loadPreset, uint8_t backupPreset) {
  return sendFrame(protocol::makePresetBackupAndLoad(address_, loadPreset, backupPreset));
}

ResultCode Amp::isAmpPresetActive(uint8_t presetNo, bool& active, uint32_t timeoutMs) {
  active = false;

  Frame request = protocol::makePresetActivePoll(address_, presetNo);
  ResultCode result = sendFrame(request);
  if (result != ResultCode::Ok) {
    return result;
  }

  Frame reply = {};
  result = backend_.receiveFrame(reply, timeoutMs);
  if (result != ResultCode::Ok) {
    return result;
  }

  if (!protocol::isPresetActiveReply(request, reply, active)) {
    return ResultCode::Error;
  }

  return ResultCode::Ok;
}

ResultCode Amp::readObject(const ObjectDescriptor& descriptor) {
  if (descriptor.confidence == Confidence::Placeholder) {
    return ResultCode::PlaceholderOnly;
  }

  if (descriptor.access == Access::File || descriptor.objectClass == ObjectClass::Eq) {
    return ResultCode::NotSupported;
  }

  if (descriptor.access != Access::Read && descriptor.access != Access::ReadWrite) {
    return ResultCode::NotSupported;
  }

  Frame frame = protocol::makeReadUnstructured(address_,
                                               descriptor.id,
                                               descriptor.channel);
  return sendFrame(frame);
}

ResultCode Amp::writeObject(const ObjectDescriptor& descriptor,
                            const uint8_t* value,
                            uint8_t valueLength) {
  if (descriptor.confidence == Confidence::Placeholder) {
    return ResultCode::PlaceholderOnly;
  }

  if (descriptor.access == Access::File || descriptor.objectClass == ObjectClass::Eq) {
    return ResultCode::NotSupported;
  }

  if (descriptor.access != Access::Write && descriptor.access != Access::ReadWrite) {
    return ResultCode::NotSupported;
  }

  Frame frame = protocol::makeWriteUnstructured(address_,
                                                descriptor.id,
                                                descriptor.channel,
                                                value,
                                                valueLength);
  return sendFrame(frame);
}

ResultCode Amp::readFileObject(const ObjectDescriptor& descriptor, uint8_t record) {
  if (descriptor.confidence == Confidence::Placeholder) {
    return ResultCode::PlaceholderOnly;
  }

  if (descriptor.access != Access::File) {
    return ResultCode::NotSupported;
  }

  Frame frame = protocol::makeReadFile(address_,
                                       descriptor.id,
                                       descriptor.channel,
                                       record);
  return sendFrame(frame);
}

ResultCode Amp::writeFileObject(const ObjectDescriptor& descriptor,
                                uint8_t record,
                                const uint8_t* value,
                                uint8_t valueLength) {
  if (descriptor.confidence == Confidence::Placeholder) {
    return ResultCode::PlaceholderOnly;
  }

  if (descriptor.access != Access::File) {
    return ResultCode::NotSupported;
  }

  Frame frame = protocol::makeWriteFile(address_,
                                        descriptor.id,
                                        descriptor.channel,
                                        record,
                                        value,
                                        valueLength);
  return sendFrame(frame);
}

ResultCode Amp::sendFrame(const Frame& frame) {
  return backend_.sendFrame(frame);
}

ResultCode Amp::writeBool(const ObjectDescriptor& descriptor, bool value) {
  const uint8_t byteValue = value ? 1 : 0;
  return writeObject(descriptor, &byteValue, 1);
}

ResultCode Amp::writeInt16(const ObjectDescriptor& descriptor, int16_t value) {
  const uint8_t bytes[2] = {
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>(value & 0xFF),
  };
  return writeObject(descriptor, bytes, sizeof(bytes));
}

}  // namespace dbcan
