#ifndef DB_AMP_H
#define DB_AMP_H

#include "DbCanBackend.h"
#include "DbCanProtocol.h"
#include "DbCanRegistry.h"

namespace dbcan {

class Amp {
 public:
  Amp(FrameBackend& backend, const Address& address);

  const Address& address() const;
  uint16_t requestCanId() const;
  uint16_t replyCanId() const;

  ResultCode powerOn();
  ResultCode powerOff();
  ResultCode setMute(Channel channel, bool muted);
  ResultCode setDelayEnable(Channel channel, bool enabled);
  ResultCode setRoutingA(uint8_t routing);

  // Raw signed value path for object 0206. Scaling stays outside this API until fully verified.
  ResultCode setGainRaw(Channel channel, int16_t rawValue);
  ResultCode loadPreset(uint8_t preset);
  ResultCode savePreset(uint8_t preset);
  ResultCode backupAndLoadPreset(uint8_t loadPreset, uint8_t backupPreset);
  ResultCode isAmpPresetActive(uint8_t presetNo, bool& active, uint32_t timeoutMs = 100);

  ResultCode readObject(const ObjectDescriptor& descriptor);
  ResultCode writeObject(const ObjectDescriptor& descriptor,
                         const uint8_t* value,
                         uint8_t valueLength);
  ResultCode readFileObject(const ObjectDescriptor& descriptor, uint8_t record);
  ResultCode writeFileObject(const ObjectDescriptor& descriptor,
                             uint8_t record,
                             const uint8_t* value,
                             uint8_t valueLength);
  ResultCode sendFrame(const Frame& frame);

 private:
  ResultCode writeBool(const ObjectDescriptor& descriptor, bool value);
  ResultCode writeInt16(const ObjectDescriptor& descriptor, int16_t value);

  FrameBackend& backend_;
  Address address_;
};

}  // namespace dbcan

#endif  // DB_AMP_H
