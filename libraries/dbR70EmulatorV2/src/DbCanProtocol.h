#ifndef DBCAN_PROTOCOL_H
#define DBCAN_PROTOCOL_H

#include "DbCanTypes.h"

namespace dbcan {
namespace protocol {

static const uint8_t kObjectClassPrimaryMaster = 0;
static const uint8_t kObjectClassProcessDataFromDevice = 2;

static const uint8_t kAdminReadUnstructured = 0x01;
static const uint8_t kAdminWriteUnstructured = 0x02;
static const uint8_t kAdminReadSegmented = 0x11;
static const uint8_t kAdminWriteSegmented = 0x12;
static const uint8_t kAdminReadFile = 0x21;
static const uint8_t kAdminWriteFile = 0x22;
static const uint8_t kAdminReadFileSegmented = 0x31;
static const uint8_t kAdminWriteFileSegmented = 0x32;
static const uint8_t kAdminPresetCommand = 0x60;

static const uint16_t kR70UdpDiscoveryPort = 33333;
static const uint16_t kR70TcpSessionPort = 30000;
static const uint8_t kR70StartByte = 0x43;
static const uint8_t kR70EndByte = 0x0D;
static const uint8_t kR70PayloadPrefix0 = 0x00;
static const uint8_t kR70PayloadPrefix1 = 0x02;
static const uint8_t kMaxR70PayloadLength = 16;
static const uint8_t kMaxR70FrameLength = kMaxR70PayloadLength + 4;

struct ParsedFrame {
  uint8_t admin;
  ObjectId objectId;
  Channel channel;
  uint8_t record;
  uint8_t value[4];
  uint8_t valueLength;
  bool hasRecord;
};

uint16_t makeCanId(uint8_t objectClass, const Address& address);
uint16_t requestId(const Address& address);
uint16_t replyId(const Address& address);
uint8_t node(const Address& address);
Address addressFromNode(uint8_t node);
bool isReplyFor(const Frame& request, const Frame& reply, const Address& address);

uint8_t makeData1(const ObjectId& objectId, Channel channel);
Channel channelFromData1(uint8_t data1);
ObjectId objectIdFromData1(uint8_t data1, uint8_t data2);

Frame makeReadUnstructured(const Address& address,
                           const ObjectId& objectId,
                           Channel channel);
Frame makeWriteUnstructured(const Address& address,
                            const ObjectId& objectId,
                            Channel channel,
                            const uint8_t* value,
                            uint8_t valueLength);
Frame makeReadSegmented(const Address& address,
                        const ObjectId& objectId,
                        Channel channel);
Frame makeWriteSegmented(const Address& address,
                         const ObjectId& objectId,
                         Channel channel,
                         uint8_t segment,
                         const uint8_t* value,
                         uint8_t valueLength);
Frame makeReadFile(const Address& address,
                   const ObjectId& objectId,
                   Channel channel,
                   uint8_t record);
Frame makeWriteFile(const Address& address,
                    const ObjectId& objectId,
                    Channel channel,
                    uint8_t record,
                    const uint8_t* value,
                    uint8_t valueLength);
Frame makePresetLoad(const Address& address, uint8_t preset);
Frame makePresetSave(const Address& address, uint8_t preset);
Frame makePresetBackupAndLoad(const Address& address, uint8_t loadPreset, uint8_t backupPreset);
Frame makePresetActivePoll(const Address& address, uint8_t preset);
bool isPresetActiveReply(const Frame& request, const Frame& reply, bool& active);
ResultCode parseFrame(const Frame& frame, ParsedFrame& parsed);

uint8_t xorChecksum(const uint8_t* bytes, uint8_t length);
uint8_t r70Checksum(const uint8_t* bytes, uint8_t length);
ResultCode encodeR70Frame(const Frame& frame,
                          const Address& address,
                          uint8_t* out,
                          uint8_t outCapacity,
                          uint8_t& outLength);
ResultCode decodeR70Frame(const uint8_t* bytes,
                          uint8_t length,
                          Frame& frame,
                          Address& address);

}  // namespace protocol
}  // namespace dbcan

#endif  // DBCAN_PROTOCOL_H
