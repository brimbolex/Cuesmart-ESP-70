#include "DebugCli.h"

namespace dbcan {
namespace r70v2 {

namespace {

void printTimestampPrefix(Print& out, uint32_t nowMs) {
  out.print("T=");
  out.print(nowMs);
  out.print("ms ");
}

void printHexByte(Print& out, uint8_t value) {
  if (value < 0x10) {
    out.print('0');
  }
  out.print(value, HEX);
}

void printHexWord(Print& out, uint16_t value) {
  printHexByte(out, static_cast<uint8_t>((value >> 8) & 0xFFu));
  printHexByte(out, static_cast<uint8_t>(value & 0xFFu));
}

const char* wrappedCanLabel(const NormalizedRequest& request) {
  if (request.frame.length == 2 &&
      request.frame.data[0] == 0x00 &&
      request.frame.data[1] == 0x01 &&
      request.canId.device == 0) {
    return "IdentBroadcastRandomDelay";
  }
  return "TcpWrappedCan";
}

const char* canRxLabel(const NormalizedRequest& request) {
  if (request.canId.objectClass == 2 &&
      request.frame.length > 0 &&
      request.frame.data[0] == 0x00) {
    return "IdentResponse";
  }
  return "CanRxForwarded";
}

}  // namespace

DebugCli::DebugCli() : out_(0), verboseFrameLogEnabled_(false) {}

void DebugCli::begin(Print& out) {
  out_ = &out;
}

void DebugCli::setVerboseFrameLogEnabled(bool enabled) {
  verboseFrameLogEnabled_ = enabled;
}

void DebugCli::logText(const char* text) const {
  if (out_ == 0 || text == 0) {
    return;
  }
  out_->println(text);
}

void DebugCli::logHex(const char* prefix, const uint8_t* bytes, size_t length) const {
  if (out_ == 0 || bytes == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  if (prefix != 0) {
    out_->print(prefix);
  }
  for (size_t index = 0; index < length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, bytes[index]);
  }
  out_->println();
}

void DebugCli::logTcpFrame(const char* prefix,
                          const uint8_t* bytes,
                          uint8_t length,
                          bool hasCrcStatus,
                          bool crcOk) const {
  if (out_ == 0 || bytes == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  logHex(prefix, bytes, length);
  if (hasCrcStatus) {
    out_->print("TCP_RX_CRC crc_ok=");
    out_->println(crcOk ? "true" : "false");
  }
}

void DebugCli::logCanFrame(const char* prefix, const Frame& frame, const NormalizedRequest* request) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  if (prefix != 0) {
    out_->print(prefix);
  }
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(frame.canId, HEX);
  out_->print(" len=");
  out_->print(frame.length);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  if (request != 0 && request->valid) {
    out_->print(" subnet=");
    out_->print(request->canId.subnet);
    out_->print(" device=");
    out_->print(request->canId.device);
    out_->print(" field=");
    if (request->canId.objectClass == 2 && request->frame.length > 0 && request->frame.data[0] == 0x00) {
      out_->print("IdentResponse");
    } else {
      out_->print(fieldIdName(request->fieldId));
    }
  }
  out_->println();
}

void DebugCli::logTcpCanTx(const NormalizedRequest& request) const {
  if (out_ == 0 || !request.valid || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_CAN_TX id=0x");
  if (request.frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(request.frame.canId, HEX);
  out_->print(" objClass=");
  out_->print(request.canId.objectClass);
  out_->print(" subnet=");
  out_->print(request.canId.subnet);
  out_->print(" device=");
  out_->print(request.canId.device);
  out_->print(" dlc=");
  out_->print(request.frame.length);
  out_->print(" data=");
  for (uint8_t index = 0; index < request.frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, request.frame.data[index]);
  }
  out_->print(" label=");
  out_->print(wrappedCanLabel(request));
  out_->print(" checksum=0x");
  printHexByte(*out_, request.receivedChecksum);
  out_->print(" checksumMode=xor_sof_len_cmd_id_data");
  out_->print(" crc_ok=");
  out_->print(request.checksumValid ? "true" : "false");
  out_->print(" rxChecksum=0x");
  printHexByte(*out_, request.receivedChecksum);
  out_->println();
}

void DebugCli::logTcpCanRxForward(const NormalizedRequest& request) const {
  if (out_ == 0 || !request.valid || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_CAN_RX_FORWARD id=0x");
  if (request.frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(request.frame.canId, HEX);
  out_->print(" objClass=");
  out_->print(request.canId.objectClass);
  out_->print(" subnet=");
  out_->print(request.canId.subnet);
  out_->print(" device=");
  out_->print(request.canId.device);
  out_->print(" dlc=");
  out_->print(request.frame.length);
  out_->print(" data=");
  for (uint8_t index = 0; index < request.frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, request.frame.data[index]);
  }
  out_->print(" label=");
  out_->println(canRxLabel(request));
}

void DebugCli::logTcpTxWrappedCan(const Frame& frame, const uint8_t* bytes, uint8_t length) const {
  if (out_ == 0 || bytes == 0 || length < 2 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_TX ");
  for (uint8_t index = 0; index < length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, bytes[index]);
  }
  out_->print(" checksum=0x");
  printHexByte(*out_, bytes[length - 2]);
  out_->print(" checksumMode=xor_sof_len_cmd_id_data can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->println(frame.canId, HEX);
}

void DebugCli::logChecksumMismatch(const TcpEnvelope& envelope) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_CAN_CHECKSUM_FAIL rxChecksum=0x");
  printHexByte(*out_, envelope.checksumByte);
  out_->println(" forwarding=allowed checksumMode=xor_sof_len_cmd_id_data");
}

void DebugCli::logTcpBackpressureOn(uint16_t queueDepth, uint16_t highWater) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_BACKPRESSURE_ON queue_depth=");
  out_->print(queueDepth);
  out_->print(" highWater=");
  out_->println(highWater);
}

void DebugCli::logTcpBackpressureOff(uint16_t queueDepth, uint16_t lowWater, uint32_t activeMs) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_BACKPRESSURE_OFF queue_depth=");
  out_->print(queueDepth);
  out_->print(" lowWater=");
  out_->print(lowWater);
  out_->print(" active_ms=");
  out_->println(activeMs);
}

void DebugCli::logTcpBackpressureLimited(const char* reason) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_BACKPRESSURE_LIMITED reason=");
  out_->println(reason != 0 ? reason : "unknown");
}

void DebugCli::logReadCoalesced(const NormalizedRequest& request, uint32_t count) const {
  if (out_ == 0 || !request.valid || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("READ_COALESCED can_id=0x");
  if (request.frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(request.frame.canId, HEX);
  out_->print(" data=");
  for (uint8_t index = 0; index < request.frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, request.frame.data[index]);
  }
  out_->print(" count=");
  out_->println(count);
}

void DebugCli::logCacheReply(const NormalizedRequest& request,
                             const Frame& frame,
                             uint32_t ageMs,
                             uint32_t ttlMs) const {
  if (out_ == 0 || !request.valid || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("CACHE_REPLY node=");
  out_->print(request.canId.subnet);
  out_->print('.');
  out_->print(request.canId.device);
  out_->print(" field=");
  out_->print(fieldIdName(request.fieldId));
  out_->print(" age_ms=");
  out_->print(ageMs);
  out_->print(" ttl_ms=");
  out_->print(ttlMs);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(frame.canId, HEX);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logNameAssembled(uint8_t subnet,
                                uint8_t device,
                                FieldId fieldId,
                                const uint8_t* bytes,
                                uint8_t length) const {
  if (out_ == 0 || bytes == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("NAME_ASSEMBLED node=");
  out_->print(subnet);
  out_->print('.');
  out_->print(device);
  out_->print(" field=");
  out_->print(fieldIdName(fieldId));
  out_->print(" value=\"");
  for (uint8_t index = 0; index < length; ++index) {
    const char ch = static_cast<char>(bytes[index]);
    out_->print((ch >= 32 && ch <= 126) ? ch : '.');
  }
  out_->println("\"");
}

void DebugCli::logNameSegmentCacheReply(uint8_t subnet,
                                        uint8_t device,
                                        FieldId fieldId,
                                        uint8_t segmentNumber,
                                        const Frame& frame,
                                        uint32_t ageMs) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("NAME_SEGMENT_CACHE_REPLY node=");
  out_->print(subnet);
  out_->print('.');
  out_->print(device);
  out_->print(" field=");
  out_->print(fieldIdName(fieldId));
  out_->print(" segment=");
  out_->print(segmentNumber);
  out_->print(" age_ms=");
  out_->print(ageMs);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(frame.canId, HEX);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logFirmwareSegmentCacheReply(uint8_t subnet,
                                            uint8_t device,
                                            uint8_t segmentNumber,
                                            const Frame& frame,
                                            uint32_t ageMs) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("FIRMWARE_SEGMENT_CACHE_REPLY node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" segment=");
  out_->print(segmentNumber);
  out_->print(" age_ms=");
  out_->print(ageMs);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logStableSetupCacheReply(uint8_t subnet,
                                        uint8_t device,
                                        FieldId fieldId,
                                        const Frame& frame,
                                        uint32_t ageMs) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("STABLE_SETUP_CACHE_REPLY node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" field=");
  out_->print(fieldIdName(fieldId));
  out_->print(" age_ms=");
  out_->print(ageMs);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logStableSetupCacheInvalidate(uint8_t subnet,
                                             uint8_t device,
                                             FieldId fieldId,
                                             const char* reason) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("STABLE_SETUP_CACHE_INVALIDATE node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" field=");
  out_->print(fieldIdName(fieldId));
  out_->print(" reason=");
  out_->println(reason != 0 ? reason : "unknown");
}

void DebugCli::logTopCanTxObject(uint8_t rank,
                                 uint16_t objectKey,
                                 const char* label,
                                 uint32_t tcpReadRequests,
                                 uint32_t canTxForwarded,
                                 uint32_t cacheReplies,
                                 uint32_t cacheStale,
                                 uint32_t cacheMiss,
                                 uint32_t coalescedDuplicates,
                                 uint32_t queueFull) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TOP_CAN_TX_OBJECTS rank=");
  out_->print(rank);
  out_->print(" object=");
  if (objectKey < 0x1000) {
    out_->print('0');
  }
  if (objectKey < 0x100) {
    out_->print('0');
  }
  if (objectKey < 0x10) {
    out_->print('0');
  }
  out_->print(objectKey, HEX);
  out_->print(" label=");
  out_->print(label != 0 ? label : "unknown");
  out_->print(" tcp_reads=");
  out_->print(tcpReadRequests);
  out_->print(" can_tx=");
  out_->print(canTxForwarded);
  out_->print(" cache_replies=");
  out_->print(cacheReplies);
  out_->print(" stale=");
  out_->print(cacheStale);
  out_->print(" miss=");
  out_->print(cacheMiss);
  out_->print(" coalesced=");
  out_->print(coalescedDuplicates);
  out_->print(" queue_full=");
  out_->println(queueFull);
}

void DebugCli::logNameSegmentCacheMiss(uint8_t subnet,
                                       uint8_t device,
                                       FieldId fieldId,
                                       uint8_t segmentNumber,
                                       const char* reason) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("NAME_SEGMENT_CACHE_MISS node=");
  out_->print(subnet);
  out_->print('.');
  out_->print(device);
  out_->print(" field=");
  out_->print(fieldIdName(fieldId));
  out_->print(" segment=");
  out_->print(segmentNumber);
  out_->print(" reason=");
  out_->println(reason != 0 ? reason : "unknown");
}

void DebugCli::logNameCacheInvalidate(uint8_t subnet,
                                      uint8_t device,
                                      FieldId fieldId,
                                      const char* reason) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("NAME_CACHE_INVALIDATE node=");
  out_->print(subnet);
  out_->print('.');
  out_->print(device);
  out_->print(" field=");
  out_->print(fieldIdName(fieldId));
  out_->print(" reason=");
  out_->println(reason != 0 ? reason : "unknown");
}

void DebugCli::logNormalQueueLatency(uint32_t oldestAgeMs, uint16_t depth, bool critical) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print(critical ? "NORMAL_QUEUE_LATENCY_CRITICAL oldest_age_ms="
                       : "NORMAL_QUEUE_LATENCY_WARN oldest_age_ms=");
  out_->print(oldestAgeMs);
  out_->print(" depth=");
  out_->println(depth);
}

void DebugCli::logWriteQueueFull(const Frame& frame) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("WRITE_QUEUE_FULL can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(frame.canId, HEX);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logIdentQueued(uint8_t subnet, const Frame& frame) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("IDENT_QUEUED subnet=");
  out_->print(subnet);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->println(frame.canId, HEX);
}

void DebugCli::logTcpCanTxFail(const NormalizedRequest& request, const char* reason) const {
  if (out_ == 0 || !request.valid || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("TCP_CAN_TX_FAIL id=0x");
  if (request.frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(request.frame.canId, HEX);
  out_->print(" dlc=");
  out_->print(request.frame.length);
  out_->print(" reason=");
  out_->print(reason != 0 ? reason : "unknown");
  out_->print(" subnet=");
  out_->print(request.canId.subnet);
  out_->print(" device=");
  out_->print(request.canId.device);
  if (request.frame.length >= 3) {
    out_->print(" object=");
    printHexByte(*out_, request.frame.data[1]);
    printHexByte(*out_, request.frame.data[2]);
  }
  out_->print(" field=");
  out_->print(fieldIdName(request.fieldId));
  if (request.mapping != 0 && request.mapping->label != 0) {
    out_->print(" label=");
    out_->print(request.mapping->label);
  }
  out_->println();
}

void DebugCli::logMuteWriteQueued(uint8_t subnet,
                                  uint8_t device,
                                  char channel,
                                  uint8_t value,
                                  const Frame& frame) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("MUTE_WRITE_QUEUED node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" channel=");
  out_->print(channel);
  out_->print(" value=");
  out_->print(value);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(frame.canId, HEX);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logMuteWriteTx(uint8_t subnet,
                              uint8_t device,
                              char channel,
                              uint8_t value,
                              const Frame& frame) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("MUTE_WRITE_TX node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" channel=");
  out_->print(channel);
  out_->print(" value=");
  out_->print(value);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(frame.canId, HEX);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logMuteWriteAck(uint8_t subnet,
                               uint8_t device,
                               char channel,
                               uint8_t value,
                               const Frame& frame) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("MUTE_WRITE_ACK node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" channel=");
  out_->print(channel);
  out_->print(" value=");
  out_->print(value);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->print(frame.canId, HEX);
  out_->print(" data=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      out_->print(' ');
    }
    printHexByte(*out_, frame.data[index]);
  }
  out_->println();
}

void DebugCli::logMuteWriteTimeout(uint8_t subnet,
                                   uint8_t device,
                                   char channel,
                                   uint8_t value,
                                   uint32_t ageMs) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("MUTE_WRITE_TIMEOUT node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" channel=");
  out_->print(channel);
  out_->print(" value=");
  out_->print(value);
  out_->print(" age_ms=");
  out_->println(ageMs);
}

void DebugCli::logIdentTx(uint8_t subnet, const Frame& frame) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("IDENT_TX subnet=");
  out_->print(subnet);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->println(frame.canId, HEX);
}

void DebugCli::logIdentRx(uint8_t subnet, uint8_t device, const Frame& frame) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("IDENT_RX subnet=");
  out_->print(subnet);
  out_->print(" device=");
  out_->print(device);
  out_->print(" can_id=0x");
  if (frame.canId < 0x100) {
    out_->print('0');
  }
  out_->println(frame.canId, HEX);
}

void DebugCli::logIdentSummary(uint8_t subnet,
                               uint32_t rxCount,
                               uint32_t rx441,
                               uint32_t rx442) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  out_->print("IDENT_SUMMARY subnet=");
  out_->print(subnet);
  out_->print(" rx_count=");
  out_->print(rxCount);
  if (subnet == 1) {
    out_->print(" rx_0x441=");
    out_->print(rx441);
    out_->print(" rx_0x442=");
    out_->print(rx442);
  }
  out_->println();
}

void DebugCli::logNodeReplyAgeTop(uint8_t subnet,
                                  uint8_t device,
                                  uint8_t rank,
                                  uint32_t nowMs,
                                  uint32_t lastTcpTxAgeMs,
                                  uint32_t lastRealCanRxAgeMs,
                                  uint32_t lastCacheReplyAgeMs,
                                  uint32_t lastCanTxPollAgeMs,
                                  uint32_t tcpTxCount,
                                  uint32_t realCanRxCount,
                                  uint32_t cacheReplyCount,
                                  uint32_t canTxPollCount) const {
  if (out_ == 0) {
    return;
  }
  printTimestampPrefix(*out_, nowMs);
  out_->print("NODE_REPLY_AGE_TOP rank=");
  out_->print(rank);
  out_->print(" node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" last_tcp_tx_age_ms=");
  out_->print(lastTcpTxAgeMs);
  out_->print(" last_real_can_rx_age_ms=");
  out_->print(lastRealCanRxAgeMs);
  out_->print(" last_cache_reply_age_ms=");
  out_->print(lastCacheReplyAgeMs);
  out_->print(" last_can_tx_poll_age_ms=");
  out_->print(lastCanTxPollAgeMs);
  out_->print(" tcp_tx=");
  out_->print(tcpTxCount);
  out_->print(" can_rx=");
  out_->print(realCanRxCount);
  out_->print(" cache_reply=");
  out_->print(cacheReplyCount);
  out_->print(" can_tx=");
  out_->println(canTxPollCount);
}

void DebugCli::logNodeTimeoutRisk(uint8_t subnet,
                                  uint8_t device,
                                  uint32_t nowMs,
                                  uint32_t lastTcpTxAgeMs,
                                  uint32_t lastRealCanRxAgeMs,
                                  uint32_t lastCacheReplyAgeMs,
                                  uint32_t lastCanTxPollAgeMs) const {
  if (out_ == 0) {
    return;
  }
  printTimestampPrefix(*out_, nowMs);
  out_->print("NODE_TIMEOUT_RISK node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" last_tcp_tx_age_ms=");
  out_->print(lastTcpTxAgeMs);
  out_->print(" last_real_can_rx_age_ms=");
  out_->print(lastRealCanRxAgeMs);
  out_->print(" last_cache_reply_age_ms=");
  out_->print(lastCacheReplyAgeMs);
  out_->print(" last_can_tx_poll_age_ms=");
  out_->println(lastCanTxPollAgeMs);
}

void DebugCli::logNodeRequestWithoutReply(uint8_t subnet,
                                          uint8_t device,
                                          uint32_t nowMs,
                                          FieldId fieldId,
                                          uint32_t ageMs) const {
  if (out_ == 0) {
    return;
  }
  printTimestampPrefix(*out_, nowMs);
  out_->print("NODE_REQUEST_WITHOUT_REPLY node=");
  out_->print(subnet);
  out_->print('.');
  if (device < 10) {
    out_->print('0');
  }
  out_->print(device);
  out_->print(" request_field=");
  out_->print(fieldIdName(fieldId));
  out_->print(" age_ms=");
  out_->println(ageMs);
}

void DebugCli::logStateSummary(const DebugCounters& counters,
                               uint32_t nowMs,
                               uint32_t canTxFramesDelta,
                               uint32_t canRxFramesDelta,
                               uint32_t tcpRxFramesDelta,
                               uint32_t tcpTxFramesDelta,
                               uint32_t tcpBackpressureActiveMs,
                               uint32_t currentSpacingUs) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  printTimestampPrefix(*out_, nowMs);
  out_->print("STATE_SUMMARY can_tx_queue_depth=");
  out_->print(counters.can_tx_queue_depth);
  out_->print(" highwater=");
  out_->print(counters.can_tx_queue_highwater);
  out_->print(" queue_full_total=");
  out_->print(counters.can_tx_queue_full_total);
  out_->print(" can_tx_enqueue_failed_queue_full=");
  out_->print(counters.can_tx_enqueue_failed_queue_full);
  out_->print(" can_tx_enqueue_failed_other=");
  out_->print(counters.can_tx_enqueue_failed_other);
  out_->print(" can_tx_frames_delta=");
  out_->print(canTxFramesDelta);
  out_->print(" can_rx_frames_delta=");
  out_->print(canRxFramesDelta);
  out_->print(" tcp_rx_frames_delta=");
  out_->print(tcpRxFramesDelta);
  out_->print(" tcp_tx_frames_delta=");
  out_->print(tcpTxFramesDelta);
  out_->print(" tcp_backpressure_active=");
  out_->print(counters.tcp_backpressure_active);
  out_->print(" tcp_backpressure_active_ms=");
  out_->print(tcpBackpressureActiveMs);
  out_->print(" tcp_backpressure_limited_count=");
  out_->print(counters.tcp_backpressure_limited_count);
  out_->print(" tcp_rx_paused_due_to_can_queue=");
  out_->print(counters.tcp_rx_paused_due_to_can_queue);
  out_->print(" loop_max_us=");
  out_->print(counters.loop_max_us);
  out_->print(" loop_avg_us=");
  out_->print(counters.loop_avg_us);
  out_->print(" can_poll_max_gap_us=");
  out_->print(counters.can_poll_max_gap_us);
  out_->print(" tcp_poll_max_gap_us=");
  out_->print(counters.tcp_poll_max_gap_us);
  out_->print(" max_serial_print_burst_ms=");
  out_->print(counters.max_serial_print_burst_ms);
  out_->print(" serial_bytes_printed_delta=");
  out_->print(counters.serial_bytes_printed_delta);
  out_->print(" timeout_correlation_enabled=");
  out_->print(counters.timeout_correlation_enabled);
  out_->print(" verbose_frame_log_enabled=");
  out_->print(counters.verbose_frame_log_enabled);
  out_->print(" can_tx_spacing_us_current=");
  out_->print(currentSpacingUs);
  out_->print(" write_queue_depth=");
  out_->print(counters.can_tx_write_queue_depth);
  out_->print(" ident_queue_depth=");
  out_->print(counters.can_tx_ident_queue_depth);
  out_->print(" normal_read_queue_depth=");
  out_->print(counters.normal_read_queue_depth);
  out_->print(" normal_read_queue_oldest_age_ms=");
  out_->print(counters.normal_read_queue_oldest_age_ms);
  out_->print(" active_amp_count=");
  out_->print(counters.active_amp_count);
  out_->print(" field_cache_used=");
  out_->print(counters.field_cache_used);
  out_->print(" field_cache_capacity=");
  out_->print(counters.field_cache_capacity);
  out_->print(" field_cache_evictions=");
  out_->print(counters.field_cache_evictions);
  out_->print(" field_cache_insert_fail=");
  out_->print(counters.field_cache_insert_fail);
  out_->print(" segment_cache_used=");
  out_->print(counters.segment_cache_used);
  out_->print(" segment_cache_capacity=");
  out_->print(counters.segment_cache_capacity);
  out_->print(" segment_cache_insert_fail=");
  out_->print(counters.segment_cache_insert_fail);
  out_->print(" name_segments_learned=");
  out_->print(counters.name_segments_learned);
  out_->print(" name_segment_cache_read_requests=");
  out_->print(counters.name_segment_cache_read_requests);
  out_->print(" name_segment_cache_replies=");
  out_->print(counters.name_segment_cache_replies);
  out_->print(" name_segment_cache_miss=");
  out_->print(counters.name_segment_cache_miss);
  out_->print(" name_cache_invalidations=");
  out_->print(counters.name_cache_invalidations);
  out_->print(" name_cache_slots_used=");
  out_->print(counters.name_cache_slots_used);
  out_->print(" name_cache_slots_capacity=");
  out_->print(counters.name_cache_slots_capacity);
  out_->print(" cache_hits=");
  out_->print(counters.cache_hits);
  out_->print(" cache_misses=");
  out_->print(counters.cache_misses);
  out_->print(" cache_lookup_skipped_label_only=");
  out_->print(counters.cache_lookup_skipped_label_only);
  out_->print(" cache_replies_sent=");
  out_->print(counters.cache_replies_sent);
  out_->print(" cache_reply_ineligible=");
  out_->print(counters.cache_reply_ineligible);
  out_->print(" cache_reply_stale=");
  out_->print(counters.cache_reply_stale);
  out_->print(" config_scalar_cache_replies=");
  out_->print(counters.config_scalar_cache_replies);
  out_->print(" config_scalar_cache_miss=");
  out_->print(counters.config_scalar_cache_miss);
  out_->print(" config_scalar_cache_stale=");
  out_->print(counters.config_scalar_cache_stale);
  out_->print(" watchdog_status_cache_replies=");
  out_->print(counters.watchdog_status_cache_replies);
  out_->print(" watchdog_status_cache_miss=");
  out_->print(counters.watchdog_status_cache_miss);
  out_->print(" watchdog_status_cache_stale=");
  out_->print(counters.watchdog_status_cache_stale);
  out_->print(" critical_status_cache_replies=");
  out_->print(counters.critical_status_cache_replies);
  out_->print(" critical_status_cache_misses=");
  out_->print(counters.critical_status_cache_misses);
  out_->print(" critical_status_cache_stale=");
  out_->print(counters.critical_status_cache_stale);
  out_->print(" firmware_segment_read_requests=");
  out_->print(counters.firmware_segment_read_requests);
  out_->print(" firmware_segment_can_tx=");
  out_->print(counters.firmware_segment_can_tx);
  out_->print(" firmware_segment_can_rx=");
  out_->print(counters.firmware_segment_can_rx);
  out_->print(" firmware_segment_cache_candidate_hits_possible=");
  out_->print(counters.firmware_segment_cache_candidate_hits_possible);
  out_->print(" firmware_segment_cache_replies=");
  out_->print(counters.firmware_segment_cache_replies);
  out_->print(" firmware_segment_cache_miss=");
  out_->print(counters.firmware_segment_cache_miss);
  out_->print(" firmware_segment_cache_stale=");
  out_->print(counters.firmware_segment_cache_stale);
  out_->print(" firmware_segment_cache_learned=");
  out_->print(counters.firmware_segment_cache_learned);
  out_->print(" firmware_segment_cache_slots_used=");
  out_->print(counters.firmware_segment_cache_slots_used);
  out_->print(" firmware_segment_cache_slots_capacity=");
  out_->print(counters.firmware_segment_cache_slots_capacity);
  out_->print(" unknown_0111_cache_replies=");
  out_->print(counters.unknown_0111_cache_replies);
  out_->print(" unknown_0111_cache_miss=");
  out_->print(counters.unknown_0111_cache_miss);
  out_->print(" unknown_0111_cache_stale=");
  out_->print(counters.unknown_0111_cache_stale);
  out_->print(" unknown_0111_cache_learned=");
  out_->print(counters.unknown_0111_cache_learned);
  out_->print(" unknown_0111_read_requests=");
  out_->print(counters.unknown_0111_read_requests);
  out_->print(" unknown_0111_can_tx=");
  out_->print(counters.unknown_0111_can_tx);
  out_->print(" unknown_0111_can_rx=");
  out_->print(counters.unknown_0111_can_rx);
  out_->print(" unknown_0111_tcp_tx=");
  out_->print(counters.unknown_0111_tcp_tx);
  out_->print(" unknown_0111_pending=");
  out_->print(counters.unknown_0111_pending);
  out_->print(" gpi_enabled_mask=");
  out_->print(counters.gpi_enabled_mask);
  out_->print(" gpi_action_backup_tx=");
  out_->print(counters.gpi_action_backup_tx);
  out_->print(" gpi_action_load_tx=");
  out_->print(counters.gpi_action_load_tx);
  out_->print(" gpi_action_restore_tx=");
  out_->print(counters.gpi_action_restore_tx);
  out_->print(" gpi_action_send_fail=");
  out_->print(counters.gpi_action_send_fail);
  out_->print(" gpi_press_count=");
  out_->print(counters.gpi_press_count);
  out_->print(" gpi_release_count=");
  out_->print(counters.gpi_release_count);
  out_->print(" gpi_restore_repeat_sent=");
  out_->print(counters.gpi_restore_repeat_sent);
  out_->print(" gpi_can_tx_failed=");
  out_->print(counters.gpi_can_tx_failed);
  out_->print(" gpi_pending_repeats=");
  out_->print(counters.gpi_pending_repeats);
  out_->print(" gpi_route_trigger_count=");
  out_->print(counters.gpi_route_trigger_count);
  out_->print(" gpi_route_backup_recall_tx=");
  out_->print(counters.gpi_route_backup_recall_tx);
  out_->print(" gpi_route_revert_tx=");
  out_->print(counters.gpi_route_revert_tx);
  out_->print(" gpi_route_revert_pending=");
  out_->print(counters.gpi_route_revert_pending);
  out_->print(" web_requests_total=");
  out_->print(counters.web_requests_total);
  out_->print(" web_save_network_count=");
  out_->print(counters.web_save_network_count);
  out_->print(" web_error_count=");
  out_->print(counters.web_error_count);
  out_->print(" web_route_backup_recall_count=");
  out_->print(counters.web_route_backup_recall_count);
  out_->print(" web_route_revert_count=");
  out_->print(counters.web_route_revert_count);
  out_->print(" web_route_error_count=");
  out_->print(counters.web_route_error_count);
  out_->print(" stable_setup_cache_replies=");
  out_->print(counters.stable_setup_cache_replies);
  out_->print(" stable_setup_cache_miss=");
  out_->print(counters.stable_setup_cache_miss);
  out_->print(" stable_setup_cache_stale=");
  out_->print(counters.stable_setup_cache_stale);
  out_->print(" stable_setup_cache_invalidations=");
  out_->print(counters.stable_setup_cache_invalidations);
  out_->print(" speaker_id_read_requests=");
  out_->print(counters.speaker_id_read_requests);
  out_->print(" speaker_id_can_tx=");
  out_->print(counters.speaker_id_can_tx);
  out_->print(" speaker_id_can_rx=");
  out_->print(counters.speaker_id_can_rx);
  out_->print(" speaker_id_cache_replies=");
  out_->print(counters.speaker_id_cache_replies);
  out_->print(" speaker_id_cache_miss=");
  out_->print(counters.speaker_id_cache_miss);
  out_->print(" speaker_id_cache_stale=");
  out_->print(counters.speaker_id_cache_stale);
  out_->print(" input_mode_read_requests=");
  out_->print(counters.input_mode_read_requests);
  out_->print(" input_mode_can_tx=");
  out_->print(counters.input_mode_can_tx);
  out_->print(" input_mode_can_rx=");
  out_->print(counters.input_mode_can_rx);
  out_->print(" input_mode_cache_replies=");
  out_->print(counters.input_mode_cache_replies);
  out_->print(" input_mode_cache_miss=");
  out_->print(counters.input_mode_cache_miss);
  out_->print(" input_mode_cache_stale=");
  out_->print(counters.input_mode_cache_stale);
  out_->print(" datalink_preset_read_requests=");
  out_->print(counters.datalink_preset_read_requests);
  out_->print(" datalink_preset_can_tx=");
  out_->print(counters.datalink_preset_can_tx);
  out_->print(" datalink_preset_can_rx=");
  out_->print(counters.datalink_preset_can_rx);
  out_->print(" datalink_preset_cache_candidate_hits_possible=");
  out_->print(counters.datalink_preset_cache_candidate_hits_possible);
  out_->print(" cache_updates_from_can=");
  out_->print(counters.cache_updates_from_can);
  out_->print(" read_coalesced_duplicate_count=");
  out_->print(counters.read_coalesced_duplicate_count);
  out_->print(" write_queue_full=");
  out_->print(counters.write_queue_full);
  out_->print(" ident_queue_full=");
  out_->print(counters.ident_queue_full);
  out_->print(" normal_queue_full=");
  out_->print(counters.normal_queue_full);
  out_->print(" write_tx_count=");
  out_->print(counters.write_tx_count);
  out_->print(" write_ack_count=");
  out_->print(counters.write_ack_count);
  out_->print(" write_timeout_count=");
  out_->print(counters.write_timeout_count);
  out_->print(" ident_tx_count=");
  out_->print(counters.ident_tx_count);
  out_->print(" ident_rx_count=");
  out_->print(counters.ident_rx_count);
  out_->print(" mute_write_tx_count=");
  out_->print(counters.mute_write_tx_count);
  out_->print(" mute_write_ack_count=");
  out_->print(counters.mute_write_ack_count);
  out_->print(" mute_write_timeout_count=");
  out_->print(counters.mute_write_timeout_count);
  out_->print(" psram_total=");
  out_->print(counters.psram_total);
  out_->print(" psram_free=");
  out_->println(counters.psram_free);
}

void DebugCli::logLongRunPerfSummary(const DebugCounters& counters,
                                     uint32_t nowMs,
                                     uint32_t durationSeconds) const {
  if (out_ == 0 || !verboseFrameLogEnabled_) {
    return;
  }
  printTimestampPrefix(*out_, nowMs);
  out_->print("LONG_RUN_PERF_SUMMARY duration_s=");
  out_->print(durationSeconds);
  out_->print(" normal_oldest_min=");
  out_->print(counters.normal_read_queue_oldest_age_ms_min);
  out_->print(" normal_oldest_avg=");
  out_->print(counters.normal_read_queue_oldest_age_ms_avg);
  out_->print(" normal_oldest_max=");
  out_->print(counters.normal_read_queue_oldest_age_ms_max);
  out_->print(" normal_depth_max=");
  out_->print(counters.normal_read_queue_depth_max);
  out_->print(" can_tx_depth_max=");
  out_->print(counters.can_tx_queue_depth_max);
  out_->print(" queue_full_delta=");
  out_->print(counters.queue_full_total_delta);
  out_->print(" backpressure_ms_delta=");
  out_->print(counters.tcp_backpressure_active_ms_delta);
  out_->print(" cache_replies_delta=");
  out_->print(counters.cache_replies_sent_delta);
  out_->print(" can_tx_delta=");
  out_->print(counters.can_tx_frames_delta_long_run);
  out_->print(" can_rx_delta=");
  out_->println(counters.can_rx_frames_delta_long_run);
}

void DebugCli::printCounters(const DebugCounters& counters) const {
  if (out_ == 0) {
    return;
  }
  out_->print("PASSIVE_SUMMARY active_amp_count=");
  out_->print(counters.active_amp_count);
  out_->print(" can_tx_per_sec=");
  out_->print(counters.can_tx_per_sec);
  out_->print(" can_rx_per_sec=");
  out_->print(counters.can_rx_per_sec);
  out_->print(" tcp_rx_per_sec=");
  out_->print(counters.tcp_rx_per_sec);
  out_->print(" tcp_tx_per_sec=");
  out_->println(counters.tcp_tx_per_sec);

  out_->print("PASSIVE_SUMMARY normal_q_hwm=");
  out_->print(counters.normal_read_queue_highwater);
  out_->print(" can_tx_q_hwm=");
  out_->print(counters.can_tx_queue_highwater);
  out_->print(" max_can_response_latency_ms=");
  out_->print(counters.max_can_response_latency_ms);
  out_->print(" avg_can_response_latency_ms=");
  out_->print(counters.avg_can_response_latency_ms);
  out_->print(" can_response_over_500ms=");
  out_->print(counters.can_response_over_500ms);
  out_->print(" can_response_over_1000ms=");
  out_->println(counters.can_response_over_1000ms);

  out_->print("PASSIVE_SUMMARY r1_timeout_events=");
  out_->print(counters.r1_timeout_events);
  out_->print(" channel_incomplete_count=");
  out_->print(counters.channel_incomplete_count);
  out_->print(" bootstrap_incomplete_after_10s=");
  out_->print(counters.bootstrap_incomplete_after_10s);
  out_->print(" subnet_service_gap_count=");
  out_->print(counters.subnet_service_gap_count);
  out_->print(" deferred_requests_active=");
  out_->print(counters.deferred_requests_active);
  out_->print(" dropped_or_suppressed_requests=");
  out_->print(counters.dropped_or_suppressed_requests);
  out_->print(" segmented_requests_suppressed=");
  out_->print(counters.segmented_requests_suppressed);
  out_->print(" fanout_waiters_active=");
  out_->print(counters.fanout_waiters_active);
  out_->print(" retry_requests_sent=");
  out_->print(counters.retry_requests_sent);
  out_->print(" requests_received=");
  out_->print(counters.requests_received);
  out_->print(" requests_completed=");
  out_->print(counters.requests_completed);
  out_->print(" requests_pending=");
  out_->print(counters.requests_pending);
  out_->print(" requests_expired=");
  out_->print(counters.requests_expired);
  out_->print(" requests_dropped=");
  out_->print(counters.requests_dropped);
  out_->print(" requests_suppressed=");
  out_->print(counters.requests_suppressed);
  out_->print(" requests_cancelled=");
  out_->println(counters.requests_cancelled);

  out_->print("STATEFUL_CACHE_SUMMARY stateful_cache_enabled=");
  out_->print(counters.stateful_cache_enabled);
  out_->print(" cache_hits_10s=");
  out_->print(counters.stateful_cache_hits_10s);
  out_->print(" cache_misses_10s=");
  out_->print(counters.stateful_cache_misses_10s);
  out_->print(" can_reads_prevented_10s=");
  out_->print(counters.stateful_can_reads_prevented_10s);
  out_->print(" inflight_joined_10s=");
  out_->print(counters.stateful_inflight_joined_10s);
  out_->print(" cache_age_max_ms=");
  out_->print(counters.stateful_cache_age_max_ms);
  out_->print(" cache_disabled_reason=");
  out_->println(counters.stateful_cache_disabled_reason);
}

void DebugCli::printBootstrapDiagnostics(const DebugCounters& counters) const {
  if (out_ == 0 || counters.bootstrap_nodes_incomplete == 0) {
    return;
  }
  out_->print("BOOTSTRAP_SUMMARY bootstrap_nodes_total=");
  out_->print(counters.bootstrap_nodes_total);
  out_->print(" bootstrap_nodes_incomplete=");
  out_->print(counters.bootstrap_nodes_incomplete);
  out_->print(" bootstrap_retry_sent=");
  out_->print(counters.bootstrap_retry_sent);
  out_->print(" bootstrap_retry_replied=");
  out_->print(counters.bootstrap_retry_replied);
  out_->print(" bootstrap_retry_failed=");
  out_->print(counters.bootstrap_retry_failed);
  out_->print(" retry_requests_sent=");
  out_->print(counters.retry_requests_sent);
  out_->print(" deferred_requests_active=");
  out_->print(counters.deferred_requests_active);
  out_->print(" fanout_waiters_active=");
  out_->print(counters.fanout_waiters_active);
  out_->print(" worst_bootstrap_node=");
  if (counters.bootstrap_worst_node_device == 0) {
    out_->print("none");
  } else {
    out_->print(counters.bootstrap_worst_node_subnet);
    out_->print('.');
    if (counters.bootstrap_worst_node_device < 10) {
      out_->print('0');
    }
    out_->print(counters.bootstrap_worst_node_device);
  }
  out_->print(" worst_missing_object=");
  if (counters.bootstrap_worst_missing_object == 0) {
    out_->println("none");
    return;
  }
  printHexWord(*out_, static_cast<uint16_t>(counters.bootstrap_worst_missing_object & 0xFFFFu));
  out_->println();
}

}  // namespace r70v2
}  // namespace dbcan
