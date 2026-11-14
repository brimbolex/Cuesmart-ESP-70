#include "DbR70Discovery.h"

#include <string.h>

namespace dbcan {

namespace {

const uint8_t kR70DiscoveryRequest[kR70DiscoveryRequestLength] = {
    0x01, 0x00, 0x80, 0x01,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x52, 0x37,
    0x30, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00};

const uint8_t kR70DiscoveryLegacyRequestHeader[4] = {0x00, 0x00, 0x80, 0x01};
const uint8_t kR70DiscoveryModelMarker[4] = {0x52, 0x37, 0x30, 0x20};

bool timedOut(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void copyZeroPaddedString(const uint8_t* source, size_t sourceLength, char* dest, size_t destSize) {
  if (dest == nullptr || destSize == 0) {
    return;
  }

  size_t index = 0;
  while (index < sourceLength && index + 1 < destSize && source[index] != 0) {
    dest[index] = static_cast<char>(source[index]);
    ++index;
  }
  dest[index] = '\0';
}

void writeZeroPaddedString(uint8_t* dest, size_t destLength, const char* text) {
  if (dest == nullptr || destLength == 0) {
    return;
  }

  memset(dest, 0, destLength);
  if (text == nullptr) {
    return;
  }

  size_t index = 0;
  while (index < destLength && text[index] != '\0') {
    dest[index] = static_cast<uint8_t>(text[index]);
    ++index;
  }
}

void printHexByte(Print& out, uint8_t value) {
  if (value < 0x10) {
    out.print('0');
  }
  out.print(value, HEX);
}

ResultCode parsePacketFromUdp(EthernetUDP& udp, int packetSize, R70DiscoveryReply& reply) {
  uint8_t buffer[kR70DiscoveryHeaderLength + kMaxR70DiscoveryEntries * kR70DiscoveryEntryLength] = {};
  int remaining = packetSize;
  size_t stored = 0;

  while (remaining > 0) {
    uint8_t discard[32] = {};
    uint8_t* target = discard;
    size_t capacity = sizeof(discard);
    if (stored < sizeof(buffer)) {
      target = buffer + stored;
      capacity = sizeof(buffer) - stored;
    }

    const int chunk = udp.read(target, remaining < static_cast<int>(capacity) ? remaining : static_cast<int>(capacity));
    if (chunk <= 0) {
      break;
    }
    if (target != discard) {
      stored += static_cast<size_t>(chunk);
    }
    remaining -= chunk;
  }

  while (udp.available() > 0) {
    udp.read();
  }

  if (packetSize > static_cast<int>(sizeof(buffer))) {
    return ResultCode::InvalidValue;
  }

  return parseR70DiscoveryReply(buffer, stored, reply);
}

}  // namespace

extern const uint8_t kR70DiscoveryDefaultMeta[kMaxR70DiscoveryEntries][4] = {
    {0x03, 0x00, 0x00, 0x50},
    {0x05, 0x01, 0x00, 0x45},
    {0x06, 0x00, 0x00, 0x17},
    {0x00, 0xFF, 0x00, 0x00},
    {0x02, 0x00, 0x75, 0x30},
};

const uint8_t* r70DiscoveryRequestPayload() {
  return kR70DiscoveryRequest;
}

ResultCode buildR70DiscoveryRequest(uint8_t* out, size_t outCapacity, size_t& outLength) {
  outLength = 0;
  if (out == nullptr || outCapacity < kR70DiscoveryRequestLength) {
    return ResultCode::InvalidValue;
  }

  memcpy(out, kR70DiscoveryRequest, kR70DiscoveryRequestLength);
  outLength = kR70DiscoveryRequestLength;
  return ResultCode::Ok;
}

bool isR70DiscoveryRequest(const uint8_t* bytes, size_t length) {
  return bytes != nullptr && length == kR70DiscoveryRequestLength &&
         memcmp(bytes, kR70DiscoveryRequest, kR70DiscoveryRequestLength) == 0;
}

bool isR70DiscoveryProbe(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len != kR70DiscoveryRequestLength) {
    return false;
  }

  const bool confirmedHeader = memcmp(buf, kR70DiscoveryRequest, 4) == 0;
  const bool legacyHeader = memcmp(buf, kR70DiscoveryLegacyRequestHeader, 4) == 0;
  if (!confirmedHeader && !legacyHeader) {
    return false;
  }

  return memcmp(buf + 18, kR70DiscoveryModelMarker, sizeof(kR70DiscoveryModelMarker)) == 0;
}

ResultCode parseR70DiscoveryReply(const uint8_t* bytes,
                                  size_t length,
                                  R70DiscoveryReply& reply) {
  reply = {};
  if (bytes == nullptr || length < kR70DiscoveryHeaderLength) {
    return ResultCode::InvalidValue;
  }

  memcpy(reply.header, bytes, sizeof(reply.header));

  if (bytes[0] != 0x01 || bytes[1] != 0x00 || bytes[2] != 0x00) {
    return ResultCode::NotSupported;
  }

  const uint8_t count = bytes[3];
  reply.reportedEntryCount = count;
  if (count > kMaxR70DiscoveryEntries) {
    return ResultCode::NotSupported;
  }

  const size_t expectedMinimum =
      kR70DiscoveryHeaderLength + static_cast<size_t>(count) * kR70DiscoveryEntryLength;
  if (length < expectedMinimum) {
    return ResultCode::InvalidValue;
  }

  reply.entryCount = count;
  for (uint8_t index = 0; index < count; ++index) {
    const uint8_t* record = bytes + kR70DiscoveryHeaderLength +
                            static_cast<size_t>(index) * kR70DiscoveryEntryLength;
    R70DiscoveryEntry& entry = reply.entries[index];

    entry.ip = IPAddress(record[0], record[1], record[2], record[3]);
    memcpy(entry.meta, record + 4, sizeof(entry.meta));
    memcpy(entry.mac, record + 8, sizeof(entry.mac));
    copyZeroPaddedString(record + 14, kR70DiscoveryProductBytes, entry.product,
                         sizeof(entry.product));
    copyZeroPaddedString(record + 36, kR70DiscoveryHostnameBytes, entry.hostname,
                         sizeof(entry.hostname));
  }

  return ResultCode::Ok;
}

size_t buildR70DiscoveryReply(uint8_t* out, size_t maxLen, const DeviceInfo& dev) {
  if (out == nullptr || maxLen < kR70DiscoveryReplyLength) {
    return 0;
  }

  memset(out, 0, kR70DiscoveryReplyLength);
  out[0] = 0x01;
  out[1] = 0x00;
  out[2] = 0x00;
  out[3] = kMaxR70DiscoveryEntries;

  const char* product = dev.product != nullptr ? dev.product : "R70 ";
  const char* hostname = dev.hostname != nullptr ? dev.hostname : "db-r70";

  for (uint8_t index = 0; index < kMaxR70DiscoveryEntries; ++index) {
    uint8_t* record = out + kR70DiscoveryHeaderLength +
                      static_cast<size_t>(index) * kR70DiscoveryEntryLength;

    // Confirmed fields from two real R70 replies:
    // IP, fixed rawMeta, MAC, product string, and hostname string.
    memcpy(record + 0, dev.ip, 4);
    memcpy(record + 4, kR70DiscoveryDefaultMeta[index], 4);
    memcpy(record + 8, dev.mac, 6);
    writeZeroPaddedString(record + 14, kR70DiscoveryProductBytes, product);
    writeZeroPaddedString(record + 36, kR70DiscoveryHostnameBytes, hostname);
  }

  return kR70DiscoveryReplyLength;
}

bool sendDiscoveryReplyToProbe(EthernetUDP& udp,
                               const IPAddress& remoteIp,
                               uint16_t remotePort,
                               const DeviceInfo& dev) {
  uint8_t reply[kR70DiscoveryReplyLength] = {};
  const size_t replyLength = buildR70DiscoveryReply(reply, sizeof(reply), dev);
  if (replyLength != kR70DiscoveryReplyLength) {
    return false;
  }

  if (udp.beginPacket(remoteIp, remotePort) != 1) {
    return false;
  }

  udp.write(reply, replyLength);
  return udp.endPacket() == 1;
}

void dumpDiscoveryReplySummary(Print& out, const uint8_t* bytes, size_t length) {
  if (bytes == nullptr) {
    out.println(F("R70 discovery reply: null"));
    return;
  }

  out.print(F("R70 discovery reply length: "));
  out.println(length);
  if (length != kR70DiscoveryReplyLength) {
    out.println(F("R70 discovery reply length invalid"));
    return;
  }

  out.print(F("R70 discovery header: "));
  for (uint8_t index = 0; index < kR70DiscoveryHeaderLength; ++index) {
    if (index > 0) {
      out.print(' ');
    }
    printHexByte(out, bytes[index]);
  }
  out.println();

  for (uint8_t index = 0; index < kMaxR70DiscoveryEntries; ++index) {
    const uint8_t* record = bytes + kR70DiscoveryHeaderLength +
                            static_cast<size_t>(index) * kR70DiscoveryEntryLength;
    R70DiscoveryEntry entry = {};
    entry.ip = IPAddress(record[0], record[1], record[2], record[3]);
    memcpy(entry.meta, record + 4, sizeof(entry.meta));
    memcpy(entry.mac, record + 8, sizeof(entry.mac));
    copyZeroPaddedString(record + 14, kR70DiscoveryProductBytes, entry.product,
                         sizeof(entry.product));
    copyZeroPaddedString(record + 36, kR70DiscoveryHostnameBytes, entry.hostname,
                         sizeof(entry.hostname));
    printR70DiscoveryEntry(out, entry, index);
  }
}

void handleDiscoveryPacket(EthernetUDP& udp, const DeviceInfo& dev) {
  const int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  const IPAddress remoteIp = udp.remoteIP();
  const uint16_t remotePort = udp.remotePort();
  uint8_t probe[kR70DiscoveryRequestLength] = {};
  size_t stored = 0;
  int remaining = packetSize;

  while (remaining > 0) {
    uint8_t discard[32] = {};
    uint8_t* target = discard;
    size_t capacity = sizeof(discard);
    if (stored < sizeof(probe)) {
      target = probe + stored;
      capacity = sizeof(probe) - stored;
    }

    const int chunk = udp.read(target, remaining < static_cast<int>(capacity) ? remaining : static_cast<int>(capacity));
    if (chunk <= 0) {
      break;
    }
    if (target != discard) {
      stored += static_cast<size_t>(chunk);
    }
    remaining -= chunk;
  }

  while (udp.available() > 0) {
    udp.read();
  }

  if (stored != kR70DiscoveryRequestLength ||
      !isR70DiscoveryProbe(probe, static_cast<size_t>(packetSize))) {
    return;
  }

  sendDiscoveryReplyToProbe(udp, remoteIp, remotePort, dev);
}

String r70IpToString(const IPAddress& ip) {
  char text[16] = {};
  snprintf(text, sizeof(text), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return String(text);
}

String r70MacToString(const uint8_t mac[6]) {
  char text[18] = {};
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

String r70MetaToString(const uint8_t meta[4]) {
  char text[12] = {};
  snprintf(text, sizeof(text), "%02X %02X %02X %02X",
           meta[0], meta[1], meta[2], meta[3]);
  return String(text);
}

String r70DiscoveryText(const char* zeroPaddedText, size_t textCapacity) {
  if (zeroPaddedText == nullptr || textCapacity == 0) {
    return String();
  }

  size_t length = 0;
  while (length < textCapacity && zeroPaddedText[length] != '\0') {
    ++length;
  }

  String value;
  value.reserve(length);
  for (size_t index = 0; index < length; ++index) {
    value += zeroPaddedText[index];
  }
  return value;
}

void printR70DiscoveryEntry(Print& out, const R70DiscoveryEntry& entry, uint8_t index) {
  out.print(F("R70 discovery entry "));
  out.println(index);
  out.print(F("  ip: "));
  out.println(r70IpToString(entry.ip));
  out.print(F("  meta[4]: "));
  out.println(r70MetaToString(entry.meta));
  out.print(F("  mac: "));
  out.println(r70MacToString(entry.mac));
  out.print(F("  product: "));
  out.println(r70DiscoveryText(entry.product, sizeof(entry.product)));
  out.print(F("  hostname: "));
  out.println(r70DiscoveryText(entry.hostname, sizeof(entry.hostname)));
}

void printR70DiscoveryReply(Print& out, const R70DiscoveryReply& reply) {
  out.print(F("R70 discovery header: "));
  for (uint8_t index = 0; index < sizeof(reply.header); ++index) {
    if (index > 0) {
      out.print(' ');
    }
    printHexByte(out, reply.header[index]);
  }
  out.println();
  out.print(F("R70 discovery records: "));
  out.print(reply.entryCount);
  out.print(F(" reported="));
  out.println(reply.reportedEntryCount);

  for (uint8_t index = 0; index < reply.entryCount; ++index) {
    printR70DiscoveryEntry(out, reply.entries[index], index);
  }
}

ResultCode sendR70DiscoveryProbe(EthernetUDP& udp) {
  return sendR70DiscoveryProbe(udp, IPAddress(255, 255, 255, 255),
                               protocol::kR70UdpDiscoveryPort);
}

ResultCode sendR70DiscoveryProbe(EthernetUDP& udp,
                                 const IPAddress& broadcastIp,
                                 uint16_t remotePort) {
  uint8_t request[kR70DiscoveryRequestLength] = {};
  size_t requestLength = 0;
  ResultCode result = buildR70DiscoveryRequest(request, sizeof(request), requestLength);
  if (result != ResultCode::Ok) {
    return result;
  }

  if (udp.beginPacket(broadcastIp, remotePort) != 1) {
    return ResultCode::Error;
  }

  if (udp.write(request, requestLength) != requestLength) {
    return ResultCode::Timeout;
  }

  return udp.endPacket() == 1 ? ResultCode::Ok : ResultCode::Error;
}

ResultCode readR70DiscoveryReply(EthernetUDP& udp,
                                 R70DiscoveryReply& reply,
                                 uint32_t timeoutMs,
                                 R70DiscoveryEndpoint* endpoint) {
  const uint32_t deadline = millis() + timeoutMs;
  while (!timedOut(millis(), deadline)) {
    const int packetSize = udp.parsePacket();
    if (packetSize <= 0) {
      delay(1);
      continue;
    }

    R70DiscoveryReply parsed = {};
    const ResultCode result = parsePacketFromUdp(udp, packetSize, parsed);
    if (result == ResultCode::Ok) {
      reply = parsed;
      if (endpoint != nullptr) {
        endpoint->ip = udp.remoteIP();
        endpoint->port = udp.remotePort();
      }
      return ResultCode::Ok;
    }
  }

  return ResultCode::Timeout;
}

ResultCode discoverR70Devices(EthernetUDP& udp,
                              R70DiscoveryReply& reply,
                              uint32_t timeoutMs,
                              uint16_t localPort) {
  if (udp.begin(localPort) != 1) {
    return ResultCode::Error;
  }

  ResultCode result = sendR70DiscoveryProbe(udp);
  if (result != ResultCode::Ok) {
    return result;
  }

  return readR70DiscoveryReply(udp, reply, timeoutMs);
}

bool R70DiscoveryClient::begin(uint16_t localPort) {
  if (started_) {
    return true;
  }
  started_ = udp_.begin(localPort) == 1;
  return started_;
}

void R70DiscoveryClient::end() {
  if (started_) {
    udp_.stop();
    started_ = false;
  }
}

bool R70DiscoveryClient::isStarted() const {
  return started_;
}

ResultCode R70DiscoveryClient::sendProbe() {
  if (!started_ && !begin()) {
    return ResultCode::Error;
  }
  return sendR70DiscoveryProbe(udp_);
}

ResultCode R70DiscoveryClient::readReply(R70DiscoveryReply& reply,
                                         uint32_t timeoutMs,
                                         R70DiscoveryEndpoint* endpoint) {
  if (!started_ && !begin()) {
    return ResultCode::Error;
  }
  return readR70DiscoveryReply(udp_, reply, timeoutMs, endpoint);
}

ResultCode R70DiscoveryClient::discover(R70DiscoveryReply& reply,
                                        uint32_t timeoutMs,
                                        R70DiscoveryEndpoint* endpoint) {
  ResultCode result = sendProbe();
  if (result != ResultCode::Ok) {
    return result;
  }
  return readReply(reply, timeoutMs, endpoint);
}

}  // namespace dbcan
