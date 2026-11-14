#include "UdpDiscovery.h"

#include <string.h>

#include "../R70GatewayV2.h"

namespace dbcan {
namespace r70v2 {

namespace {

const uint8_t kConfirmedHeader[4] = {0x01, 0x00, 0x80, 0x01};
const uint8_t kLegacyHeader[4] = {0x00, 0x00, 0x80, 0x01};
const uint8_t kModelMarker[4] = {0x52, 0x37, 0x30, 0x20};
const uint8_t kDiscoveryMeta[kDiscoveryRecordCount][4] = {
    {0x03, 0x00, 0x00, 0x50},
    {0x05, 0x01, 0x00, 0x45},
    {0x06, 0x00, 0x00, 0x17},
    {0x00, 0xFF, 0x00, 0x00},
    {0x02, 0x00, 0x75, 0x30},
};

void writeZeroPadded(uint8_t* dest, size_t length, const char* text) {
  memset(dest, 0, length);
  if (text == 0) {
    return;
  }
  for (size_t index = 0; index < length && text[index] != '\0'; ++index) {
    dest[index] = static_cast<uint8_t>(text[index]);
  }
}

}  // namespace

bool UdpDiscovery::begin(uint16_t port) {
  if (started_) {
    return true;
  }
  started_ = udp_.begin(port) == 1;
  return started_;
}

void UdpDiscovery::end() {
  if (started_) {
    udp_.stop();
    started_ = false;
  }
}

UdpDiscovery::ProbeVariant UdpDiscovery::classify(const uint8_t* bytes, size_t length) {
  if (bytes == 0 || length != 68) {
    return ProbeVariant::Unknown;
  }
  const bool confirmed = memcmp(bytes, kConfirmedHeader, sizeof(kConfirmedHeader)) == 0;
  const bool legacy = memcmp(bytes, kLegacyHeader, sizeof(kLegacyHeader)) == 0;
  if ((confirmed || legacy) && memcmp(bytes + 18, kModelMarker, sizeof(kModelMarker)) == 0) {
    return confirmed ? ProbeVariant::Confirmed : ProbeVariant::Legacy;
  }
  return ProbeVariant::Unknown;
}

uint16_t UdpDiscovery::buildReply(const GatewayIdentity& identity, uint8_t* out, size_t capacity) {
  if (out == 0 || capacity < kDiscoveryReplyLength) {
    return 0;
  }

  memset(out, 0, kDiscoveryReplyLength);
  out[0] = 0x01;
  out[1] = 0x00;
  out[2] = 0x00;
  out[3] = kDiscoveryRecordCount;

  for (uint8_t index = 0; index < kDiscoveryRecordCount; ++index) {
    uint8_t* record = out + 4 + static_cast<size_t>(index) * kDiscoveryRecordLength;
    record[0] = identity.ip[0];
    record[1] = identity.ip[1];
    record[2] = identity.ip[2];
    record[3] = identity.ip[3];
    memcpy(record + 4, kDiscoveryMeta[index], 4);
    memcpy(record + 8, identity.mac, 6);
    writeZeroPadded(record + 14, 22, identity.product);
    writeZeroPadded(record + 36, 28, identity.hostname);
  }

  return kDiscoveryReplyLength;
}

void UdpDiscovery::poll(const GatewayIdentity& identity,
                        DebugCounters& counters,
                        const DebugCli& debugCli) {
  if (!started_) {
    return;
  }

  const int packetSize = udp_.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  uint8_t buffer[96] = {};
  const int readLength = udp_.read(buffer, sizeof(buffer));
  while (udp_.available() > 0) {
    udp_.read();
  }
  if (readLength <= 0) {
    return;
  }

  ++counters.udp_discovery_probes;
  debugCli.logTcpFrame("UDP_DISCOVERY_PROBE ", buffer, static_cast<uint8_t>(readLength));

  const ProbeVariant variant = classify(buffer, static_cast<size_t>(readLength));
  if (variant == ProbeVariant::Unknown) {
    ++counters.udp_discovery_unknown_variants;
    return;
  }

  uint8_t reply[kDiscoveryReplyLength] = {};
  const uint16_t replyLength = buildReply(identity, reply, sizeof(reply));
  if (replyLength == 0) {
    return;
  }

  if (udp_.beginPacket(udp_.remoteIP(), udp_.remotePort()) != 1) {
    return;
  }
  udp_.write(reply, replyLength);
  if (udp_.endPacket() == 1) {
    ++counters.udp_discovery_replies;
    debugCli.logTcpFrame("UDP_DISCOVERY_REPLY ", reply, 32);
  }
}

}  // namespace r70v2
}  // namespace dbcan
