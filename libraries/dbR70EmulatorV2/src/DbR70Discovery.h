#ifndef DB_R70_DISCOVERY_H
#define DB_R70_DISCOVERY_H

#include <Arduino.h>
#include <EthernetUdp.h>
#include <IPAddress.h>
#include <Print.h>
#include <stddef.h>
#include <stdint.h>

#include "DbCanProtocol.h"
#include "DbCanTypes.h"

namespace dbcan {

static const uint8_t kR70DiscoveryRequestLength = 68;
static const uint8_t kR70DiscoveryHeaderLength = 4;
static const uint8_t kR70DiscoveryEntryLength = 64;
static const uint8_t kMaxR70DiscoveryEntries = 5;
static const uint16_t kR70DiscoveryReplyLength =
    kR70DiscoveryHeaderLength + kMaxR70DiscoveryEntries * kR70DiscoveryEntryLength;
static const uint8_t kR70DiscoveryProductBytes = 22;
static const uint8_t kR70DiscoveryHostnameBytes = 28;

extern const uint8_t kR70DiscoveryDefaultMeta[kMaxR70DiscoveryEntries][4];

struct R70DiscoveryEndpoint {
  IPAddress ip;
  uint16_t port;
};

struct R70DiscoveryEntry {
  IPAddress ip;
  uint8_t meta[4];  // Confirmed present at bytes 4..7; meaning intentionally undecoded.
  uint8_t mac[6];
  char product[kR70DiscoveryProductBytes + 1];
  char hostname[kR70DiscoveryHostnameBytes + 1];
};

struct R70DiscoveryReply {
  uint8_t header[kR70DiscoveryHeaderLength];
  uint8_t reportedEntryCount;
  uint8_t entryCount;
  R70DiscoveryEntry entries[kMaxR70DiscoveryEntries];
};

struct DeviceInfo {
  uint8_t ip[4];
  uint8_t mac[6];
  const char* product;
  const char* hostname;
  // Kept for source compatibility. Discovery reply generation uses the fixed
  // measured R70 rawMeta table, not per-device identity data.
  uint8_t meta[kMaxR70DiscoveryEntries][4];
};

const uint8_t* r70DiscoveryRequestPayload();
ResultCode buildR70DiscoveryRequest(uint8_t* out, size_t outCapacity, size_t& outLength);
bool isR70DiscoveryRequest(const uint8_t* bytes, size_t length);
bool isR70DiscoveryProbe(const uint8_t* buf, size_t len);
ResultCode parseR70DiscoveryReply(const uint8_t* bytes,
                                  size_t length,
                                  R70DiscoveryReply& reply);
size_t buildR70DiscoveryReply(uint8_t* out, size_t maxLen, const DeviceInfo& dev);
bool sendDiscoveryReplyToProbe(EthernetUDP& udp,
                               const IPAddress& remoteIp,
                               uint16_t remotePort,
                               const DeviceInfo& dev);
void dumpDiscoveryReplySummary(Print& out, const uint8_t* bytes, size_t length);
void handleDiscoveryPacket(EthernetUDP& udp, const DeviceInfo& dev);

String r70IpToString(const IPAddress& ip);
String r70MacToString(const uint8_t mac[6]);
String r70MetaToString(const uint8_t meta[4]);
String r70DiscoveryText(const char* zeroPaddedText, size_t textCapacity);
void printR70DiscoveryEntry(Print& out, const R70DiscoveryEntry& entry, uint8_t index);
void printR70DiscoveryReply(Print& out, const R70DiscoveryReply& reply);

ResultCode sendR70DiscoveryProbe(EthernetUDP& udp);
ResultCode sendR70DiscoveryProbe(EthernetUDP& udp,
                                 const IPAddress& broadcastIp,
                                 uint16_t remotePort = protocol::kR70UdpDiscoveryPort);
ResultCode readR70DiscoveryReply(EthernetUDP& udp,
                                 R70DiscoveryReply& reply,
                                 uint32_t timeoutMs,
                                 R70DiscoveryEndpoint* endpoint = nullptr);
ResultCode discoverR70Devices(EthernetUDP& udp,
                              R70DiscoveryReply& reply,
                              uint32_t timeoutMs,
                              uint16_t localPort = protocol::kR70UdpDiscoveryPort);

class R70DiscoveryClient {
 public:
  bool begin(uint16_t localPort = protocol::kR70UdpDiscoveryPort);
  void end();
  bool isStarted() const;

  ResultCode sendProbe();
  ResultCode readReply(R70DiscoveryReply& reply,
                       uint32_t timeoutMs,
                       R70DiscoveryEndpoint* endpoint = nullptr);
  ResultCode discover(R70DiscoveryReply& reply,
                      uint32_t timeoutMs,
                      R70DiscoveryEndpoint* endpoint = nullptr);

 private:
  EthernetUDP udp_;
  bool started_ = false;
};

}  // namespace dbcan

#endif  // DB_R70_DISCOVERY_H
