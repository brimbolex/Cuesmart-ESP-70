#ifndef DB_R70_V2_UDP_DISCOVERY_H
#define DB_R70_V2_UDP_DISCOVERY_H

#include <EthernetUdp.h>
#include <IPAddress.h>
#include <stdint.h>

#include "../debug/DebugCli.h"
#include "../debug/DebugCounters.h"

namespace dbcan {
namespace r70v2 {

struct GatewayIdentity;

class UdpDiscovery {
 public:
  bool begin(uint16_t port);
  void end();
  void poll(const GatewayIdentity& identity, DebugCounters& counters, const DebugCli& debugCli);

 private:
  enum class ProbeVariant : uint8_t {
    Confirmed,
    Legacy,
    Unknown
  };

  static ProbeVariant classify(const uint8_t* bytes, size_t length);
  static uint16_t buildReply(const GatewayIdentity& identity, uint8_t* out, size_t capacity);

  EthernetUDP udp_;
  bool started_ = false;
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_UDP_DISCOVERY_H
