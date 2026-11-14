#ifndef DB_R70_V2_TCP_R1_SERVER_H
#define DB_R70_V2_TCP_R1_SERVER_H

#include <Ethernet.h>
#include <stdint.h>

#include "../debug/DebugCli.h"
#include "../debug/DebugCounters.h"
#include "../protocol/R70Frame.h"

namespace dbcan {
namespace r70v2 {

class TcpR1Server {
 public:
  TcpR1Server();

  void begin(uint16_t port);
  void poll(DebugCounters& counters, const DebugCli& debugCli, bool allowRead = true);
  bool nextFrame(TcpEnvelope& envelope, uint8_t* rawBytes, uint8_t& rawLength, DebugCounters& counters);
  bool sendFrame(const uint8_t* bytes, uint8_t length, DebugCounters* counters = 0);
  bool connected() const;

 private:
  static const uint8_t kQueueCapacity = 8;

  struct QueuedFrame {
    bool used;
    uint8_t rawLength;
    uint8_t raw[R70Frame::kMaxFrameLength];
    TcpEnvelope envelope;
  };

  EthernetServer* server_;
  EthernetClient client_;
  uint8_t rxLength_;
  uint8_t expectedLength_;
  uint8_t liveFrame_[R70Frame::kMaxFrameLength];
  QueuedFrame queue_[kQueueCapacity];
  uint8_t queueHead_;
  uint8_t queueTail_;
  uint8_t queueCount_;

  bool queueFrame(const uint8_t* bytes, uint8_t rawLength, const TcpEnvelope& envelope);
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_TCP_R1_SERVER_H
