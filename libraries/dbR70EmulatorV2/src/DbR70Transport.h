#ifndef DB_R70_TRANSPORT_H
#define DB_R70_TRANSPORT_H

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <IPAddress.h>

#include "DbCanBackend.h"
#include "DbCanProtocol.h"
#include "DbR70Discovery.h"

namespace dbcan {

class R70Transport : public FrameBackend {
 public:
  explicit R70Transport(const Address& address);
  ~R70Transport();

  bool beginDiscovery(uint16_t udpPort = protocol::kR70UdpDiscoveryPort);
  ResultCode sendDiscoveryProbe();
  bool waitForDiscovery(IPAddress& remoteIp, uint32_t timeoutMs = 1000);
  bool connect(const IPAddress& remoteIp, uint16_t tcpPort = protocol::kR70TcpSessionPort);
  void end();

  bool isStarted() const override;

  ResultCode sendFrame(const Frame& frame, uint32_t timeoutMs = 100) override;
  ResultCode receiveFrame(Frame& frame, uint32_t timeoutMs = 20) override;

  bool sendRawPayload(const uint8_t* payload, uint8_t length, uint32_t timeoutMs = 100);
  ResultCode isAmpPresetActive(uint8_t node, uint8_t presetNo, bool& active, uint32_t timeoutMs = 100);

 private:
  ResultCode readByte(uint8_t& value, uint32_t deadlineMs);

  Address address_;
  EthernetUDP udp_;
  EthernetClient client_;
  bool udpStarted_ = false;
};

}  // namespace dbcan

#endif  // DB_R70_TRANSPORT_H
