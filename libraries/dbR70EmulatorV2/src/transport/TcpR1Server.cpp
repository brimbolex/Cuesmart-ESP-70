#include "TcpR1Server.h"

namespace dbcan {
namespace r70v2 {

TcpR1Server::TcpR1Server()
    : server_(0), rxLength_(0), expectedLength_(0), queueHead_(0), queueTail_(0), queueCount_(0) {
  for (uint8_t index = 0; index < sizeof(liveFrame_); ++index) {
    liveFrame_[index] = 0;
  }
}

void TcpR1Server::begin(uint16_t port) {
  static EthernetServer server(port);
  server_ = &server;
  server_->begin();
  client_.stop();
  rxLength_ = 0;
  expectedLength_ = 0;
  for (uint8_t index = 0; index < sizeof(liveFrame_); ++index) {
    liveFrame_[index] = 0;
  }
  queueHead_ = 0;
  queueTail_ = 0;
  queueCount_ = 0;
}

bool TcpR1Server::queueFrame(const uint8_t* bytes, uint8_t rawLength, const TcpEnvelope& envelope) {
  if (queueCount_ >= kQueueCapacity) {
    return false;
  }
  QueuedFrame& slot = queue_[queueTail_];
  slot = {};
  slot.used = true;
  slot.rawLength = rawLength;
  slot.envelope = envelope;
  for (uint8_t index = 0; index < rawLength; ++index) {
    slot.raw[index] = bytes[index];
  }
  queueTail_ = static_cast<uint8_t>((queueTail_ + 1) % kQueueCapacity);
  ++queueCount_;
  return true;
}

void TcpR1Server::poll(DebugCounters& counters, const DebugCli& debugCli, bool allowRead) {
  if (server_ == 0) {
    return;
  }

  if (!client_ || !client_.connected()) {
    EthernetClient incoming = server_->available();
    if (incoming) {
      client_.stop();
      client_ = incoming;
      ++counters.tcp_clients_connected;
      rxLength_ = 0;
      expectedLength_ = 0;
    }
  }

  if (!client_ || !client_.connected()) {
    return;
  }

  if (!allowRead) {
    ++counters.tcp_rx_paused_due_to_can_queue;
    return;
  }

  while (queueCount_ < kQueueCapacity && client_.available() > 0) {
    const int next = client_.read();
    if (next < 0) {
      break;
    }

    const uint8_t byteValue = static_cast<uint8_t>(next);
    if (rxLength_ == 0 && byteValue != R70Frame::kStartByte) {
      continue;
    }
    if (rxLength_ >= R70Frame::kMaxFrameLength) {
      rxLength_ = 0;
      expectedLength_ = 0;
      continue;
    }

    liveFrame_[rxLength_++] = byteValue;
    if (rxLength_ == 2) {
      expectedLength_ = static_cast<uint8_t>(liveFrame_[1] + 4);
      if (expectedLength_ < 7 || expectedLength_ > R70Frame::kMaxFrameLength) {
        rxLength_ = 0;
        expectedLength_ = 0;
      }
    }

    if (expectedLength_ > 0 && rxLength_ == expectedLength_) {
      ++counters.tcp_rx_frames;
      ++counters.tcp_frames_received;
      TcpEnvelope envelope = {};
      if (R70Frame::parse(liveFrame_, rxLength_, envelope)) {
        if (envelope.checksumValid) {
          ++counters.tcp_rx_crc_ok;
        } else {
          ++counters.tcp_rx_crc_fail;
        }
        debugCli.logTcpFrame("TCP_RX ", liveFrame_, rxLength_, true, envelope.checksumValid);
        if (!queueFrame(liveFrame_, rxLength_, envelope)) {
          break;
        }
      } else {
        ++counters.tcp_frames_unknown;
        ++counters.tcp_unknown_frames;
        debugCli.logTcpFrame("TCP_RX ", liveFrame_, rxLength_);
      }
      rxLength_ = 0;
      expectedLength_ = 0;
      for (uint8_t index = 0; index < sizeof(liveFrame_); ++index) {
        liveFrame_[index] = 0;
      }
    }
  }
}

bool TcpR1Server::nextFrame(TcpEnvelope& envelope,
                            uint8_t* rawBytes,
                            uint8_t& rawLength,
                            DebugCounters& counters) {
  rawLength = 0;
  if (queueCount_ == 0) {
    return false;
  }
  QueuedFrame& slot = queue_[queueHead_];
  envelope = slot.envelope;
  rawLength = slot.rawLength;
  for (uint8_t index = 0; index < rawLength; ++index) {
    rawBytes[index] = slot.raw[index];
  }
  slot = {};
  queueHead_ = static_cast<uint8_t>((queueHead_ + 1) % kQueueCapacity);
  --queueCount_;
  ++counters.tcp_frames_parsed;
  return true;
}

bool TcpR1Server::sendFrame(const uint8_t* bytes, uint8_t length, DebugCounters* counters) {
  if (!connected() || bytes == 0 || length == 0) {
    return false;
  }
  if (client_.write(bytes, length) != length) {
    return false;
  }
  if (counters != 0) {
    ++counters->tcp_tx_frames;
  }
  return true;
}

bool TcpR1Server::connected() const {
  EthernetClient& client = const_cast<EthernetClient&>(client_);
  return client.connected() == 1;
}

}  // namespace r70v2
}  // namespace dbcan
