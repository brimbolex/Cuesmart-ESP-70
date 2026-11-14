#pragma once

#include <Arduino.h>
#include "driver/twai.h"
#include "DbCanBackend.h"

namespace dbcan {

class Transport : public FrameBackend {
public:
  Transport() = default;
  ~Transport();

  Transport(const Transport&) = delete;
  Transport& operator=(const Transport&) = delete;

  bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t bitrate = 100000, bool listenOnly = false);
  void end();

  bool isStarted() const override { return started_; }

  ResultCode sendFrame(const Frame& frame, uint32_t timeoutMs = 100) override;
  ResultCode receiveFrame(Frame& frame, uint32_t timeoutMs = 20) override;

  bool sendRaw(uint16_t canId, const uint8_t* data, uint8_t len, uint32_t timeoutMs = 100);

private:
  static bool makeTimingConfig(uint32_t bitrate, twai_timing_config_t& timing);

  bool installed_ = false;
  bool started_ = false;
  gpio_num_t txPin_ = GPIO_NUM_NC;
  gpio_num_t rxPin_ = GPIO_NUM_NC;
  uint32_t bitrate_ = 0;
  bool listenOnly_ = false;
};

} // namespace dbcan
