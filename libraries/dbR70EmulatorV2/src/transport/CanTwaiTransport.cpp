#include "CanTwaiTransport.h"

namespace dbcan {
namespace r70v2 {

CanTwaiTransport::CanTwaiTransport() : installed_(false), started_(false), listenOnly_(false) {}

CanTwaiTransport::~CanTwaiTransport() {
  end();
}

bool CanTwaiTransport::makeTimingConfig(uint32_t bitrate, twai_timing_config_t& timingConfig) {
  switch (bitrate) {
    case 100000:
      timingConfig = TWAI_TIMING_CONFIG_100KBITS();
      return true;
    case 125000:
      timingConfig = TWAI_TIMING_CONFIG_125KBITS();
      return true;
    case 250000:
      timingConfig = TWAI_TIMING_CONFIG_250KBITS();
      return true;
    case 500000:
      timingConfig = TWAI_TIMING_CONFIG_500KBITS();
      return true;
    default:
      return false;
  }
}

bool CanTwaiTransport::begin(gpio_num_t txPin,
                             gpio_num_t rxPin,
                             uint32_t bitrate,
                             bool listenOnly) {
  end();

  twai_timing_config_t timingConfig = {};
  if (!makeTimingConfig(bitrate, timingConfig)) {
    return false;
  }

  const twai_mode_t mode = listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, mode);
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&generalConfig, &timingConfig, &filterConfig) != ESP_OK) {
    return false;
  }
  installed_ = true;

  if (twai_start() != ESP_OK) {
    end();
    return false;
  }

  started_ = true;
  listenOnly_ = listenOnly;
  return true;
}

void CanTwaiTransport::end() {
  if (started_) {
    twai_stop();
    started_ = false;
  }
  if (installed_) {
    twai_driver_uninstall();
    installed_ = false;
  }
  listenOnly_ = false;
}

bool CanTwaiTransport::receive(Frame& frame, uint32_t timeoutMs) {
  if (!started_) {
    return false;
  }

  twai_message_t message = {};
  if (twai_receive(&message, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) {
    return false;
  }
  if (message.extd || message.data_length_code > kMaxFrameDataLength) {
    return false;
  }

  frame = {};
  frame.canId = static_cast<uint16_t>(message.identifier & 0x7FF);
  frame.length = message.data_length_code;
  for (uint8_t index = 0; index < frame.length; ++index) {
    frame.data[index] = message.data[index];
  }
  return true;
}

bool CanTwaiTransport::send(const Frame& frame, uint32_t timeoutMs) {
  if (!started_ || listenOnly_ || frame.canId > 0x7FF || frame.length > kMaxFrameDataLength) {
    return false;
  }

  twai_message_t message = {};
  message.identifier = frame.canId;
  message.extd = 0;
  message.rtr = 0;
  message.ss = 1;
  message.data_length_code = frame.length;
  for (uint8_t index = 0; index < frame.length; ++index) {
    message.data[index] = frame.data[index];
  }

  return twai_transmit(&message, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

}  // namespace r70v2
}  // namespace dbcan
