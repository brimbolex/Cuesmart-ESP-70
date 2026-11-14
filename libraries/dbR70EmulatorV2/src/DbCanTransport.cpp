#include "DbCanTransport.h"

namespace dbcan {

Transport::~Transport() {
  end();
}

bool Transport::makeTimingConfig(uint32_t bitrate, twai_timing_config_t& timing) {
  switch (bitrate) {
    case 100000:
      timing = TWAI_TIMING_CONFIG_100KBITS();
      return true;
    case 125000:
      timing = TWAI_TIMING_CONFIG_125KBITS();
      return true;
    case 250000:
      timing = TWAI_TIMING_CONFIG_250KBITS();
      return true;
    case 500000:
      timing = TWAI_TIMING_CONFIG_500KBITS();
      return true;
    case 1000000:
      timing = TWAI_TIMING_CONFIG_1MBITS();
      return true;
    default:
      return false;
  }
}

bool Transport::begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t bitrate, bool listenOnly) {
  end();

  twai_timing_config_t t_config{};
  if (!makeTimingConfig(bitrate, t_config)) {
    return false;
  }

  twai_mode_t mode = listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, mode);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    installed_ = false;
    started_ = false;
    return false;
  }

  installed_ = true;

  err = twai_start();
  if (err != ESP_OK) {
    twai_driver_uninstall();
    installed_ = false;
    started_ = false;
    return false;
  }

  started_ = true;
  txPin_ = txPin;
  rxPin_ = rxPin;
  bitrate_ = bitrate;
  listenOnly_ = listenOnly;
  return true;
}

void Transport::end() {
  if (started_) {
    twai_stop();
    started_ = false;
  }

  if (installed_) {
    twai_driver_uninstall();
    installed_ = false;
  }

  txPin_ = GPIO_NUM_NC;
  rxPin_ = GPIO_NUM_NC;
  bitrate_ = 0;
  listenOnly_ = false;
}

ResultCode Transport::sendFrame(const Frame& frame, uint32_t timeoutMs) {
  if (!started_) {
    return ResultCode::NotSupported;
  }

  if (listenOnly_) {
    return ResultCode::NotSupported;
  }

  if (frame.length > 8) {
    return ResultCode::InvalidValue;
  }

  if (frame.canId > 0x7FF) {
    return ResultCode::InvalidValue;
  }

  twai_message_t msg{};
  msg.identifier = frame.canId;
  msg.extd = 0;
  msg.rtr = 0;
  msg.ss = 1;
  msg.self = 0;
  msg.data_length_code = frame.length;

  for (uint8_t i = 0; i < frame.length; ++i) {
    msg.data[i] = frame.data[i];
  }

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs));
  if (err == ESP_OK) {
    return ResultCode::Ok;
  }
  if (err == ESP_ERR_TIMEOUT) {
    return ResultCode::Timeout;
  }
  return ResultCode::Error;
}

ResultCode Transport::receiveFrame(Frame& frame, uint32_t timeoutMs) {
  if (!started_) {
    return ResultCode::NotSupported;
  }

  twai_message_t msg{};
  esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(timeoutMs));

  if (err == ESP_ERR_TIMEOUT) {
    return ResultCode::Timeout;
  }
  if (err != ESP_OK) {
    return ResultCode::Error;
  }

  if (msg.extd) {
    return ResultCode::NotSupported;
  }

  if (msg.data_length_code > 8) {
    return ResultCode::Error;
  }

  frame = {};
  frame.canId = static_cast<uint16_t>(msg.identifier & 0x7FF);
  frame.length = msg.data_length_code;

  for (uint8_t i = 0; i < frame.length; ++i) {
    frame.data[i] = msg.data[i];
  }

  return ResultCode::Ok;
}

bool Transport::sendRaw(uint16_t canId, const uint8_t* data, uint8_t len, uint32_t timeoutMs) {
  if (data == nullptr && len > 0) {
    return false;
  }

  Frame frame{};
  frame.canId = canId;
  frame.length = len;

  for (uint8_t i = 0; i < len && i < 8; ++i) {
    frame.data[i] = data[i];
  }

  return sendFrame(frame, timeoutMs) == ResultCode::Ok;
}

} // namespace dbcan
