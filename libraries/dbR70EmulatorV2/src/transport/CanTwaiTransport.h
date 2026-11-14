#ifndef DB_R70_V2_CAN_TWAI_TRANSPORT_H
#define DB_R70_V2_CAN_TWAI_TRANSPORT_H

#include <Arduino.h>
#include <stdint.h>

#include "driver/twai.h"

#include "../DbCanFrame.h"

namespace dbcan {
namespace r70v2 {

class CanTwaiTransport {
 public:
  CanTwaiTransport();
  ~CanTwaiTransport();

  bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t bitrate, bool listenOnly);
  void end();

  bool receive(Frame& frame, uint32_t timeoutMs = 0);
  bool send(const Frame& frame, uint32_t timeoutMs = 20);

 private:
  static bool makeTimingConfig(uint32_t bitrate, twai_timing_config_t& timingConfig);

  bool installed_;
  bool started_;
  bool listenOnly_;
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_CAN_TWAI_TRANSPORT_H
