#ifndef DB_CAN_SERIAL_COMMAND_H
#define DB_CAN_SERIAL_COMMAND_H

#include <Arduino.h>
#include <ESP.h>
#include <string.h>

namespace dbcan {

class SerialRebootCommandHandler {
 public:
  bool shouldPoll(int nextByte) const {
    return length_ != 0 || nextByte == 'c' || nextByte == '\r' || nextByte == '\n';
  }

  template <typename SerialType>
  void poll(SerialType& serial, bool configSaveInProgress) {
    while (serial.available() > 0) {
      const int next = serial.read();
      if (next < 0) {
        break;
      }
      processByte(static_cast<char>(next), configSaveInProgress);
    }
  }

  bool handleLine(const char* command, bool configSaveInProgress) {
    if (command == nullptr || strcmp(command, "cmd:reboot") != 0) {
      return false;
    }

    if (configSaveInProgress) {
      Serial.println(F("ERR reboot blocked: config save in progress"));
      return true;
    }

    Serial.println(F("ACK cmd:reboot"));
    Serial.flush();
    delay(100);
    ESP.restart();
    return true;
  }

 private:
  void reset() {
    length_ = 0;
    overflowed_ = false;
    buffer_[0] = '\0';
  }

  void processByte(char c, bool configSaveInProgress) {
    if (c == '\r') {
      return;
    }

    if (c == '\n') {
      if (overflowed_) {
        Serial.println(F("ERR serial cmd too long"));
        reset();
        return;
      }

      buffer_[length_] = '\0';
      handleLine(length_ == 0 ? nullptr : buffer_, configSaveInProgress);
      reset();
      return;
    }

    if (overflowed_) {
      return;
    }

    if (length_ + 1 < sizeof(buffer_)) {
      buffer_[length_++] = c;
      buffer_[length_] = '\0';
      return;
    }

    overflowed_ = true;
    length_ = 0;
    buffer_[0] = '\0';
  }

  char buffer_[80] = {};
  size_t length_ = 0;
  bool overflowed_ = false;
};

}  // namespace dbcan

#endif  // DB_CAN_SERIAL_COMMAND_H
