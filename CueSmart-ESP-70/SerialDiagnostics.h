#pragma once
#include <Arduino.h>

// Set to 1 and rebuild to restore serial diagnostics and test commands.
#ifndef ENABLE_SERIAL_DIAGNOSTICS
#define ENABLE_SERIAL_DIAGNOSTICS 0
#endif

#if ENABLE_SERIAL_DIAGNOSTICS
#define DiagnosticSerial Serial
#else
class SilentDiagnosticStream : public Stream {
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t*, size_t size) override { return size; }
};
static SilentDiagnosticStream DiagnosticSerial;
#endif
