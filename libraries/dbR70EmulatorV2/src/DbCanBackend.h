#ifndef DBCAN_BACKEND_H
#define DBCAN_BACKEND_H

#include "DbCanTypes.h"

namespace dbcan {

class FrameBackend {
 public:
  virtual ~FrameBackend() {}

  virtual bool isStarted() const = 0;
  virtual ResultCode sendFrame(const Frame& frame, uint32_t timeoutMs = 100) = 0;
  virtual ResultCode receiveFrame(Frame& frame, uint32_t timeoutMs = 20) = 0;
};

}  // namespace dbcan

#endif  // DBCAN_BACKEND_H
