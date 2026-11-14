#ifndef DBCAN_TYPES_H
#define DBCAN_TYPES_H

#include <stddef.h>
#include <stdint.h>

#include "DbCanFrame.h"

namespace dbcan {

enum class ObjectClass : uint8_t {
  Unknown,
  Settings,
  Config,
  Status,
  Meter,
  Eq,
  File,
  Placeholder
};

enum class Channel : uint8_t {
  None,
  A,
  B,
  Both,
  Unknown
};

enum class ResultCode : uint8_t {
  Ok,
  Error,
  Timeout,
  InvalidValue,
  NotSupported,
  PlaceholderOnly
};

enum class Access : uint8_t {
  Unknown,
  None,
  Read,
  Write,
  ReadWrite,
  File
};

enum class Confidence : uint8_t {
  Confirmed,
  Working,
  Observed,
  Likely,
  Placeholder
};

enum class ValueType : uint8_t {
  Unknown,
  Boolean,
  SignedInteger,
  UnsignedInteger,
  Float,
  Text,
  Bytes
};

struct Address {
  uint8_t subnet;
  uint8_t device;
};

struct ObjectId {
  uint16_t box;
  uint16_t object;
};

struct ObjectDescriptor {
  const char* key;
  ObjectId id;
  ObjectClass objectClass;
  Channel channel;
  Access access;
  Confidence confidence;
  ValueType valueType;
  const char* notes;
};

}  // namespace dbcan

#endif  // DBCAN_TYPES_H
