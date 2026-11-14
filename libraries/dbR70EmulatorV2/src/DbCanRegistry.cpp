#include "DbCanRegistry.h"

#include <string.h>

namespace dbcan {
namespace registry {

namespace {

const ObjectDescriptor kDescriptors[] = {
    {"settings_power",
     {0x01, 0x00},
     ObjectClass::Settings,
     Channel::None,
     Access::ReadWrite,
     Confidence::Confirmed,
     ValueType::Boolean,
     "CSV confirmed Settings_PwrOn object 0100."},
    {"config_delay_on_a",
     {0x02, 0x03},
     ObjectClass::Config,
     Channel::A,
     Access::ReadWrite,
     Confidence::Confirmed,
     ValueType::Boolean,
     "CSV confirmed 22 03 write/read for Delay On/Off A."},
    {"config_delay_on_b",
     {0x02, 0x03},
     ObjectClass::Config,
     Channel::B,
     Access::ReadWrite,
     Confidence::Confirmed,
     ValueType::Boolean,
     "CSV confirmed 42 03 write/read for Delay On/Off B."},
    {"config_mute_a",
     {0x02, 0x05},
     ObjectClass::Config,
     Channel::A,
     Access::ReadWrite,
     Confidence::Confirmed,
     ValueType::Boolean,
     "CSV confirmed 22 05 write/read for Mute A."},
    {"config_mute_b",
     {0x02, 0x05},
     ObjectClass::Config,
     Channel::B,
     Access::ReadWrite,
     Confidence::Confirmed,
     ValueType::Boolean,
     "CSV confirmed 42 05 write/read for Mute B."},
    {"config_gain_a",
     {0x02, 0x06},
     ObjectClass::Config,
     Channel::A,
     Access::ReadWrite,
     Confidence::Working,
     ValueType::SignedInteger,
     "CSV confirmed object 0206; value encoding retained as raw signed integer for now."},
    {"config_gain_b",
     {0x02, 0x06},
     ObjectClass::Config,
     Channel::B,
     Access::ReadWrite,
     Confidence::Working,
     ValueType::SignedInteger,
     "CSV confirmed object 0206; value encoding retained as raw signed integer for now."},
    {"config_cut_a",
     {0x02, 0x00},
     ObjectClass::Config,
     Channel::A,
     Access::ReadWrite,
     Confidence::Observed,
     ValueType::Boolean,
     "R70 notes map 22 00 to Cut A; keep semantic status observed."},
    {"config_hfa_a",
     {0x02, 0x01},
     ObjectClass::Config,
     Channel::A,
     Access::ReadWrite,
     Confidence::Observed,
     ValueType::Boolean,
     "R70 notes map 22 01 to HFA A; keep semantic status observed."},
    {"config_routing_a",
     {0x02, 0x0B},
     ObjectClass::Config,
     Channel::A,
     Access::ReadWrite,
     Confidence::Confirmed,
     ValueType::UnsignedInteger,
     "R70 notes confirm Routing A values: 0=A, 1=B, 2=A+B."},
    {"config_name_a",
     {0x02, 0x15},
     ObjectClass::Config,
     Channel::A,
     Access::ReadWrite,
     Confidence::Observed,
     ValueType::Text,
     "R70 notes map 22 14/15 to Name A; segmented string support remains minimal."},
    {"channel_delay_value_a",
     {0x04, 0x15},
     ObjectClass::Status,
     Channel::A,
     Access::ReadWrite,
     Confidence::Observed,
     ValueType::SignedInteger,
     "R70 notes map 24 15 to Delay Value A; scaling not implemented."},
    {"channel_delay_aux_a",
     {0x04, 0x17},
     ObjectClass::Status,
     Channel::A,
     Access::Read,
     Confidence::Observed,
     ValueType::SignedInteger,
     "R70 notes map 24 17 to Delay Aux A; exact meaning not implemented."},
    {"eq_frequency_a",
     {0x07, 0x00},
     ObjectClass::Eq,
     Channel::A,
     Access::File,
     Confidence::Confirmed,
     ValueType::UnsignedInteger,
     "CSV confirmed read/write file object 0700 for EQ A frequency bands 1..4."},
    {"eq_q_a",
     {0x07, 0x01},
     ObjectClass::Eq,
     Channel::A,
     Access::File,
     Confidence::Confirmed,
     ValueType::UnsignedInteger,
     "CSV confirmed read/write file object 0701 for EQ A quality bands 1..4."},
    {"eq_gain_a",
     {0x07, 0x02},
     ObjectClass::Eq,
     Channel::A,
     Access::File,
     Confidence::Confirmed,
     ValueType::SignedInteger,
     "CSV confirmed read/write file object 0702 for EQ A gain bands 1..4."},
    {"eq_mode_a",
     {0x07, 0x03},
     ObjectClass::Eq,
     Channel::A,
     Access::File,
     Confidence::Likely,
     ValueType::UnsignedInteger,
     "CSV marks mode values as likely; keep raw until bus-verified."},
};

}  // namespace

const ObjectDescriptor settings_power = kDescriptors[0];
const ObjectDescriptor config_delay_on_a = kDescriptors[1];
const ObjectDescriptor config_delay_on_b = kDescriptors[2];
const ObjectDescriptor config_mute_a = kDescriptors[3];
const ObjectDescriptor config_mute_b = kDescriptors[4];
const ObjectDescriptor config_gain_a = kDescriptors[5];
const ObjectDescriptor config_gain_b = kDescriptors[6];
const ObjectDescriptor config_cut_a = kDescriptors[7];
const ObjectDescriptor config_hfa_a = kDescriptors[8];
const ObjectDescriptor config_routing_a = kDescriptors[9];
const ObjectDescriptor config_name_a = kDescriptors[10];
const ObjectDescriptor channel_delay_value_a = kDescriptors[11];
const ObjectDescriptor channel_delay_aux_a = kDescriptors[12];
const ObjectDescriptor eq_frequency_a = kDescriptors[13];
const ObjectDescriptor eq_q_a = kDescriptors[14];
const ObjectDescriptor eq_gain_a = kDescriptors[15];
const ObjectDescriptor eq_mode_a = kDescriptors[16];

const ObjectDescriptor* findByKey(const char* key) {
  if (key == 0) {
    return 0;
  }

  for (uint8_t index = 0; index < descriptorCount(); ++index) {
    if (strcmp(kDescriptors[index].key, key) == 0) {
      return &kDescriptors[index];
    }
  }

  return 0;
}

const ObjectDescriptor* descriptors() {
  return kDescriptors;
}

uint8_t descriptorCount() {
  return static_cast<uint8_t>(sizeof(kDescriptors) / sizeof(kDescriptors[0]));
}

}  // namespace registry
}  // namespace dbcan
