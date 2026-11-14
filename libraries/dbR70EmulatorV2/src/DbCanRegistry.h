#ifndef DBCAN_REGISTRY_H
#define DBCAN_REGISTRY_H

#include "DbCanTypes.h"

namespace dbcan {
namespace registry {

extern const ObjectDescriptor settings_power;
extern const ObjectDescriptor config_delay_on_a;
extern const ObjectDescriptor config_delay_on_b;
extern const ObjectDescriptor config_mute_a;
extern const ObjectDescriptor config_mute_b;
extern const ObjectDescriptor config_gain_a;
extern const ObjectDescriptor config_gain_b;
extern const ObjectDescriptor config_cut_a;
extern const ObjectDescriptor config_hfa_a;
extern const ObjectDescriptor config_routing_a;
extern const ObjectDescriptor config_name_a;
extern const ObjectDescriptor channel_delay_value_a;
extern const ObjectDescriptor channel_delay_aux_a;

extern const ObjectDescriptor eq_frequency_a;
extern const ObjectDescriptor eq_q_a;
extern const ObjectDescriptor eq_gain_a;
extern const ObjectDescriptor eq_mode_a;

const ObjectDescriptor* findByKey(const char* key);
const ObjectDescriptor* descriptors();
uint8_t descriptorCount();

}  // namespace registry
}  // namespace dbcan

#endif  // DBCAN_REGISTRY_H
