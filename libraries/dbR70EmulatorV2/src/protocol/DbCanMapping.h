#ifndef DB_R70_V2_DBCAN_MAPPING_H
#define DB_R70_V2_DBCAN_MAPPING_H

#include <stddef.h>
#include <stdint.h>

#include "../DbCanFrame.h"

namespace dbcan {
namespace r70v2 {

enum class StructureConfidence : uint8_t {
  Unknown,
  SpecDefined,
  CaptureConfirmed
};

enum class BehaviorConfidence : uint8_t {
  Unknown,
  SeenInSpecOnly,
  SeenOnTcp,
  SeenOnCan,
  ConfirmedCapture,
  ConfirmedRoundTrip
};

enum class FieldClass : uint8_t {
  Unknown,
  Device,
  Control,
  Status,
  Name,
  Candidate
};

enum class FieldId : uint8_t {
  Unknown = 0,
  fixed_serial_number,
  fixed_firmware_version,
  settings_power,
  settings_lock_cmd,
  settings_buzzer,
  settings_device_name,
  settings_input_mode,
  settings_data_link_preset,
  config_filter1_a,
  config_filter1_b,
  config_filter2_a,
  config_filter2_b,
  config_filter3_a,
  config_filter3_b,
  config_cut_a,
  config_cut_b,
  config_hfa_a,
  config_hfa_b,
  config_delay_on_a,
  config_delay_on_b,
  config_mute_a,
  config_mute_b,
  config_gain_a,
  config_gain_b,
  config_delay_value_a,
  config_delay_value_b,
  config_name_a,
  config_name_b,
  config_delay_dim_a,
  config_delay_dim_b,
  config_input_source_a,
  config_input_source_b,
  config_eqs_enabled_a,
  config_eqs_enabled_b,
  config_speaker_id_a,
  config_speaker_id_b,
  status_pwr_ok,
  status_text,
  status_digin_status,
  status_hardware_type,
  chstatus_isp_a,
  chstatus_isp_b,
  chstatus_gr_a,
  chstatus_gr_b,
  chstatus_ovl_a,
  chstatus_ovl_b,
  chstatus_osp_a,
  chstatus_osp_b,
  chstatus_gr_head_a,
  chstatus_gr_head_b,
  chstatus_input_voltage_a,
  chstatus_input_voltage_b,
  chstatus_input_voltage_peak_a,
  chstatus_input_voltage_peak_b,
  chstatus_speaker_impedance_a,
  chstatus_speaker_impedance_b,
  chstatus_gr_head_peak_a,
  chstatus_gr_head_peak_b,
  chstatus_output_power_a,
  chstatus_output_power_b,
  chstatus_output_power_peak_a,
  chstatus_output_power_peak_b,
  chstatus_ms_delay_a,
  chstatus_ms_delay_b,
  chstatus_amp_temperature_a,
  chstatus_amp_temperature_b,
  chstatus_input_overload_a,
  chstatus_input_overload_b,
  chstatus_output_overload_a,
  chstatus_output_overload_b,
  chstatus_remhold_isp_a,
  chstatus_remhold_isp_b,
  chstatus_remhold_gr_a,
  chstatus_remhold_gr_b,
  chstatus_remhold_ovl_a,
  chstatus_remhold_ovl_b,
  chstatus_remhold_osp_a,
  chstatus_remhold_osp_b,
  chstatus_remhold_gr_head_a,
  chstatus_remhold_gr_head_b,
  error_general,
  error_number,
  error_text,
  cherr_channel_err_a,
  cherr_channel_err_b,
  cherr_err_nr_a,
  cherr_err_nr_b,
  cherr_error_text_a,
  cherr_error_text_b,
  cherr_en_imp_err_a,
  cherr_en_imp_err_b,
  cherr_en_imp_err_lf_a,
  cherr_en_imp_err_lf_b,
  cherr_en_imp_err_hf_a,
  cherr_en_imp_err_hf_b,
  cherr_psp_err_a,
  cherr_psp_err_b,
  eq_frequency_a,
  eq_frequency_b,
  eq_q_a,
  eq_q_b,
  eq_gain_a,
  eq_gain_b,
  eq_mode_a,
  eq_mode_b,
  loadmon_enabled_a,
  loadmon_enabled_b,
  Count
};

enum class FunctionCode : uint8_t {
  Command = 0,
  ReadObject = 1,
  WriteObject = 2
};

enum class TelegramType : uint8_t {
  Unstructured = 0,
  Segmented = 1,
  File = 2,
  FileSegmented = 3
};

enum class RequestOrigin : uint8_t {
  Tcp,
  Can
};

struct CanIdParts {
  bool valid;
  uint8_t objectClass;
  uint8_t subnet;
  uint8_t device;
};

struct AdminParts {
  bool valid;
  bool errorBit;
  TelegramType telegramType;
  FunctionCode functionCode;
};

struct ObjectAddress {
  uint8_t channelNumber;
  uint8_t boxNumber;
  uint8_t objectNumber;
  uint16_t recordNumber;
  uint8_t segmentNumber;
  bool hasRecord;
  bool hasSegment;
};

struct MappingEntry {
  FieldId fieldId;
  FieldClass fieldClass;
  const char* label;
  uint8_t channelNumber;
  uint8_t boxNumber;
  uint8_t objectNumber;
  StructureConfidence structureConfidence;
  BehaviorConfidence behaviorConfidence;
  bool isSpecOnly;
  bool isPdfXlsOnly;
  bool allowDecode;
  bool allowReadPoll;
  bool allowWrite;
  bool allowCacheReply;
  bool allowPollSuppress;
};

struct NormalizedRequest {
  bool valid;
  RequestOrigin origin;
  bool tcpWrappedCan;
  Frame frame;
  CanIdParts canId;
  AdminParts admin;
  ObjectAddress objectAddress;
  FieldId fieldId;
  const MappingEntry* mapping;
  uint8_t tcpNode;
  uint8_t receivedChecksum;
  bool checksumChecked;
  bool checksumValid;
};

static const uint8_t kMaxFieldValueBytes = 32;
static const uint8_t kMaxSegmentCount = 16;
static const uint8_t kDiscoveryRecordCount = 5;
static const uint8_t kDiscoveryRecordLength = 64;
static const uint16_t kDiscoveryReplyLength =
    static_cast<uint16_t>(4 + kDiscoveryRecordCount * kDiscoveryRecordLength);

bool isReadFunction(FunctionCode functionCode);
bool isWriteFunction(FunctionCode functionCode);
bool isSegmentedType(TelegramType telegramType);
uint8_t fieldIndex(FieldId fieldId);
const char* fieldIdName(FieldId fieldId);

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_DBCAN_MAPPING_H
