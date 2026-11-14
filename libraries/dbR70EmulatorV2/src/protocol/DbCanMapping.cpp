#include "DbCanMapping.h"

namespace dbcan {
namespace r70v2 {

bool isReadFunction(FunctionCode functionCode) {
  return functionCode == FunctionCode::ReadObject;
}

bool isWriteFunction(FunctionCode functionCode) {
  return functionCode == FunctionCode::WriteObject;
}

bool isSegmentedType(TelegramType telegramType) {
  return telegramType == TelegramType::Segmented ||
         telegramType == TelegramType::FileSegmented;
}

uint8_t fieldIndex(FieldId fieldId) {
  const uint8_t index = static_cast<uint8_t>(fieldId);
  if (index >= static_cast<uint8_t>(FieldId::Count)) {
    return 0;
  }
  return index;
}

const char* fieldIdName(FieldId fieldId) {
  switch (fieldId) {
    case FieldId::fixed_serial_number:
      return "fixed_serial_number";
    case FieldId::fixed_firmware_version:
      return "fixed_firmware_version";
    case FieldId::settings_power:
      return "settings_power";
    case FieldId::settings_lock_cmd:
      return "settings_lock_cmd";
    case FieldId::settings_buzzer:
      return "settings_buzzer";
    case FieldId::settings_device_name:
      return "settings_device_name";
    case FieldId::settings_input_mode:
      return "settings_input_mode";
    case FieldId::settings_data_link_preset:
      return "settings_data_link_preset";
    case FieldId::config_filter1_a:
      return "config_filter1_a";
    case FieldId::config_filter1_b:
      return "config_filter1_b";
    case FieldId::config_filter2_a:
      return "config_filter2_a";
    case FieldId::config_filter2_b:
      return "config_filter2_b";
    case FieldId::config_filter3_a:
      return "config_filter3_a";
    case FieldId::config_filter3_b:
      return "config_filter3_b";
    case FieldId::config_cut_a:
      return "config_cut_a";
    case FieldId::config_cut_b:
      return "config_cut_b";
    case FieldId::config_hfa_a:
      return "config_hfa_a";
    case FieldId::config_hfa_b:
      return "config_hfa_b";
    case FieldId::config_delay_on_a:
      return "config_delay_on_a";
    case FieldId::config_delay_on_b:
      return "config_delay_on_b";
    case FieldId::config_mute_a:
      return "config_mute_a";
    case FieldId::config_mute_b:
      return "config_mute_b";
    case FieldId::config_gain_a:
      return "config_gain_a";
    case FieldId::config_gain_b:
      return "config_gain_b";
    case FieldId::config_delay_value_a:
      return "config_delay_value_a";
    case FieldId::config_delay_value_b:
      return "config_delay_value_b";
    case FieldId::config_delay_dim_a:
      return "config_delay_dim_a";
    case FieldId::config_delay_dim_b:
      return "config_delay_dim_b";
    case FieldId::config_input_source_a:
      return "config_input_source_a";
    case FieldId::config_input_source_b:
      return "config_input_source_b";
    case FieldId::config_eqs_enabled_a:
      return "config_eqs_enabled_a";
    case FieldId::config_eqs_enabled_b:
      return "config_eqs_enabled_b";
    case FieldId::config_speaker_id_a:
      return "config_speaker_id_a";
    case FieldId::config_speaker_id_b:
      return "config_speaker_id_b";
    case FieldId::config_name_a:
      return "config_name_a";
    case FieldId::config_name_b:
      return "config_name_b";
    case FieldId::status_pwr_ok:
      return "status_pwr_ok";
    case FieldId::status_text:
      return "status_text";
    case FieldId::status_digin_status:
      return "status_digin_status";
    case FieldId::status_hardware_type:
      return "status_hardware_type";
    case FieldId::chstatus_isp_a:
      return "chstatus_isp_a";
    case FieldId::chstatus_isp_b:
      return "chstatus_isp_b";
    case FieldId::chstatus_gr_a:
      return "chstatus_gr_a";
    case FieldId::chstatus_gr_b:
      return "chstatus_gr_b";
    case FieldId::chstatus_ovl_a:
      return "chstatus_ovl_a";
    case FieldId::chstatus_ovl_b:
      return "chstatus_ovl_b";
    case FieldId::chstatus_osp_a:
      return "chstatus_osp_a";
    case FieldId::chstatus_osp_b:
      return "chstatus_osp_b";
    case FieldId::chstatus_gr_head_a:
      return "chstatus_gr_head_a";
    case FieldId::chstatus_gr_head_b:
      return "chstatus_gr_head_b";
    case FieldId::chstatus_input_voltage_a:
      return "chstatus_input_voltage_a";
    case FieldId::chstatus_input_voltage_b:
      return "chstatus_input_voltage_b";
    case FieldId::chstatus_input_voltage_peak_a:
      return "chstatus_input_voltage_peak_a";
    case FieldId::chstatus_input_voltage_peak_b:
      return "chstatus_input_voltage_peak_b";
    case FieldId::chstatus_speaker_impedance_a:
      return "chstatus_speaker_impedance_a";
    case FieldId::chstatus_speaker_impedance_b:
      return "chstatus_speaker_impedance_b";
    case FieldId::chstatus_gr_head_peak_a:
      return "chstatus_gr_head_peak_a";
    case FieldId::chstatus_gr_head_peak_b:
      return "chstatus_gr_head_peak_b";
    case FieldId::chstatus_output_power_a:
      return "chstatus_output_power_a";
    case FieldId::chstatus_output_power_b:
      return "chstatus_output_power_b";
    case FieldId::chstatus_output_power_peak_a:
      return "chstatus_output_power_peak_a";
    case FieldId::chstatus_output_power_peak_b:
      return "chstatus_output_power_peak_b";
    case FieldId::chstatus_ms_delay_a:
      return "chstatus_ms_delay_a";
    case FieldId::chstatus_ms_delay_b:
      return "chstatus_ms_delay_b";
    case FieldId::chstatus_amp_temperature_a:
      return "chstatus_amp_temperature_a";
    case FieldId::chstatus_amp_temperature_b:
      return "chstatus_amp_temperature_b";
    case FieldId::chstatus_input_overload_a:
      return "chstatus_input_overload_a";
    case FieldId::chstatus_input_overload_b:
      return "chstatus_input_overload_b";
    case FieldId::chstatus_output_overload_a:
      return "chstatus_output_overload_a";
    case FieldId::chstatus_output_overload_b:
      return "chstatus_output_overload_b";
    case FieldId::chstatus_remhold_isp_a:
      return "chstatus_remhold_isp_a";
    case FieldId::chstatus_remhold_isp_b:
      return "chstatus_remhold_isp_b";
    case FieldId::chstatus_remhold_gr_a:
      return "chstatus_remhold_gr_a";
    case FieldId::chstatus_remhold_gr_b:
      return "chstatus_remhold_gr_b";
    case FieldId::chstatus_remhold_ovl_a:
      return "chstatus_remhold_ovl_a";
    case FieldId::chstatus_remhold_ovl_b:
      return "chstatus_remhold_ovl_b";
    case FieldId::chstatus_remhold_osp_a:
      return "chstatus_remhold_osp_a";
    case FieldId::chstatus_remhold_osp_b:
      return "chstatus_remhold_osp_b";
    case FieldId::chstatus_remhold_gr_head_a:
      return "chstatus_remhold_gr_head_a";
    case FieldId::chstatus_remhold_gr_head_b:
      return "chstatus_remhold_gr_head_b";
    case FieldId::error_general:
      return "error_general";
    case FieldId::error_number:
      return "error_number";
    case FieldId::error_text:
      return "last_error_text";
    case FieldId::cherr_channel_err_a:
      return "cherr_channel_err_a";
    case FieldId::cherr_channel_err_b:
      return "cherr_channel_err_b";
    case FieldId::cherr_err_nr_a:
      return "cherr_err_nr_a";
    case FieldId::cherr_err_nr_b:
      return "cherr_err_nr_b";
    case FieldId::cherr_error_text_a:
      return "cherr_last_error_text_a";
    case FieldId::cherr_error_text_b:
      return "cherr_last_error_text_b";
    case FieldId::cherr_en_imp_err_a:
      return "cherr_en_imp_err_a";
    case FieldId::cherr_en_imp_err_b:
      return "cherr_en_imp_err_b";
    case FieldId::cherr_en_imp_err_lf_a:
      return "cherr_en_imp_err_lf_a";
    case FieldId::cherr_en_imp_err_lf_b:
      return "cherr_en_imp_err_lf_b";
    case FieldId::cherr_en_imp_err_hf_a:
      return "cherr_en_imp_err_hf_a";
    case FieldId::cherr_en_imp_err_hf_b:
      return "cherr_en_imp_err_hf_b";
    case FieldId::cherr_psp_err_a:
      return "cherr_psp_err_a";
    case FieldId::cherr_psp_err_b:
      return "cherr_psp_err_b";
    case FieldId::eq_frequency_a:
      return "eq_frequency_a";
    case FieldId::eq_frequency_b:
      return "eq_frequency_b";
    case FieldId::eq_q_a:
      return "eq_q_a";
    case FieldId::eq_q_b:
      return "eq_q_b";
    case FieldId::eq_gain_a:
      return "eq_gain_a";
    case FieldId::eq_gain_b:
      return "eq_gain_b";
    case FieldId::eq_mode_a:
      return "eq_mode_a";
    case FieldId::eq_mode_b:
      return "eq_mode_b";
    case FieldId::loadmon_enabled_a:
      return "loadmon_enabled_a";
    case FieldId::loadmon_enabled_b:
      return "loadmon_enabled_b";
    case FieldId::Unknown:
    case FieldId::Count:
    default:
      return "unknown";
  }
}

}  // namespace r70v2
}  // namespace dbcan
