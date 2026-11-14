#include "DbCanWebServer.h"

#if __has_include(<Ethernet.h>)
#include <Ethernet.h>
#define DBCAN_HAS_ARDUINO_ETHERNET 1
#endif

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace dbcan {

#ifdef DBCAN_HAS_ARDUINO_ETHERNET

namespace {

static const size_t kMaxRequestBytes = 2304;
static const size_t kMaxHeaderBytes = 512;
static const size_t kMaxConfigBodyBytes = 1536;
static const size_t kMaxActionBodyBytes = 128;
static const size_t kMaxStatusJsonBytes = 4096;
static const size_t kMaxBytesPerLoop = 192;
static const uint32_t kClientIdleTimeoutMs = 1500;

const char* commandName(ActionCommandType command) {
  switch (command) {
    case ActionCommandType::None:
      return "none";
    case ActionCommandType::PowerOn:
      return "power_on";
    case ActionCommandType::PowerOff:
      return "power_off";
    case ActionCommandType::MuteAOn:
      return "mute_a_on";
    case ActionCommandType::MuteAOff:
      return "mute_a_off";
    case ActionCommandType::MuteBOn:
      return "mute_b_on";
    case ActionCommandType::MuteBOff:
      return "mute_b_off";
    case ActionCommandType::DelayAOn:
      return "delay_a_on";
    case ActionCommandType::DelayAOff:
      return "delay_a_off";
    case ActionCommandType::DelayBOn:
      return "delay_b_on";
    case ActionCommandType::DelayBOff:
      return "delay_b_off";
    case ActionCommandType::GainASet:
      return "gain_a";
    case ActionCommandType::GainBSet:
      return "gain_b";
    case ActionCommandType::PresetLoad:
      return "preset_load";
    case ActionCommandType::PresetBackupRestore:
      return "preset_backup_restore";
    default:
      return "none";
  }
}

bool tryParseCommandName(const char* text, ActionCommandType& command) {
  if (text == nullptr) {
    return false;
  }
  if (strcmp(text, "none") == 0) {
    command = ActionCommandType::None;
    return true;
  }
  if (strcmp(text, "power_on") == 0) {
    command = ActionCommandType::PowerOn;
    return true;
  }
  if (strcmp(text, "power_off") == 0) {
    command = ActionCommandType::PowerOff;
    return true;
  }
  if (strcmp(text, "mute_a_on") == 0) {
    command = ActionCommandType::MuteAOn;
    return true;
  }
  if (strcmp(text, "mute_a_off") == 0) {
    command = ActionCommandType::MuteAOff;
    return true;
  }
  if (strcmp(text, "mute_b_on") == 0) {
    command = ActionCommandType::MuteBOn;
    return true;
  }
  if (strcmp(text, "mute_b_off") == 0) {
    command = ActionCommandType::MuteBOff;
    return true;
  }
  if (strcmp(text, "delay_a_on") == 0) {
    command = ActionCommandType::DelayAOn;
    return true;
  }
  if (strcmp(text, "delay_a_off") == 0) {
    command = ActionCommandType::DelayAOff;
    return true;
  }
  if (strcmp(text, "delay_b_on") == 0) {
    command = ActionCommandType::DelayBOn;
    return true;
  }
  if (strcmp(text, "delay_b_off") == 0) {
    command = ActionCommandType::DelayBOff;
    return true;
  }
  if (strcmp(text, "gain_a") == 0 || strcmp(text, "gain_a_set") == 0) {
    command = ActionCommandType::GainASet;
    return true;
  }
  if (strcmp(text, "gain_b") == 0 || strcmp(text, "gain_b_set") == 0) {
    command = ActionCommandType::GainBSet;
    return true;
  }
  if (strcmp(text, "preset_load") == 0) {
    command = ActionCommandType::PresetLoad;
    return true;
  }
  if (strcmp(text, "preset_backup_restore") == 0) {
    command = ActionCommandType::PresetBackupRestore;
    return true;
  }
  return false;
}

const char* serviceCommandName(ServiceCommand command) {
  switch (command) {
    case ServiceCommand::SaveConfig:
      return "save_config";
    case ServiceCommand::ClearErrors:
      return "clear_errors";
    case ServiceCommand::ResetPeaks:
      return "reset_peaks";
    case ServiceCommand::FactoryDefaults:
      return "factory_defaults";
    case ServiceCommand::Restart:
      return "restart";
    case ServiceCommand::None:
    default:
      return "none";
  }
}

ServiceCommand parseServiceCommand(const char* text) {
  if (text == nullptr) {
    return ServiceCommand::None;
  }
  if (strcmp(text, "save_config") == 0) {
    return ServiceCommand::SaveConfig;
  }
  if (strcmp(text, "clear_errors") == 0) {
    return ServiceCommand::ClearErrors;
  }
  if (strcmp(text, "reset_peaks") == 0) {
    return ServiceCommand::ResetPeaks;
  }
  if (strcmp(text, "factory_defaults") == 0) {
    return ServiceCommand::FactoryDefaults;
  }
  if (strcmp(text, "restart") == 0) {
    return ServiceCommand::Restart;
  }
  return ServiceCommand::None;
}

void copyText(char* dest, size_t destSize, const char* source) {
  if (dest == nullptr || destSize == 0) {
    return;
  }
  if (source == nullptr) {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, source, destSize - 1);
  dest[destSize - 1] = '\0';
}

int hexNibble(char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'a' && value <= 'f') {
    return value - 'a' + 10;
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  return -1;
}

bool decodeUrlComponent(const char* source, size_t sourceLen, char* dest, size_t destSize) {
  if (dest == nullptr || destSize == 0) {
    return false;
  }

  size_t out = 0;
  for (size_t index = 0; index < sourceLen; ++index) {
    char value = source[index];
    if (value == '+') {
      value = ' ';
    } else if (value == '%' && index + 2 < sourceLen) {
      const int high = hexNibble(source[index + 1]);
      const int low = hexNibble(source[index + 2]);
      if (high >= 0 && low >= 0) {
        value = static_cast<char>((high << 4) | low);
        index += 2;
      }
    }

    if (out + 1 >= destSize) {
      dest[0] = '\0';
      return false;
    }
    dest[out++] = value;
  }

  dest[out] = '\0';
  return true;
}

bool findFormValue(const char* body, size_t bodyLen, const char* key, char* out, size_t outSize) {
  if (body == nullptr || key == nullptr || out == nullptr || outSize == 0) {
    return false;
  }

  const size_t keyLen = strlen(key);
  size_t index = 0;
  while (index < bodyLen) {
    size_t pairEnd = index;
    while (pairEnd < bodyLen && body[pairEnd] != '&') {
      ++pairEnd;
    }

    size_t equals = index;
    while (equals < pairEnd && body[equals] != '=') {
      ++equals;
    }

    if (equals > index && equals - index == keyLen && strncmp(body + index, key, keyLen) == 0) {
      const size_t valueStart = equals < pairEnd ? equals + 1 : pairEnd;
      return decodeUrlComponent(body + valueStart, pairEnd - valueStart, out, outSize);
    }

    index = pairEnd + 1;
  }

  return false;
}

bool formHasKey(const char* body, size_t bodyLen, const char* key) {
  char value[4] = {};
  return findFormValue(body, bodyLen, key, value, sizeof(value));
}

bool parseUnsignedValue(const char* text, uint32_t& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseSignedValue(const char* text, int32_t& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  value = static_cast<int32_t>(parsed);
  return true;
}

bool parseIpAddress(const char* text, IPv4AddressConfig& ip) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  unsigned int parts[4] = {0, 0, 0, 0};
  if (sscanf(text, "%u.%u.%u.%u", &parts[0], &parts[1], &parts[2], &parts[3]) != 4) {
    return false;
  }

  for (uint8_t index = 0; index < 4; ++index) {
    if (parts[index] > 255U) {
      return false;
    }
    ip.bytes[index] = static_cast<uint8_t>(parts[index]);
  }

  return true;
}

void formatIp(char* out, size_t outSize, const IPv4AddressConfig& ip) {
  snprintf(out, outSize, "%u.%u.%u.%u", ip.bytes[0], ip.bytes[1], ip.bytes[2], ip.bytes[3]);
}

class JsonBuilder {
 public:
  JsonBuilder(char* out, size_t capacity) : out_(out), capacity_(capacity), length_(0), ok_(true) {
    if (out_ != nullptr && capacity_ > 0) {
      out_[0] = '\0';
    } else {
      ok_ = false;
    }
  }

  bool append(const char* text) {
    if (!ok_ || text == nullptr) {
      return false;
    }

    const size_t textLen = strlen(text);
    if (length_ + textLen + 1 > capacity_) {
      ok_ = false;
      return false;
    }

    memcpy(out_ + length_, text, textLen);
    length_ += textLen;
    out_[length_] = '\0';
    return true;
  }

  bool appendChar(char value) {
    if (!ok_ || length_ + 2 > capacity_) {
      ok_ = false;
      return false;
    }
    out_[length_++] = value;
    out_[length_] = '\0';
    return true;
  }

  bool appendFormat(const char* format, ...) {
    if (!ok_) {
      return false;
    }

    va_list args;
    va_start(args, format);
    const int written = vsnprintf(out_ + length_, capacity_ - length_, format, args);
    va_end(args);

    if (written < 0 || length_ + static_cast<size_t>(written) >= capacity_) {
      ok_ = false;
      return false;
    }

    length_ += static_cast<size_t>(written);
    return true;
  }

  bool appendEscaped(const char* text) {
    if (text == nullptr) {
      return append("");
    }

    for (const char* p = text; *p != '\0'; ++p) {
      switch (*p) {
        case '\\':
          if (!append("\\\\")) {
            return false;
          }
          break;
        case '"':
          if (!append("\\\"")) {
            return false;
          }
          break;
        case '\n':
          if (!append("\\n")) {
            return false;
          }
          break;
        case '\r':
          if (!append("\\r")) {
            return false;
          }
          break;
        case '\t':
          if (!append("\\t")) {
            return false;
          }
          break;
        default:
          if (!appendChar(*p)) {
            return false;
          }
          break;
      }
    }
    return true;
  }

  size_t length() const {
    return length_;
  }

  bool ok() const {
    return ok_;
  }

 private:
  char* out_;
  size_t capacity_;
  size_t length_;
  bool ok_;
};

static const char kIndexPage[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CueSmart-CueCan-Config</title><style>
body{margin:0;background:#202124;color:#e7eaed;font:13px Arial,Helvetica,sans-serif}
main{max-width:1160px;margin:0 auto;padding:10px}
h1{font-size:18px;margin:4px 0 10px;font-weight:normal;color:#fff}
h2{font-size:14px;margin:0 0 8px;padding:5px 6px;background:#2d3136;color:#fff}
section{background:#262a2f;border:1px solid #4b525a;margin:0 0 10px;padding:6px}
.cols{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.grid{display:grid;grid-template-columns:180px 1fr;gap:5px 8px;align-items:center}
label{color:#b9c0c7}
input,select,button{font:inherit}
input,select{width:100%;box-sizing:border-box;background:#17191c;color:#eef2f6;border:1px solid #58606a;padding:4px}
input[type=checkbox]{width:auto}
button{background:#4c5967;color:#fff;border:1px solid #788391;padding:5px 10px;margin:2px 4px 2px 0}
table{width:100%;border-collapse:collapse}
th,td{border:1px solid #505761;padding:4px 6px;text-align:left;vertical-align:top}
th{background:#30353a}
.muted{color:#9ea6af}.small{font-size:12px}
.status{display:grid;grid-template-columns:1fr 1fr;gap:4px 14px}
.indicator{display:flex;align-items:center;gap:7px;min-height:20px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#b51d1d}
.dot.ok{background:#1e9b38}
.toolbar,.svc{display:flex;flex-wrap:wrap;align-items:center;gap:6px}
#message{min-height:16px;color:#cfe2ff}
.queue{font-size:12px;color:#9ea6af;margin-top:6px}
@media (max-width:820px){.cols{grid-template-columns:1fr}.grid{grid-template-columns:1fr}}
</style></head><body><main>
<h1>CueSmart-CueCan-Config</h1>
<div class="toolbar"><div id="message" class="small"></div></div>
<div class="cols">
<section><h2>Device Info</h2><div class="grid">
<label>Device name</label><div id="device_name"></div>
<label>Host name</label><input id="host_name" maxlength="31">
<label>Hardware</label><div id="hardware_version"></div>
<label>Firmware</label><div id="firmware_version"></div>
<label>Schema</label><div id="schema_version"></div>
</div></section>
<section><h2>Online Error Status</h2><div id="errors" class="status"></div><div id="queues" class="queue"></div></section>
</div>
<div class="cols">
<section><h2>LAN Parameters</h2><div class="grid">
<label>DHCP enabled</label><input id="dhcp_enabled" type="checkbox">
<label>IP address</label><input id="ip_address">
<label>Subnet mask</label><input id="subnet_mask">
<label>Gateway</label><input id="gateway">
<label>DNS</label><input id="dns">
</div></section>
<section><h2>Gateway Parameters</h2><div class="grid">
<label>CAN bitrate</label><input id="can_bitrate" type="number">
<label>Device mode</label><input id="device_mode" readonly>
<label>Max managed amps</label><input id="max_amps" type="number">
<label>Poll fast ms</label><input id="poll_fast" type="number">
<label>Poll slow ms</label><input id="poll_slow" type="number">
<label>Action queue length</label><input id="action_queue_len" type="number">
<label>CAN RX queue length</label><input id="can_rx_queue_len" type="number">
<label>CAN TX queue length</label><input id="can_tx_queue_len" type="number">
<label>Ethernet RX queue length</label><input id="eth_rx_queue_len" type="number">
<label>Ethernet TX queue length</label><input id="eth_tx_queue_len" type="number">
</div></section>
</div>
<section><h2>Web Test Actions</h2><div class="toolbar">
<button type="button" data-action="0">Trigger 1</button>
<button type="button" data-action="1">Trigger 2</button>
<button type="button" data-action="2">Trigger 3</button>
<button type="button" data-action="3">Trigger 4</button>
<span class="muted small" id="trigger_hint"></span>
</div></section>
<section><h2>Action Configuration</h2><table><thead><tr><th>Action</th><th>Enabled</th><th>Name</th><th>Amp</th><th>Command</th><th>Value</th><th>Delay ms</th></tr></thead><tbody id="actions_body"></tbody></table></section>
<section><h2>GPI Configuration</h2><table><thead><tr><th>GPI</th><th>Enabled</th><th>GPIO</th><th>Active low</th><th>Debounce ms</th><th>Action</th><th>Level</th><th>Triggers</th></tr></thead><tbody id="gpis_body"></tbody></table></section>
<section><h2>Footer / Service Actions</h2><div class="svc">
<button type="button" id="save_btn">Save config</button>
<button type="button" data-service="save_config">Queue save</button>
<button type="button" data-service="clear_errors">Clear errors</button>
<button type="button" data-service="reset_peaks">Reset peaks</button>
<button type="button" data-service="factory_defaults">Factory defaults</button>
<button type="button" data-service="restart">Restart device</button>
</div></section>
</main><script>
const commandOptions=['none','power_on','power_off','mute_a_on','mute_a_off','mute_b_on','mute_b_off','preset_load','preset_backup_restore','gain_a','gain_b','delay_a_on','delay_a_off','delay_b_on','delay_b_off'];
const errorLabels=[['can_rx_overflow','CAN RX buffer overflow'],['can_tx_overflow','CAN TX buffer overflow'],['can_timeout','CAN transmission timeout'],['can_error_counter_overflow','CAN error counter overflow'],['can_bus_off','CAN bus off error'],['eth_rx_overflow','Ethernet RX buffer overflow'],['eth_tx_overflow','Ethernet TX buffer overflow'],['eth_link_down','Ethernet connection lost'],['action_queue_overflow','Internal action queue overflow'],['config_error','Config load/save error']];
let formDirty=false;
function el(id){return document.getElementById(id);}
function setMessage(text,bad){const n=el('message');n.textContent=text||'';n.style.color=bad?'#ffb4ab':'#cfe2ff';}
function esc(text){return String(text??'').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));}
function commandSelect(id,value){let html='<select id="'+id+'">';for(const cmd of commandOptions){html+='<option value="'+cmd+'"'+(cmd===value?' selected':'')+'>'+cmd+'</option>';}return html+'</select>';}
function queueSummary(q){return 'Action '+q.action.current+'/'+q.action.capacity+' peak '+q.action.peak+' | CAN RX '+q.can_rx.current+'/'+q.can_rx.capacity+' | CAN TX '+q.can_tx.current+'/'+q.can_tx.capacity+' | ETH RX '+q.eth_rx.current+'/'+q.eth_rx.capacity+' | ETH TX '+q.eth_tx.current+'/'+q.eth_tx.capacity;}
function markDirty(){formDirty=true;}
async function postForm(path,params){const res=await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},body:params.toString()});let data={ok:false};try{data=await res.json();}catch(_e){}if(!res.ok&&data.ok!==true&&!data.message){data.message='HTTP '+res.status;}return data;}
function renderErrors(errors){el('errors').innerHTML=errorLabels.map(([key,label])=>'<div class="indicator"><span class="dot '+(errors[key]?'':'ok')+'"></span><span>'+label+'</span></div>').join('');}
function renderTables(status){
  el('actions_body').innerHTML=status.actions.map((action,index)=>'<tr><td>'+index+'</td><td><input id="action_'+index+'_enabled" type="checkbox" '+(action.enabled?'checked':'')+'></td><td><input id="action_'+index+'_name" maxlength="31" value="'+esc(action.name)+'"></td><td><input id="action_'+index+'_amp" type="number" value="'+action.amp+'"></td><td>'+commandSelect('action_'+index+'_command',action.command)+'</td><td><input id="action_'+index+'_value" type="number" value="'+action.value+'"></td><td><input id="action_'+index+'_delay_ms" type="number" value="'+action.delay_ms+'"></td></tr>').join('');
  el('gpis_body').innerHTML=status.gpis.map((gpi,index)=>'<tr><td>'+(index+1)+'</td><td><input id="gpi_'+index+'_enabled" type="checkbox" '+(gpi.enabled?'checked':'')+'></td><td><input id="gpi_'+index+'_gpio" type="number" value="'+gpi.gpio+'"></td><td><input id="gpi_'+index+'_active_low" type="checkbox" '+(gpi.active_low?'checked':'')+'></td><td><input id="gpi_'+index+'_debounce_ms" type="number" value="'+gpi.debounce_ms+'"></td><td><input id="gpi_'+index+'_action_index" type="number" value="'+gpi.action_index+'"></td><td>'+gpi.level+'</td><td>'+gpi.triggers+'</td></tr>').join('');
  document.querySelectorAll('input,select').forEach(node=>node.oninput=markDirty);
}
function renderConfig(status){
  el('device_name').textContent=status.device.name;
  el('host_name').value=status.device.host_name;
  el('hardware_version').textContent=status.device.hardware;
  el('firmware_version').textContent=status.device.firmware;
  el('schema_version').textContent=status.device.schema;
  el('dhcp_enabled').checked=status.lan.dhcp_enabled;
  el('ip_address').value=status.lan.ip;
  el('subnet_mask').value=status.lan.subnet;
  el('gateway').value=status.lan.gateway;
  el('dns').value=status.lan.dns;
  el('can_bitrate').value=status.gateway.can_bitrate;
  el('device_mode').value=status.gateway.device_mode;
  el('max_amps').value=status.gateway.max_amps;
  el('poll_fast').value=status.gateway.poll_fast_ms;
  el('poll_slow').value=status.gateway.poll_slow_ms;
  el('action_queue_len').value=status.gateway.action_queue_len;
  el('can_rx_queue_len').value=status.gateway.can_rx_queue_len;
  el('can_tx_queue_len').value=status.gateway.can_tx_queue_len;
  el('eth_rx_queue_len').value=status.gateway.eth_rx_queue_len;
  el('eth_tx_queue_len').value=status.gateway.eth_tx_queue_len;
  renderTables(status);
  el('trigger_hint').textContent=status.actions.map((a,i)=>'Trigger '+(i+1)+': '+a.name).join(' | ');
}
function renderRuntime(status){
  renderErrors(status.errors);
  el('queues').textContent=queueSummary(status.queues);
}
function gatherConfig(){
  const p=new URLSearchParams();
  p.set('host_name',el('host_name').value);
  if(el('dhcp_enabled').checked){p.set('dhcp_enabled','1');}
  p.set('ip',el('ip_address').value);
  p.set('subnet',el('subnet_mask').value);
  p.set('gateway',el('gateway').value);
  p.set('dns',el('dns').value);
  p.set('can_bitrate',el('can_bitrate').value);
  p.set('max_amps',el('max_amps').value);
  p.set('poll_fast',el('poll_fast').value);
  p.set('poll_slow',el('poll_slow').value);
  p.set('action_queue_len',el('action_queue_len').value);
  p.set('can_rx_queue_len',el('can_rx_queue_len').value);
  p.set('can_tx_queue_len',el('can_tx_queue_len').value);
  p.set('eth_rx_queue_len',el('eth_rx_queue_len').value);
  p.set('eth_tx_queue_len',el('eth_tx_queue_len').value);
  for(let i=0;i<4;i++){
    if(el('action_'+i+'_enabled').checked){p.set('action_'+i+'_enabled','1');}
    p.set('action_'+i+'_name',el('action_'+i+'_name').value);
    p.set('action_'+i+'_amp',el('action_'+i+'_amp').value);
    p.set('action_'+i+'_command',el('action_'+i+'_command').value);
    p.set('action_'+i+'_value',el('action_'+i+'_value').value);
    p.set('action_'+i+'_delay_ms',el('action_'+i+'_delay_ms').value);
    if(el('gpi_'+i+'_enabled').checked){p.set('gpi_'+i+'_enabled','1');}
    p.set('gpi_'+i+'_gpio',el('gpi_'+i+'_gpio').value);
    if(el('gpi_'+i+'_active_low').checked){p.set('gpi_'+i+'_active_low','1');}
    p.set('gpi_'+i+'_debounce_ms',el('gpi_'+i+'_debounce_ms').value);
    p.set('gpi_'+i+'_action_index',el('gpi_'+i+'_action_index').value);
  }
  return p;
}
async function loadStatus(){
  try{
    const res=await fetch('/api/status');
    const status=await res.json();
    if(!formDirty){renderConfig(status);}
    renderRuntime(status);
  }catch(_e){
    setMessage('Status poll failed',true);
  }
}
async function saveConfig(){
  const result=await postForm('/api/config',gatherConfig());
  setMessage(result.message|| (result.ok?'Config queued':'Config update failed'),!result.ok);
  if(result.ok){formDirty=false;loadStatus();}
}
async function triggerAction(index){
  const p=new URLSearchParams();p.set('action',String(index));
  const result=await postForm('/api/action',p);
  setMessage(result.message|| (result.ok?'Action queued':'Action queue failed'),!result.ok);
}
async function sendService(command){
  const p=new URLSearchParams();p.set('command',command);
  const result=await postForm('/api/service',p);
  setMessage(result.message|| (result.ok?'Service queued':'Service queue failed'),!result.ok);
}
window.addEventListener('load',()=>{
  document.querySelectorAll('[data-action]').forEach(btn=>btn.onclick=()=>triggerAction(Number(btn.dataset.action)));
  document.querySelectorAll('[data-service]').forEach(btn=>btn.onclick=()=>sendService(btn.dataset.service));
  el('save_btn').onclick=saveConfig;
  loadStatus();
  setInterval(loadStatus,3000);
});
</script></body></html>
)HTML";

}  // namespace

struct DbCanWebServer::RequestState {
  EthernetClient client;
  bool active;
  bool headerParsed;
  size_t length;
  size_t headerEnd;
  size_t contentLength;
  char method[8];
  char path[32];
  char buffer[kMaxRequestBytes + 1];
  uint32_t lastActivityMs;
};

DbCanWebServer::DbCanWebServer(ConfigManager& configManager,
                               ActionQueue& actionQueue,
                               EthernetServer& server)
    : configManager_(configManager),
      actionQueue_(actionQueue),
      server_(server),
      request_(new (std::nothrow) RequestState()) {
  lastError_[0] = '\0';
  if (request_ != nullptr) {
    resetRequest();
  }
}

DbCanWebServer::~DbCanWebServer() {
  delete request_;
}

bool DbCanWebServer::begin() {
  if (request_ == nullptr) {
    setError("no_request_state");
    return false;
  }
  server_.begin();
  resetRequest();
  return true;
}

void DbCanWebServer::loop() {
  if (request_ == nullptr) {
    return;
  }

  if (!request_->active) {
    acceptClient();
  }

  if (!request_->active) {
    return;
  }

  readClientData();

  if (!request_->active) {
    return;
  }

  if (request_->headerParsed && bodyComplete()) {
    dispatchRequest();
    closeClient();
    return;
  }

  if ((!request_->client.connected() && request_->client.available() == 0) ||
      static_cast<int32_t>(millis() - request_->lastActivityMs) > static_cast<int32_t>(kClientIdleTimeoutMs)) {
    closeClient();
  }
}

size_t DbCanWebServer::buildStatusJson(char* out, size_t capacity) const {
  JsonBuilder json(out, capacity);
  char ip[16];
  char subnet[16];
  char gateway[16];
  char dns[16];

  const AppConfig& config = configManager_.config();
  const RuntimeStatus& status = configManager_.runtimeStatus();

  formatIp(ip, sizeof(ip), config.lan.ipAddress);
  formatIp(subnet, sizeof(subnet), config.lan.subnetMask);
  formatIp(gateway, sizeof(gateway), config.lan.gateway);
  formatIp(dns, sizeof(dns), config.lan.dns);

  json.append("{\"device\":{\"name\":\"");
  json.appendEscaped(config.device.deviceName);
  json.append("\",\"host_name\":\"");
  json.appendEscaped(config.device.hostName);
  json.append("\",\"hardware\":\"");
  json.appendEscaped(config.device.hardwareVersion);
  json.append("\",\"firmware\":\"");
  json.appendEscaped(config.device.firmwareVersion);
  json.appendFormat("\",\"schema\":%u},", static_cast<unsigned>(config.schemaVersion));

  json.append("\"lan\":{\"dhcp_enabled\":");
  json.append(config.lan.dhcpEnabled ? "true" : "false");
  json.appendFormat(",\"ip\":\"%s\",\"subnet\":\"%s\",\"gateway\":\"%s\",\"dns\":\"%s\"},",
                    ip,
                    subnet,
                    gateway,
                    dns);

  json.append("\"gateway\":{");
  json.appendFormat("\"device_mode\":\"%s\",", config.gateway.deviceMode == DeviceMode::Controller ? "controller"
                                                  : config.gateway.deviceMode == DeviceMode::ReservedR70Like
                                                        ? "reserved_r70_like"
                                                        : "gateway");
  json.appendFormat("\"can_bitrate\":%lu,", static_cast<unsigned long>(config.gateway.canBitrate));
  json.appendFormat("\"max_amps\":%u,", static_cast<unsigned>(config.gateway.maxManagedAmps));
  json.appendFormat("\"poll_fast_ms\":%u,", static_cast<unsigned>(config.gateway.pollIntervalFastMs));
  json.appendFormat("\"poll_slow_ms\":%u,", static_cast<unsigned>(config.gateway.pollIntervalSlowMs));
  json.appendFormat("\"action_queue_len\":%u,", static_cast<unsigned>(config.gateway.actionQueueLength));
  json.appendFormat("\"can_rx_queue_len\":%u,", static_cast<unsigned>(config.gateway.canRxQueueLength));
  json.appendFormat("\"can_tx_queue_len\":%u,", static_cast<unsigned>(config.gateway.canTxQueueLength));
  json.appendFormat("\"eth_rx_queue_len\":%u,", static_cast<unsigned>(config.gateway.ethRxQueueLength));
  json.appendFormat("\"eth_tx_queue_len\":%u},", static_cast<unsigned>(config.gateway.ethTxQueueLength));

  json.append("\"errors\":{");
  json.appendFormat("\"can_rx_overflow\":%s,", status.errors.canRxOverflow ? "true" : "false");
  json.appendFormat("\"can_tx_overflow\":%s,", status.errors.canTxOverflow ? "true" : "false");
  json.appendFormat("\"can_timeout\":%s,", status.errors.canTimeout ? "true" : "false");
  json.appendFormat("\"can_error_counter_overflow\":%s,", status.errors.canErrorCounterOverflow ? "true" : "false");
  json.appendFormat("\"can_bus_off\":%s,", status.errors.canBusOff ? "true" : "false");
  json.appendFormat("\"eth_rx_overflow\":%s,", status.errors.ethRxOverflow ? "true" : "false");
  json.appendFormat("\"eth_tx_overflow\":%s,", status.errors.ethTxOverflow ? "true" : "false");
  json.appendFormat("\"eth_link_down\":%s,", status.errors.ethLinkDown ? "true" : "false");
  json.appendFormat("\"action_queue_overflow\":%s,", status.errors.actionQueueOverflow ? "true" : "false");
  json.appendFormat("\"config_error\":%s},", status.errors.configError ? "true" : "false");

  json.append("\"queues\":{");
  json.appendFormat("\"can_rx\":{\"current\":%u,\"peak\":%u,\"capacity\":%u},",
                    static_cast<unsigned>(status.canRxQueue.currentFill),
                    static_cast<unsigned>(status.canRxQueue.peakFill),
                    static_cast<unsigned>(status.canRxQueue.capacity));
  json.appendFormat("\"can_tx\":{\"current\":%u,\"peak\":%u,\"capacity\":%u},",
                    static_cast<unsigned>(status.canTxQueue.currentFill),
                    static_cast<unsigned>(status.canTxQueue.peakFill),
                    static_cast<unsigned>(status.canTxQueue.capacity));
  json.appendFormat("\"eth_rx\":{\"current\":%u,\"peak\":%u,\"capacity\":%u},",
                    static_cast<unsigned>(status.ethRxQueue.currentFill),
                    static_cast<unsigned>(status.ethRxQueue.peakFill),
                    static_cast<unsigned>(status.ethRxQueue.capacity));
  json.appendFormat("\"eth_tx\":{\"current\":%u,\"peak\":%u,\"capacity\":%u},",
                    static_cast<unsigned>(status.ethTxQueue.currentFill),
                    static_cast<unsigned>(status.ethTxQueue.peakFill),
                    static_cast<unsigned>(status.ethTxQueue.capacity));
  json.appendFormat("\"action\":{\"current\":%u,\"peak\":%u,\"capacity\":%u}},",
                    static_cast<unsigned>(status.actionQueue.currentFill),
                    static_cast<unsigned>(status.actionQueue.peakFill),
                    static_cast<unsigned>(status.actionQueue.capacity));

  json.append("\"actions\":[");
  for (uint8_t index = 0; index < kActionCount; ++index) {
    const ActionConfig& action = config.actions[index];
    if (index > 0) {
      json.append(",");
    }
    json.append("{\"index\":");
    json.appendFormat("%u", static_cast<unsigned>(index));
    json.append(",\"enabled\":");
    json.append(action.enabled ? "true" : "false");
    json.append(",\"name\":\"");
    json.appendEscaped(action.name);
    json.append("\",\"amp\":");
    json.appendFormat("%u", static_cast<unsigned>(action.targetAmpIndex));
    json.append(",\"command\":\"");
    json.append(commandName(action.commandType));
    json.append("\",\"value\":");
    json.appendFormat("%ld", static_cast<long>(action.optionalValue));
    json.append(",\"delay_ms\":");
    json.appendFormat("%lu}", static_cast<unsigned long>(action.delayAfterMs));
  }
  json.append("],\"gpis\":[");

  for (uint8_t index = 0; index < kGpiCount; ++index) {
    const GpiInputConfig& gpi = config.gpi[index];
    const GpiRuntimeStatus& runtime = status.gpi[index];
    if (index > 0) {
      json.append(",");
    }
    json.append("{\"index\":");
    json.appendFormat("%u", static_cast<unsigned>(index + 1));
    json.append(",\"enabled\":");
    json.append(gpi.enabled ? "true" : "false");
    json.append(",\"gpio\":");
    json.appendFormat("%u", static_cast<unsigned>(gpi.gpioPin));
    json.append(",\"active_low\":");
    json.append(gpi.activeLow ? "true" : "false");
    json.append(",\"debounce_ms\":");
    json.appendFormat("%u", static_cast<unsigned>(gpi.debounceMs));
    json.append(",\"action_index\":");
    json.appendFormat("%u", static_cast<unsigned>(gpi.assignedActionIndex));
    json.append(",\"level\":\"");
    json.append(runtime.currentLevel ? "high" : "low");
    json.append("\",\"triggers\":");
    json.appendFormat("%lu}", static_cast<unsigned long>(runtime.triggerCount));
  }

  json.append("]}");
  return json.ok() ? json.length() : 0;
}

bool DbCanWebServer::handleConfigUpdate(const char* body, size_t length) {
  if (body == nullptr || length == 0 || length > kMaxConfigBodyBytes) {
    setError("config_body_invalid");
    return false;
  }

  AppConfig next = configManager_.config();
  char text[64];
  uint32_t unsignedValue = 0;
  int32_t signedValue = 0;

  if (findFormValue(body, length, "host_name", text, sizeof(next.device.hostName))) {
    copyText(next.device.hostName, sizeof(next.device.hostName), text);
  }
  next.lan.dhcpEnabled = formHasKey(body, length, "dhcp_enabled");
  if (findFormValue(body, length, "ip", text, sizeof(text)) && !parseIpAddress(text, next.lan.ipAddress)) {
    setError("invalid_ip");
    return false;
  }
  if (findFormValue(body, length, "subnet", text, sizeof(text)) && !parseIpAddress(text, next.lan.subnetMask)) {
    setError("invalid_subnet");
    return false;
  }
  if (findFormValue(body, length, "gateway", text, sizeof(text)) && !parseIpAddress(text, next.lan.gateway)) {
    setError("invalid_gateway");
    return false;
  }
  if (findFormValue(body, length, "dns", text, sizeof(text)) && !parseIpAddress(text, next.lan.dns)) {
    setError("invalid_dns");
    return false;
  }

  if (findFormValue(body, length, "can_bitrate", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue == 0) {
      setError("invalid_can_bitrate");
      return false;
    }
    next.gateway.canBitrate = unsignedValue;
  }
  if (findFormValue(body, length, "max_amps", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue > 255U || unsignedValue == 0) {
      setError("invalid_max_amps");
      return false;
    }
    next.gateway.maxManagedAmps = static_cast<uint8_t>(unsignedValue);
  }
  if (findFormValue(body, length, "poll_fast", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue > 65535UL) {
      setError("invalid_poll_fast");
      return false;
    }
    next.gateway.pollIntervalFastMs = static_cast<uint16_t>(unsignedValue);
  }
  if (findFormValue(body, length, "poll_slow", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue > 65535UL) {
      setError("invalid_poll_slow");
      return false;
    }
    next.gateway.pollIntervalSlowMs = static_cast<uint16_t>(unsignedValue);
  }
  if (findFormValue(body, length, "action_queue_len", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue == 0 || unsignedValue > 16U) {
      setError("invalid_action_queue_len");
      return false;
    }
    next.gateway.actionQueueLength = static_cast<uint8_t>(unsignedValue);
  }
  if (findFormValue(body, length, "can_rx_queue_len", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue == 0 || unsignedValue > 255U) {
      setError("invalid_can_rx_queue_len");
      return false;
    }
    next.gateway.canRxQueueLength = static_cast<uint8_t>(unsignedValue);
  }
  if (findFormValue(body, length, "can_tx_queue_len", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue == 0 || unsignedValue > 255U) {
      setError("invalid_can_tx_queue_len");
      return false;
    }
    next.gateway.canTxQueueLength = static_cast<uint8_t>(unsignedValue);
  }
  if (findFormValue(body, length, "eth_rx_queue_len", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue == 0 || unsignedValue > 255U) {
      setError("invalid_eth_rx_queue_len");
      return false;
    }
    next.gateway.ethRxQueueLength = static_cast<uint8_t>(unsignedValue);
  }
  if (findFormValue(body, length, "eth_tx_queue_len", text, sizeof(text))) {
    if (!parseUnsignedValue(text, unsignedValue) || unsignedValue == 0 || unsignedValue > 255U) {
      setError("invalid_eth_tx_queue_len");
      return false;
    }
    next.gateway.ethTxQueueLength = static_cast<uint8_t>(unsignedValue);
  }

  for (uint8_t index = 0; index < kActionCount; ++index) {
    char field[32];
    ActionConfig& action = next.actions[index];

    snprintf(field, sizeof(field), "action_%u_enabled", static_cast<unsigned>(index));
    action.enabled = formHasKey(body, length, field);

    snprintf(field, sizeof(field), "action_%u_name", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(action.name))) {
      copyText(action.name, sizeof(action.name), text);
    }

    snprintf(field, sizeof(field), "action_%u_amp", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(text))) {
      if (!parseUnsignedValue(text, unsignedValue) || unsignedValue > 255U) {
        setError("invalid_action_amp");
        return false;
      }
      action.targetAmpIndex = static_cast<uint8_t>(unsignedValue);
    }

    snprintf(field, sizeof(field), "action_%u_command", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(text))) {
      if (!tryParseCommandName(text, action.commandType)) {
        setError("invalid_action_command");
        return false;
      }
    }

    snprintf(field, sizeof(field), "action_%u_value", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(text))) {
      if (!parseSignedValue(text, signedValue)) {
        setError("invalid_action_value");
        return false;
      }
      action.optionalValue = signedValue;
    }

    snprintf(field, sizeof(field), "action_%u_delay_ms", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(text))) {
      if (!parseUnsignedValue(text, unsignedValue)) {
        setError("invalid_action_delay");
        return false;
      }
      action.delayAfterMs = unsignedValue;
    }
  }

  for (uint8_t index = 0; index < kGpiCount; ++index) {
    char field[32];
    GpiInputConfig& gpi = next.gpi[index];

    snprintf(field, sizeof(field), "gpi_%u_enabled", static_cast<unsigned>(index));
    gpi.enabled = formHasKey(body, length, field);

    snprintf(field, sizeof(field), "gpi_%u_gpio", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(text))) {
      if (!parseUnsignedValue(text, unsignedValue) || unsignedValue > 255U) {
        setError("invalid_gpi_gpio");
        return false;
      }
      gpi.gpioPin = static_cast<uint8_t>(unsignedValue);
    }

    snprintf(field, sizeof(field), "gpi_%u_active_low", static_cast<unsigned>(index));
    gpi.activeLow = formHasKey(body, length, field);

    snprintf(field, sizeof(field), "gpi_%u_debounce_ms", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(text))) {
      if (!parseUnsignedValue(text, unsignedValue) || unsignedValue > 65535UL) {
        setError("invalid_gpi_debounce");
        return false;
      }
      gpi.debounceMs = static_cast<uint16_t>(unsignedValue);
    }

    snprintf(field, sizeof(field), "gpi_%u_action_index", static_cast<unsigned>(index));
    if (findFormValue(body, length, field, text, sizeof(text))) {
      if (!parseUnsignedValue(text, unsignedValue) || unsignedValue >= kActionCount) {
        setError("invalid_gpi_action_index");
        return false;
      }
      gpi.assignedActionIndex = static_cast<uint8_t>(unsignedValue);
    }
  }

  // Queue the save so Preferences writes happen outside the HTTP handler.
  if (!actionQueue_.queueConfigSave(next)) {
    setError("queue_full");
    return false;
  }

  lastError_[0] = '\0';
  return true;
}

bool DbCanWebServer::handleActionTrigger(const char* body, size_t length) {
  if (body == nullptr || length == 0 || length > kMaxActionBodyBytes) {
    setError("action_body_invalid");
    return false;
  }

  char text[16];
  uint32_t actionIndex = 0;
  if (!findFormValue(body, length, "action", text, sizeof(text)) || !parseUnsignedValue(text, actionIndex) ||
      actionIndex >= kActionCount) {
    setError("invalid_action");
    return false;
  }

  // Queue action execution so the web handler never runs dbCAN work inline.
  if (!actionQueue_.queueAction(static_cast<uint8_t>(actionIndex))) {
    setError("queue_full");
    return false;
  }

  lastError_[0] = '\0';
  return true;
}

bool DbCanWebServer::handleServiceCommand(const char* body, size_t length) {
  if (body == nullptr || length == 0 || length > kMaxActionBodyBytes) {
    setError("service_body_invalid");
    return false;
  }

  char text[32];
  if (!findFormValue(body, length, "command", text, sizeof(text))) {
    setError("missing_service_command");
    return false;
  }

  const ServiceCommand command = parseServiceCommand(text);
  if (command == ServiceCommand::None) {
    setError("invalid_service_command");
    return false;
  }

  // Queue service work so restart/defaults/save happen after the HTTP response returns.
  if (!actionQueue_.queueService(command)) {
    setError("queue_full");
    return false;
  }

  lastError_[0] = '\0';
  return true;
}

const char* DbCanWebServer::lastError() const {
  return lastError_;
}

void DbCanWebServer::resetRequest() {
  if (request_ == nullptr) {
    return;
  }

  request_->client.stop();
  request_->active = false;
  request_->headerParsed = false;
  request_->length = 0;
  request_->headerEnd = 0;
  request_->contentLength = 0;
  request_->method[0] = '\0';
  request_->path[0] = '\0';
  request_->buffer[0] = '\0';
  request_->lastActivityMs = millis();
}

void DbCanWebServer::acceptClient() {
  EthernetClient next = server_.accept();
  if (!next) {
    return;
  }

  resetRequest();
  request_->client = next;
  request_->active = true;
  request_->lastActivityMs = millis();
}

void DbCanWebServer::readClientData() {
  size_t processed = 0;
  while (request_->client.available() > 0 && processed < kMaxBytesPerLoop) {
    const int next = request_->client.read();
    if (next < 0) {
      break;
    }

    if (request_->length >= kMaxRequestBytes) {
      sendTextResult(413, "Request too large");
      closeClient();
      return;
    }

    request_->buffer[request_->length++] = static_cast<char>(next);
    request_->buffer[request_->length] = '\0';
    request_->lastActivityMs = millis();
    ++processed;
  }

  if (!request_->headerParsed && request_->length <= kMaxHeaderBytes) {
    if (parseRequestHead()) {
      request_->headerParsed = true;
    }
  } else if (!request_->headerParsed && request_->length > kMaxHeaderBytes) {
    sendTextResult(413, "Headers too large");
    closeClient();
  }
}

void DbCanWebServer::dispatchRequest() {
  if (strcmp(request_->method, "GET") == 0 && strcmp(request_->path, "/") == 0) {
    sendHtmlPage();
    return;
  }
  if (strcmp(request_->method, "GET") == 0 && strcmp(request_->path, "/api/status") == 0) {
    sendStatusJson();
    return;
  }

  const char* body = request_->buffer + request_->headerEnd;
  const size_t bodyLen = request_->length - request_->headerEnd;

  if (strcmp(request_->method, "POST") == 0 && strcmp(request_->path, "/api/config") == 0) {
    const bool ok = handleConfigUpdate(body, bodyLen);
    sendJsonResult(ok ? 200 : 400, ok, ok ? "config_queued" : lastError());
    return;
  }
  if (strcmp(request_->method, "POST") == 0 && strcmp(request_->path, "/api/action") == 0) {
    const bool ok = handleActionTrigger(body, bodyLen);
    sendJsonResult(ok ? 200 : 400, ok, ok ? "action_queued" : lastError());
    return;
  }
  if (strcmp(request_->method, "POST") == 0 && strcmp(request_->path, "/api/service") == 0) {
    const bool ok = handleServiceCommand(body, bodyLen);
    sendJsonResult(ok ? 200 : 400, ok, ok ? "service_queued" : lastError());
    return;
  }

  sendTextResult(404, "Not found");
}

void DbCanWebServer::sendHtmlPage() {
  const size_t length = strlen_P(kIndexPage);
  sendResponseHeader(200, "OK", "text/html; charset=utf-8", length);
  request_->client.print(FPSTR(kIndexPage));
}

void DbCanWebServer::sendStatusJson() {
  char json[kMaxStatusJsonBytes];
  const size_t length = buildStatusJson(json, sizeof(json));
  if (length == 0) {
    sendJsonResult(500, false, "status_build_failed");
    return;
  }

  sendResponseHeader(200, "OK", "application/json; charset=utf-8", length);
  request_->client.write(reinterpret_cast<const uint8_t*>(json), length);
}

void DbCanWebServer::sendJsonResult(int code, bool ok, const char* message) {
  char json[160];
  JsonBuilder builder(json, sizeof(json));
  builder.append("{\"ok\":");
  builder.append(ok ? "true" : "false");
  if (message != nullptr && *message != '\0') {
    builder.append(",\"message\":\"");
    builder.appendEscaped(message);
    builder.append("\"");
  }
  builder.append("}");

  const char* phrase = code == 200 ? "OK" : code == 400 ? "Bad Request" : "Error";
  sendResponseHeader(code, phrase, "application/json; charset=utf-8", builder.length());
  request_->client.write(reinterpret_cast<const uint8_t*>(json), builder.length());
}

void DbCanWebServer::sendTextResult(int code, const char* text) {
  const size_t length = text != nullptr ? strlen(text) : 0;
  const char* phrase = code == 200 ? "OK" : code == 404 ? "Not Found" : code == 413 ? "Payload Too Large" : "Error";
  sendResponseHeader(code, phrase, "text/plain; charset=utf-8", length);
  if (length > 0) {
    request_->client.write(reinterpret_cast<const uint8_t*>(text), length);
  }
}

void DbCanWebServer::sendResponseHeader(int code,
                                        const char* phrase,
                                        const char* contentType,
                                        size_t contentLength) {
  request_->client.print(F("HTTP/1.1 "));
  request_->client.print(code);
  request_->client.print(' ');
  request_->client.println(phrase);
  request_->client.print(F("Content-Type: "));
  request_->client.println(contentType);
  request_->client.print(F("Content-Length: "));
  request_->client.println(contentLength);
  request_->client.println(F("Connection: close"));
  request_->client.println(F("Cache-Control: no-store"));
  request_->client.println();
}

void DbCanWebServer::closeClient() {
  if (request_ == nullptr) {
    return;
  }
  request_->client.stop();
  resetRequest();
}

bool DbCanWebServer::parseRequestHead() {
  const char* marker = strstr(request_->buffer, "\r\n\r\n");
  if (marker == nullptr) {
    return false;
  }

  request_->headerEnd = static_cast<size_t>((marker - request_->buffer) + 4);

  const char* lineEnd = strstr(request_->buffer, "\r\n");
  if (lineEnd == nullptr) {
    setError("bad_request_line");
    sendJsonResult(400, false, lastError());
    closeClient();
    return false;
  }

  char requestLine[96];
  const size_t requestLineLength = static_cast<size_t>(lineEnd - request_->buffer);
  if (requestLineLength == 0 || requestLineLength >= sizeof(requestLine)) {
    setError("request_line_too_long");
    sendJsonResult(400, false, lastError());
    closeClient();
    return false;
  }
  memcpy(requestLine, request_->buffer, requestLineLength);
  requestLine[requestLineLength] = '\0';

  char version[16];
  if (sscanf(requestLine, "%7s %31s %15s", request_->method, request_->path, version) != 3) {
    setError("bad_request_line");
    sendJsonResult(400, false, lastError());
    closeClient();
    return false;
  }

  request_->contentLength = 0;
  const char* cursor = lineEnd + 2;
  while (cursor < request_->buffer + request_->headerEnd - 2) {
    const char* nextEnd = strstr(cursor, "\r\n");
    if (nextEnd == nullptr) {
      break;
    }
    const char* colon = static_cast<const char*>(memchr(cursor, ':', nextEnd - cursor));
    if (colon != nullptr) {
      const size_t nameLen = static_cast<size_t>(colon - cursor);
      if (nameLen == 14 && strncasecmp(cursor, "Content-Length", 14) == 0) {
        const char* value = colon + 1;
        while (value < nextEnd && (*value == ' ' || *value == '\t')) {
          ++value;
        }
        request_->contentLength = static_cast<size_t>(strtoul(value, nullptr, 10));
      }
    }
    cursor = nextEnd + 2;
  }

  const bool isConfigPost = strcmp(request_->method, "POST") == 0 && strcmp(request_->path, "/api/config") == 0;
  const size_t maxBody = isConfigPost ? kMaxConfigBodyBytes : kMaxActionBodyBytes;
  if (request_->contentLength > maxBody || request_->headerEnd + request_->contentLength > kMaxRequestBytes) {
    setError("request_too_large");
    sendJsonResult(413, false, lastError());
    closeClient();
    return false;
  }

  return true;
}

bool DbCanWebServer::bodyComplete() const {
  return request_->length >= request_->headerEnd + request_->contentLength;
}

void DbCanWebServer::setError(const char* message) {
  copyText(lastError_, sizeof(lastError_), message);
}

#else

DbCanWebServer::DbCanWebServer(ConfigManager& configManager,
                               ActionQueue& actionQueue,
                               EthernetServer& server)
    : configManager_(configManager), actionQueue_(actionQueue), server_(server), request_(nullptr) {
  lastError_[0] = '\0';
}

DbCanWebServer::~DbCanWebServer() {}

bool DbCanWebServer::begin() {
  setError("ethernet_not_available");
  return false;
}

void DbCanWebServer::loop() {}

size_t DbCanWebServer::buildStatusJson(char* out, size_t capacity) const {
  if (out != nullptr && capacity > 0) {
    out[0] = '\0';
  }
  return 0;
}

bool DbCanWebServer::handleConfigUpdate(const char* body, size_t length) {
  (void)body;
  (void)length;
  return false;
}

bool DbCanWebServer::handleActionTrigger(const char* body, size_t length) {
  (void)body;
  (void)length;
  return false;
}

bool DbCanWebServer::handleServiceCommand(const char* body, size_t length) {
  (void)body;
  (void)length;
  return false;
}

const char* DbCanWebServer::lastError() const {
  return lastError_;
}

void DbCanWebServer::resetRequest() {}
void DbCanWebServer::acceptClient() {}
void DbCanWebServer::readClientData() {}
void DbCanWebServer::dispatchRequest() {}
void DbCanWebServer::sendHtmlPage() {}
void DbCanWebServer::sendStatusJson() {}
void DbCanWebServer::sendJsonResult(int code, bool ok, const char* message) {
  (void)code;
  (void)ok;
  (void)message;
}
void DbCanWebServer::sendTextResult(int code, const char* text) {
  (void)code;
  (void)text;
}
void DbCanWebServer::sendResponseHeader(int code,
                                        const char* phrase,
                                        const char* contentType,
                                        size_t contentLength) {
  (void)code;
  (void)phrase;
  (void)contentType;
  (void)contentLength;
}
void DbCanWebServer::closeClient() {}
bool DbCanWebServer::parseRequestHead() { return false; }
bool DbCanWebServer::bodyComplete() const { return false; }
void DbCanWebServer::setError(const char* message) {
  if (message == nullptr) {
    lastError_[0] = '\0';
    return;
  }
  strncpy(lastError_, message, sizeof(lastError_) - 1);
  lastError_[sizeof(lastError_) - 1] = '\0';
}

#endif

}  // namespace dbcan
