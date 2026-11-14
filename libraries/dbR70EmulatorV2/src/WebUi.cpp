#include "WebUi.h"

#include <ESP.h>
#if __has_include(<Ethernet.h>)
#include <Ethernet.h>
#define DBCAN_HAS_ARDUINO_ETHERNET 1
#endif
#include <new>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace dbcan {

class WebUiWriter {
 public:
  explicit WebUiWriter(WebServer& server) : server_(&server), client_(nullptr) {}
  explicit WebUiWriter(Print& client) : server_(nullptr), client_(&client) {}

  void send(const char* text) {
    if (server_ != nullptr) {
      server_->sendContent(text);
    } else if (client_ != nullptr) {
      client_->print(text);
    }
  }

  void send(const String& text) {
    if (server_ != nullptr) {
      server_->sendContent(text);
    } else if (client_ != nullptr) {
      client_->print(text);
    }
  }

  void send(const __FlashStringHelper* text) {
    if (server_ != nullptr) {
      server_->sendContent(String(text));
    } else if (client_ != nullptr) {
      client_->print(text);
    }
  }

 private:
  WebServer* server_;
  Print* client_;
};

class WebUiRequest {
 public:
  virtual ~WebUiRequest() {}
  virtual bool hasArg(const String& name) const = 0;
  virtual String arg(const String& name) const = 0;
};

class NativeWebUiRequest : public WebUiRequest {
 public:
  explicit NativeWebUiRequest(WebServer& server) : server_(server) {}

  bool hasArg(const String& name) const override {
    return server_.hasArg(name);
  }

  String arg(const String& name) const override {
    return server_.arg(name);
  }

 private:
  WebServer& server_;
};

class EthernetWebUiRequest : public WebUiRequest {
 public:
  String method;
  String path;

  bool hasArg(const String& name) const override {
    for (uint8_t index = 0; index < argCount_; ++index) {
      if (args_[index].name == name) {
        return true;
      }
    }
    return false;
  }

  String arg(const String& name) const override {
    for (uint8_t index = 0; index < argCount_; ++index) {
      if (args_[index].name == name) {
        return args_[index].value;
      }
    }
    return String();
  }

  bool addArg(const String& name, const String& value) {
    for (uint8_t index = 0; index < argCount_; ++index) {
      if (args_[index].name == name) {
        args_[index].value = value;
        return true;
      }
    }
    if (argCount_ >= kMaxArgs) {
      return false;
    }
    args_[argCount_].name = name;
    args_[argCount_].value = value;
    ++argCount_;
    return true;
  }

 private:
  struct Arg {
    String name;
    String value;
  };

  static const uint8_t kMaxArgs = 72;

  Arg args_[kMaxArgs];
  uint8_t argCount_ = 0;
};

namespace {

const char* onOff(bool value) {
  return value ? "on" : "off";
}

const char* checked(bool value) {
  return value ? " checked" : "";
}

const char* selected(bool value) {
  return value ? " selected" : "";
}

const char* deviceModeName(DeviceMode mode) {
  switch (mode) {
    case DeviceMode::Gateway:
      return "gateway";
    case DeviceMode::Controller:
      return "controller";
    case DeviceMode::ReservedR70Like:
      return "reserved_r70_like";
    default:
      return "unknown";
  }
}

const char* commandTypeName(ActionCommandType type) {
  switch (type) {
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
      return "gain_a_set";
    case ActionCommandType::GainBSet:
      return "gain_b_set";
    default:
      return "unknown";
  }
}

DeviceMode parseDeviceMode(const String& value, DeviceMode fallback) {
  if (value == "gateway") {
    return DeviceMode::Gateway;
  }
  if (value == "controller") {
    return DeviceMode::Controller;
  }
  if (value == "reserved_r70_like") {
    return DeviceMode::ReservedR70Like;
  }
  return fallback;
}

ActionCommandType parseCommandType(const String& value, ActionCommandType fallback) {
  if (value == "none") {
    return ActionCommandType::None;
  }
  if (value == "power_on") {
    return ActionCommandType::PowerOn;
  }
  if (value == "power_off") {
    return ActionCommandType::PowerOff;
  }
  if (value == "mute_a_on") {
    return ActionCommandType::MuteAOn;
  }
  if (value == "mute_a_off") {
    return ActionCommandType::MuteAOff;
  }
  if (value == "mute_b_on") {
    return ActionCommandType::MuteBOn;
  }
  if (value == "mute_b_off") {
    return ActionCommandType::MuteBOff;
  }
  if (value == "delay_a_on") {
    return ActionCommandType::DelayAOn;
  }
  if (value == "delay_a_off") {
    return ActionCommandType::DelayAOff;
  }
  if (value == "delay_b_on") {
    return ActionCommandType::DelayBOn;
  }
  if (value == "delay_b_off") {
    return ActionCommandType::DelayBOff;
  }
  if (value == "gain_a_set") {
    return ActionCommandType::GainASet;
  }
  if (value == "gain_b_set") {
    return ActionCommandType::GainBSet;
  }
  return fallback;
}

uint32_t parseU32(const String& value, uint32_t fallback) {
  if (value.length() == 0) {
    return fallback;
  }
  char* end = nullptr;
  const unsigned long parsed = strtoul(value.c_str(), &end, 10);
  return end != value.c_str() ? static_cast<uint32_t>(parsed) : fallback;
}

uint16_t parseU16(const String& value, uint16_t fallback) {
  const uint32_t parsed = parseU32(value, fallback);
  return parsed <= 65535UL ? static_cast<uint16_t>(parsed) : fallback;
}

uint8_t parseU8(const String& value, uint8_t fallback) {
  const uint32_t parsed = parseU32(value, fallback);
  return parsed <= 255UL ? static_cast<uint8_t>(parsed) : fallback;
}

int16_t parseI16(const String& value, int16_t fallback) {
  if (value.length() == 0) {
    return fallback;
  }
  char* end = nullptr;
  const long parsed = strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || parsed < -32768L || parsed > 32767L) {
    return fallback;
  }
  return static_cast<int16_t>(parsed);
}

void copyArg(const WebUiRequest& request, const char* name, char* dest, size_t destSize) {
  if (dest == nullptr || destSize == 0 || !request.hasArg(name)) {
    return;
  }
  String value = request.arg(name);
  value.toCharArray(dest, destSize);
  dest[destSize - 1] = '\0';
}

void parseIp(const String& value, IPv4AddressConfig& out) {
  unsigned int part[4] = {0, 0, 0, 0};
  if (sscanf(value.c_str(), "%u.%u.%u.%u", &part[0], &part[1], &part[2], &part[3]) != 4) {
    return;
  }
  for (uint8_t index = 0; index < 4; ++index) {
    if (part[index] > 255U) {
      return;
    }
  }
  for (uint8_t index = 0; index < 4; ++index) {
    out.bytes[index] = static_cast<uint8_t>(part[index]);
  }
}

String fieldName(const char* prefix, uint8_t index, const char* suffix) {
  String name(prefix);
  name += index;
  name += suffix;
  return name;
}

void sendEscaped(WebUiWriter& writer, const char* value) {
  if (value == nullptr) {
    return;
  }
  for (const char* p = value; *p != '\0'; ++p) {
    switch (*p) {
      case '&':
        writer.send(F("&amp;"));
        break;
      case '<':
        writer.send(F("&lt;"));
        break;
      case '>':
        writer.send(F("&gt;"));
        break;
      case '"':
        writer.send(F("&quot;"));
        break;
      default: {
        char text[2] = {*p, '\0'};
        writer.send(text);
        break;
      }
    }
  }
}

void sendInput(WebUiWriter& writer, const char* name, const char* value, size_t maxLen) {
  writer.send(F("<input name=\""));
  writer.send(name);
  writer.send(F("\" maxlength=\""));
  writer.send(String(maxLen - 1));
  writer.send(F("\" value=\""));
  sendEscaped(writer, value);
  writer.send(F("\">"));
}

void sendNumberInput(WebUiWriter& writer, const char* name, long value) {
  writer.send(F("<input type=\"number\" name=\""));
  writer.send(name);
  writer.send(F("\" value=\""));
  writer.send(String(value));
  writer.send(F("\">"));
}

void sendCheckbox(WebUiWriter& writer, const char* name, bool value) {
  writer.send(F("<input type=\"checkbox\" name=\""));
  writer.send(name);
  writer.send(F("\" value=\"1\""));
  writer.send(checked(value));
  writer.send(F(">"));
}

void sendIp(WebUiWriter& writer, const IPv4AddressConfig& ip) {
  char text[16];
  snprintf(text, sizeof(text), "%u.%u.%u.%u", ip.bytes[0], ip.bytes[1], ip.bytes[2], ip.bytes[3]);
  writer.send(text);
}

void sendIndicator(WebUiWriter& writer, const char* label, bool active) {
  writer.send(F("<div class=\"err\"><span class=\"led "));
  writer.send(active ? "red" : "green");
  writer.send(F("\"></span>"));
  writer.send(label);
  writer.send(F("</div>"));
}

void sendQueueRow(WebUiWriter& writer, const char* label, const QueueStatus& status) {
  writer.send(F("<tr><td>"));
  writer.send(label);
  writer.send(F("</td><td>"));
  writer.send(String(status.currentFill));
  writer.send(F("</td><td>"));
  writer.send(String(status.peakFill));
  writer.send(F("</td><td>"));
  writer.send(String(status.capacity));
  writer.send(F("</td><td>"));
  writer.send(String(status.overflowCount));
  writer.send(F("</td><td>"));
  writer.send(String(status.droppedCount));
  writer.send(F("</td></tr>"));
}

void sendDeviceModeSelect(WebUiWriter& writer, DeviceMode mode) {
  writer.send(F("<select name=\"device_mode\">"));
  writer.send(F("<option value=\"gateway\""));
  writer.send(selected(mode == DeviceMode::Gateway));
  writer.send(F(">gateway</option><option value=\"controller\""));
  writer.send(selected(mode == DeviceMode::Controller));
  writer.send(F(">controller</option><option value=\"reserved_r70_like\""));
  writer.send(selected(mode == DeviceMode::ReservedR70Like));
  writer.send(F(">reserved_r70_like</option></select>"));
}

void sendCommandSelect(WebUiWriter& writer, const char* name, ActionCommandType type) {
  static const ActionCommandType kTypes[] = {
      ActionCommandType::None,      ActionCommandType::PowerOn,   ActionCommandType::PowerOff,
      ActionCommandType::MuteAOn,   ActionCommandType::MuteAOff,  ActionCommandType::MuteBOn,
      ActionCommandType::MuteBOff,  ActionCommandType::DelayAOn,  ActionCommandType::DelayAOff,
      ActionCommandType::DelayBOn,  ActionCommandType::DelayBOff, ActionCommandType::GainASet,
      ActionCommandType::GainBSet};

  writer.send(F("<select name=\""));
  writer.send(name);
  writer.send(F("\">"));
  for (uint8_t index = 0; index < sizeof(kTypes) / sizeof(kTypes[0]); ++index) {
    const char* value = commandTypeName(kTypes[index]);
    writer.send(F("<option value=\""));
    writer.send(value);
    writer.send(F("\""));
    writer.send(selected(type == kTypes[index]));
    writer.send(F(">"));
    writer.send(value);
    writer.send(F("</option>"));
  }
  writer.send(F("</select>"));
}

#ifdef DBCAN_HAS_ARDUINO_ETHERNET

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

String urlDecode(const String& value) {
  String decoded;
  decoded.reserve(value.length());
  for (size_t index = 0; index < value.length(); ++index) {
    const char c = value[index];
    if (c == '+') {
      decoded += ' ';
    } else if (c == '%' && index + 2 < value.length()) {
      const int high = hexNibble(value[index + 1]);
      const int low = hexNibble(value[index + 2]);
      if (high >= 0 && low >= 0) {
        decoded += static_cast<char>((high << 4) | low);
        index += 2;
      } else {
        decoded += c;
      }
    } else {
      decoded += c;
    }
  }
  return decoded;
}

void parseFormEncoded(const String& encoded, EthernetWebUiRequest& request) {
  size_t start = 0;
  while (start <= encoded.length()) {
    const int separator = encoded.indexOf('&', start);
    const size_t end = separator >= 0 ? static_cast<size_t>(separator) : encoded.length();
    if (end > start) {
      const String pair = encoded.substring(start, end);
      const int equals = pair.indexOf('=');
      if (equals >= 0) {
        request.addArg(urlDecode(pair.substring(0, equals)), urlDecode(pair.substring(equals + 1)));
      } else {
        request.addArg(urlDecode(pair), String());
      }
    }
    if (separator < 0) {
      break;
    }
    start = static_cast<size_t>(separator) + 1;
  }
}

bool readLine(EthernetClient& client, String& line, size_t maxLen, uint32_t timeoutMs) {
  line = String();
  const uint32_t started = millis();
  while (millis() - started < timeoutMs) {
    while (client.available() > 0) {
      const char c = static_cast<char>(client.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        return true;
      }
      if (line.length() < maxLen) {
        line += c;
      }
    }
    delay(1);
  }
  return line.length() > 0;
}

bool readBody(EthernetClient& client, String& body, size_t contentLength, uint32_t timeoutMs) {
  static const size_t kMaxBodyBytes = 4096;
  if (contentLength > kMaxBodyBytes) {
    return false;
  }

  body = String();
  body.reserve(contentLength);
  const uint32_t started = millis();
  while (body.length() < contentLength && millis() - started < timeoutMs) {
    while (client.available() > 0 && body.length() < contentLength) {
      body += static_cast<char>(client.read());
    }
    if (body.length() < contentLength) {
      delay(1);
    }
  }
  return body.length() == contentLength;
}

bool parseEthernetRequest(EthernetClient& client, EthernetWebUiRequest& request, bool& tooLarge) {
  static const uint32_t kRequestTimeoutMs = 1000;
  static const size_t kMaxLineLength = 256;

  tooLarge = false;

  String requestLine;
  if (!readLine(client, requestLine, kMaxLineLength, kRequestTimeoutMs)) {
    return false;
  }

  const int methodEnd = requestLine.indexOf(' ');
  const int targetEnd = methodEnd >= 0 ? requestLine.indexOf(' ', methodEnd + 1) : -1;
  if (methodEnd <= 0 || targetEnd <= methodEnd + 1) {
    return false;
  }

  request.method = requestLine.substring(0, methodEnd);
  String target = requestLine.substring(methodEnd + 1, targetEnd);
  const int queryStart = target.indexOf('?');
  if (queryStart >= 0) {
    request.path = target.substring(0, queryStart);
    parseFormEncoded(target.substring(queryStart + 1), request);
  } else {
    request.path = target;
  }

  size_t contentLength = 0;
  while (true) {
    String headerLine;
    if (!readLine(client, headerLine, kMaxLineLength, kRequestTimeoutMs)) {
      return false;
    }
    if (headerLine.length() == 0) {
      break;
    }

    const int colon = headerLine.indexOf(':');
    if (colon > 0) {
      String name = headerLine.substring(0, colon);
      String value = headerLine.substring(colon + 1);
      name.toLowerCase();
      value.trim();
      if (name == "content-length") {
        contentLength = static_cast<size_t>(strtoul(value.c_str(), nullptr, 10));
      }
    }
  }

  if (contentLength > 0) {
    String body;
    if (!readBody(client, body, contentLength, kRequestTimeoutMs)) {
      tooLarge = true;
      return false;
    }
    parseFormEncoded(body, request);
  }

  return true;
}

void sendEthernetHeader(EthernetClient& client, int code, const char* phrase, const char* contentType) {
  client.print(F("HTTP/1.1 "));
  client.print(code);
  client.print(' ');
  client.println(phrase);
  if (contentType != nullptr) {
    client.print(F("Content-Type: "));
    client.println(contentType);
  }
  client.println(F("Connection: close"));
  client.println();
}

void sendEthernetText(EthernetClient& client, int code, const char* phrase, const char* text) {
  sendEthernetHeader(client, code, phrase, "text/plain; charset=utf-8");
  client.print(text);
}

void sendEthernetRedirect(EthernetClient& client) {
  client.println(F("HTTP/1.1 303 See Other"));
  client.println(F("Location: /"));
  client.println(F("Content-Length: 0"));
  client.println(F("Connection: close"));
  client.println();
}

#endif  // DBCAN_HAS_ARDUINO_ETHERNET

}  // namespace

WebUi::WebUi(ConfigManager& configManager)
    : configManager_(configManager), server_(nullptr), ethernetServer_(nullptr) {}

WebUi::~WebUi() {
  delete server_;
}

bool WebUi::begin(uint16_t port) {
  delete server_;
  ethernetServer_ = nullptr;
  server_ = new (std::nothrow) WebServer(port);
  if (server_ == nullptr) {
    return false;
  }
  registerHandlers();
  server_->begin();
  return true;
}

bool WebUi::begin(EthernetServer& server) {
#ifdef DBCAN_HAS_ARDUINO_ETHERNET
  delete server_;
  server_ = nullptr;
  ethernetServer_ = &server;
  ethernetServer_->begin();
  return true;
#else
  (void)server;
  return false;
#endif
}

void WebUi::handleClient() {
  if (server_ != nullptr) {
    server_->handleClient();
  }
  if (ethernetServer_ != nullptr) {
    handleEthernetClient();
  }
}

void WebUi::setActionHook(ActionHook hook, void* userData) {
  actionHook_ = hook;
  actionHookUserData_ = userData;
}

uint8_t WebUi::lastTriggeredAction() const {
  return lastTriggeredAction_;
}

void WebUi::registerHandlers() {
  server_->on("/", HTTP_GET, [this]() { handleRoot(); });
  server_->on("/save", HTTP_POST, [this]() { handleSaveConfig(); });
  server_->on("/clear-errors", HTTP_POST, [this]() { handleClearErrors(); });
  server_->on("/reset-peaks", HTTP_POST, [this]() { handleResetPeaks(); });
  server_->on("/trigger1", HTTP_POST, [this]() { handleTriggerAction(1); });
  server_->on("/trigger2", HTTP_POST, [this]() { handleTriggerAction(2); });
  server_->on("/trigger3", HTTP_POST, [this]() { handleTriggerAction(3); });
  server_->on("/trigger4", HTTP_POST, [this]() { handleTriggerAction(4); });
  server_->on("/restart", HTTP_POST, [this]() { handleRestart(); });
  server_->on("/factory-defaults", HTTP_POST, [this]() { handleFactoryDefaults(); });
  server_->onNotFound([this]() { handleNotFound(); });
}

void WebUi::handleRoot() {
  sendMainPage();
}

void WebUi::handleSaveConfig() {
  NativeWebUiRequest request(*server_);
  handleSaveConfig(request);
  redirectToRoot();
}

void WebUi::handleSaveConfig(const WebUiRequest& request) {
  AppConfig& config = configManager_.config();

  copyArg(request, "host_name", config.device.hostName, sizeof(config.device.hostName));
  config.lan.dhcpEnabled = request.hasArg("dhcp");
  if (request.hasArg("ip")) {
    parseIp(request.arg("ip"), config.lan.ipAddress);
  }
  if (request.hasArg("subnet")) {
    parseIp(request.arg("subnet"), config.lan.subnetMask);
  }
  if (request.hasArg("gateway")) {
    parseIp(request.arg("gateway"), config.lan.gateway);
  }

  config.gateway.canBitrate =
      parseU32(request.arg("can_bitrate"), config.gateway.canBitrate);
  config.gateway.deviceMode =
      parseDeviceMode(request.arg("device_mode"), config.gateway.deviceMode);
  config.gateway.maxManagedAmps =
      parseU8(request.arg("max_amps"), config.gateway.maxManagedAmps);
  config.gateway.pollIntervalFastMs =
      parseU16(request.arg("poll_fast"), config.gateway.pollIntervalFastMs);
  config.gateway.pollIntervalSlowMs =
      parseU16(request.arg("poll_slow"), config.gateway.pollIntervalSlowMs);
  config.gateway.actionQueueLength =
      parseU8(request.arg("action_queue_len"), config.gateway.actionQueueLength);
  config.gateway.canRxQueueLength =
      parseU8(request.arg("can_rx_queue_len"), config.gateway.canRxQueueLength);
  config.gateway.canTxQueueLength =
      parseU8(request.arg("can_tx_queue_len"), config.gateway.canTxQueueLength);
  config.gateway.ethRxQueueLength =
      parseU8(request.arg("eth_rx_queue_len"), config.gateway.ethRxQueueLength);
  config.gateway.ethTxQueueLength =
      parseU8(request.arg("eth_tx_queue_len"), config.gateway.ethTxQueueLength);

  for (uint8_t index = 0; index < kGpiCount; ++index) {
    const uint8_t number = index + 1;
    config.gpi[index].enabled = request.hasArg(fieldName("gpi", number, "_enabled"));
    config.gpi[index].gpioPin =
        parseU8(request.arg(fieldName("gpi", number, "_pin")), config.gpi[index].gpioPin);
    config.gpi[index].activeLow = request.hasArg(fieldName("gpi", number, "_active_low"));
    config.gpi[index].debounceMs =
        parseU16(request.arg(fieldName("gpi", number, "_debounce")), config.gpi[index].debounceMs);
    config.gpi[index].assignedActionIndex =
        parseU8(request.arg(fieldName("gpi", number, "_action")), config.gpi[index].assignedActionIndex);
    if (config.gpi[index].assignedActionIndex >= kActionCount) {
      config.gpi[index].assignedActionIndex = kActionCount - 1;
    }
  }

  for (uint8_t index = 0; index < kActionCount; ++index) {
    const uint8_t number = index + 1;
    config.actions[index].enabled = request.hasArg(fieldName("act", number, "_enabled"));
    copyArg(request, fieldName("act", number, "_name").c_str(), config.actions[index].name,
            sizeof(config.actions[index].name));
    config.actions[index].targetAmpIndex =
        parseU8(request.arg(fieldName("act", number, "_amp")), config.actions[index].targetAmpIndex);
    config.actions[index].commandType =
        parseCommandType(request.arg(fieldName("act", number, "_cmd")),
                         config.actions[index].commandType);
    config.actions[index].optionalValue =
        parseI16(request.arg(fieldName("act", number, "_value")), config.actions[index].optionalValue);
    config.actions[index].delayAfterMs =
        parseU16(request.arg(fieldName("act", number, "_delay")), config.actions[index].delayAfterMs);
  }

  RuntimeStatus& status = configManager_.runtimeStatus();
  const bool saved = configManager_.save();
  if (saved) {
    status.canRxQueue.capacity = config.gateway.canRxQueueLength;
    status.canTxQueue.capacity = config.gateway.canTxQueueLength;
    status.ethRxQueue.capacity = config.gateway.ethRxQueueLength;
    status.ethTxQueue.capacity = config.gateway.ethTxQueueLength;
    status.actionQueue.capacity = config.gateway.actionQueueLength;
  }
  status.errors.configError = !saved;
}

void WebUi::handleClearErrors() {
  configManager_.runtimeStatus().errors = {};
  redirectToRoot();
}

void WebUi::handleResetPeaks() {
  RuntimeStatus& status = configManager_.runtimeStatus();
  status.canRxQueue.peakFill = status.canRxQueue.currentFill;
  status.canTxQueue.peakFill = status.canTxQueue.currentFill;
  status.ethRxQueue.peakFill = status.ethRxQueue.currentFill;
  status.ethTxQueue.peakFill = status.ethTxQueue.currentFill;
  status.actionQueue.peakFill = status.actionQueue.currentFill;
  redirectToRoot();
}

void WebUi::handleTriggerAction(uint8_t actionNumber) {
  if (actionNumber >= 1 && actionNumber <= kActionCount) {
    lastTriggeredAction_ = actionNumber;
    if (actionHook_ != nullptr) {
      actionHook_(actionNumber, actionHookUserData_);
    }
  }
  redirectToRoot();
}

void WebUi::handleRestart() {
  server_->send(200, "text/plain", "Restarting");
  delay(100);
  ESP.restart();
}

void WebUi::handleFactoryDefaults() {
  configManager_.resetToDefaults();
  configManager_.runtimeStatus().errors.configError = !configManager_.save();
  redirectToRoot();
}

void WebUi::handleNotFound() {
  server_->send(404, "text/plain", "Not found");
}

void WebUi::redirectToRoot() {
  server_->sendHeader("Location", "/", true);
  server_->send(303, "text/plain", "");
}

void WebUi::handleEthernetClient() {
#ifdef DBCAN_HAS_ARDUINO_ETHERNET
  EthernetClient client = ethernetServer_->available();
  if (!client) {
    return;
  }

  EthernetWebUiRequest request;
  bool tooLarge = false;
  if (!parseEthernetRequest(client, request, tooLarge)) {
    sendEthernetText(client, tooLarge ? 413 : 400, tooLarge ? "Payload Too Large" : "Bad Request",
                     tooLarge ? "Request body too large" : "Bad request");
    delay(1);
    client.stop();
    return;
  }

  if (request.method == "GET" && request.path == "/") {
    sendEthernetHeader(client, 200, "OK", "text/html; charset=utf-8");
    WebUiWriter writer(client);
    sendMainPage(writer);
  } else if (request.method == "POST" && request.path == "/save") {
    handleSaveConfig(request);
    sendEthernetRedirect(client);
  } else if (request.method == "POST" && request.path == "/clear-errors") {
    configManager_.runtimeStatus().errors = {};
    sendEthernetRedirect(client);
  } else if (request.method == "POST" && request.path == "/reset-peaks") {
    RuntimeStatus& status = configManager_.runtimeStatus();
    status.canRxQueue.peakFill = status.canRxQueue.currentFill;
    status.canTxQueue.peakFill = status.canTxQueue.currentFill;
    status.ethRxQueue.peakFill = status.ethRxQueue.currentFill;
    status.ethTxQueue.peakFill = status.ethTxQueue.currentFill;
    status.actionQueue.peakFill = status.actionQueue.currentFill;
    sendEthernetRedirect(client);
  } else if (request.method == "POST" && request.path == "/factory-defaults") {
    configManager_.resetToDefaults();
    configManager_.runtimeStatus().errors.configError = !configManager_.save();
    sendEthernetRedirect(client);
  } else if (request.method == "POST" && request.path == "/restart") {
    sendEthernetText(client, 200, "OK", "Restarting");
    delay(100);
    client.stop();
    ESP.restart();
    return;
  } else if (request.method == "POST" && request.path.startsWith("/trigger")) {
    const uint8_t actionNumber =
        static_cast<uint8_t>(strtoul(request.path.substring(strlen("/trigger")).c_str(), nullptr, 10));
    if (actionNumber >= 1 && actionNumber <= kActionCount) {
      lastTriggeredAction_ = actionNumber;
      if (actionHook_ != nullptr) {
        actionHook_(actionNumber, actionHookUserData_);
      }
      sendEthernetRedirect(client);
    } else {
      sendEthernetText(client, 404, "Not Found", "Not found");
    }
  } else {
    sendEthernetText(client, 404, "Not Found", "Not found");
  }

  delay(1);
  client.stop();
#endif
}

void WebUi::sendMainPage() {
  server_->setContentLength(CONTENT_LENGTH_UNKNOWN);
  server_->send(200, "text/html", "");
  WebUiWriter writer(*server_);
  sendMainPage(writer);
  writer.send("");
}

void WebUi::sendMainPage(WebUiWriter& writer) {
  const AppConfig& config = configManager_.config();
  const RuntimeStatus& status = configManager_.runtimeStatus();

  writer.send(F(
      "<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      "<title>CueSmart-CueCan-Config</title><style>"
      "body{margin:0;background:#242424;color:#ddd;font:13px Arial,Helvetica,sans-serif}"
      "main{max-width:1120px;margin:0 auto;padding:8px}"
      "h1{font-size:18px;margin:4px 0 8px;color:#fff;font-weight:normal}"
      "h2{font-size:14px;margin:0 0 6px;color:#f0f0f0;background:#333;padding:4px 6px;font-weight:bold}"
      ".panel{background:#2d2d2d;border:1px solid #555;margin:0 0 8px;padding:6px}"
      ".grid{display:grid;grid-template-columns:180px 1fr;gap:4px 8px;align-items:center}"
      ".cols{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
      "label{color:#bbb}input,select{background:#1c1c1c;color:#eee;border:1px solid #666;padding:3px;width:100%;box-sizing:border-box}"
      "input[type=checkbox]{width:auto}button{background:#555;color:#fff;border:1px solid #888;padding:4px 10px;border-radius:0;margin:2px}"
      "table{width:100%;border-collapse:collapse}td,th{border:1px solid #555;padding:3px 5px;text-align:left}th{background:#383838}"
      ".led{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}.green{background:#20b020}.red{background:#b02020}"
      ".err{display:inline-block;width:230px;margin:2px 0}.muted{color:#999}.svc form{display:inline}.small{font-size:12px}"
      "@media(max-width:780px){.cols{grid-template-columns:1fr}.grid{grid-template-columns:1fr}}</style></head><body><main>"));

  writer.send(F("<h1>CueSmart-CueCan-Config</h1><form method=\"post\" action=\"/save\">"));

  writer.send(F("<div class=\"cols\"><section class=\"panel\"><h2>Device Info</h2><div class=\"grid\">"));
  writer.send(F("<label>Device name</label><div>"));
  sendEscaped(writer, config.device.deviceName);
  writer.send(F("</div><label>Host name</label>"));
  sendInput(writer, "host_name", config.device.hostName, sizeof(config.device.hostName));
  writer.send(F("<label>Hardware</label><div>"));
  sendEscaped(writer, config.device.hardwareVersion);
  writer.send(F("</div><label>Firmware</label><div>"));
  sendEscaped(writer, config.device.firmwareVersion);
  writer.send(F("</div><label>Schema</label><div>"));
  writer.send(String(config.schemaVersion));
  writer.send(F("</div></div></section>"));

  writer.send(F("<section class=\"panel\"><h2>Online Error Status</h2>"));
  sendIndicator(writer, "CAN RX buffer overflow", status.errors.canRxOverflow);
  sendIndicator(writer, "CAN TX buffer overflow", status.errors.canTxOverflow);
  sendIndicator(writer, "CAN transmission timeout", status.errors.canTimeout);
  sendIndicator(writer, "CAN error counter overflow", status.errors.canErrorCounterOverflow);
  sendIndicator(writer, "CAN bus off error", status.errors.canBusOff);
  sendIndicator(writer, "Ethernet RX buffer overflow", status.errors.ethRxOverflow);
  sendIndicator(writer, "Ethernet TX buffer overflow", status.errors.ethTxOverflow);
  sendIndicator(writer, "Ethernet connection lost", status.errors.ethLinkDown);
  sendIndicator(writer, "internal action queue overflow", status.errors.actionQueueOverflow);
  sendIndicator(writer, "config load/save error", status.errors.configError);
  writer.send(F("</section></div>"));

  writer.send(F("<div class=\"cols\"><section class=\"panel\"><h2>LAN Parameters</h2><div class=\"grid\">"));
  writer.send(F("<label>DHCP enabled</label>"));
  sendCheckbox(writer, "dhcp", config.lan.dhcpEnabled);
  writer.send(F("<label>IP address</label><input name=\"ip\" value=\""));
  sendIp(writer, config.lan.ipAddress);
  writer.send(F("\"><label>Subnet mask</label><input name=\"subnet\" value=\""));
  sendIp(writer, config.lan.subnetMask);
  writer.send(F("\"><label>Gateway</label><input name=\"gateway\" value=\""));
  sendIp(writer, config.lan.gateway);
  writer.send(F("\"></div></section>"));

  writer.send(F("<section class=\"panel\"><h2>dbCAN / Gateway Parameters</h2><div class=\"grid\">"));
  writer.send(F("<label>CAN bitrate</label>"));
  sendNumberInput(writer, "can_bitrate", config.gateway.canBitrate);
  writer.send(F("<label>Device mode</label>"));
  sendDeviceModeSelect(writer, config.gateway.deviceMode);
  writer.send(F("<label>Max managed amps</label>"));
  sendNumberInput(writer, "max_amps", config.gateway.maxManagedAmps);
  writer.send(F("<label>Poll fast ms</label>"));
  sendNumberInput(writer, "poll_fast", config.gateway.pollIntervalFastMs);
  writer.send(F("<label>Poll slow ms</label>"));
  sendNumberInput(writer, "poll_slow", config.gateway.pollIntervalSlowMs);
  writer.send(F("<label>Action queue length</label>"));
  sendNumberInput(writer, "action_queue_len", config.gateway.actionQueueLength);
  writer.send(F("<label>CAN RX queue length</label>"));
  sendNumberInput(writer, "can_rx_queue_len", config.gateway.canRxQueueLength);
  writer.send(F("<label>CAN TX queue length</label>"));
  sendNumberInput(writer, "can_tx_queue_len", config.gateway.canTxQueueLength);
  writer.send(F("<label>Ethernet RX queue length</label>"));
  sendNumberInput(writer, "eth_rx_queue_len", config.gateway.ethRxQueueLength);
  writer.send(F("<label>Ethernet TX queue length</label>"));
  sendNumberInput(writer, "eth_tx_queue_len", config.gateway.ethTxQueueLength);
  writer.send(F("</div></section></div>"));

  writer.send(F("<section class=\"panel\"><h2>Buffer / Queue Status</h2><table><tr><th>Queue</th><th>Current</th><th>Peak</th><th>Capacity</th><th>Overflow</th><th>Dropped</th></tr>"));
  sendQueueRow(writer, "CAN RX", status.canRxQueue);
  sendQueueRow(writer, "CAN TX", status.canTxQueue);
  sendQueueRow(writer, "Ethernet RX", status.ethRxQueue);
  sendQueueRow(writer, "Ethernet TX", status.ethTxQueue);
  sendQueueRow(writer, "Action", status.actionQueue);
  writer.send(F("</table></section>"));

  writer.send(F("<section class=\"panel\"><h2>Web Test Actions</h2><div class=\"muted small\">Buttons call a stub hook only. No dbCAN command execution is wired here.</div>"));
  for (uint8_t index = 0; index < kActionCount; ++index) {
    writer.send(F("<button form=\"trig"));
    writer.send(String(index + 1));
    writer.send(F("\">Trigger "));
    writer.send(String(index + 1));
    writer.send(F(": "));
    sendEscaped(writer, config.actions[index].name);
    writer.send(F("</button>"));
  }
  if (lastTriggeredAction_ > 0) {
    writer.send(F("<span class=\"muted\">last trigger: "));
    writer.send(String(lastTriggeredAction_));
    writer.send(F("</span>"));
  }
  writer.send(F("</section>"));

  writer.send(F("<section class=\"panel\"><h2>GPI Configuration</h2><table><tr><th>GPI</th><th>Enabled</th><th>GPIO</th><th>Active low</th><th>Debounce ms</th><th>Action</th><th>Level</th><th>Triggers</th></tr>"));
  for (uint8_t index = 0; index < kGpiCount; ++index) {
    const uint8_t number = index + 1;
    writer.send(F("<tr><td>"));
    writer.send(String(number));
    writer.send(F("</td><td>"));
    sendCheckbox(writer, fieldName("gpi", number, "_enabled").c_str(), config.gpi[index].enabled);
    writer.send(F("</td><td>"));
    sendNumberInput(writer, fieldName("gpi", number, "_pin").c_str(), config.gpi[index].gpioPin);
    writer.send(F("</td><td>"));
    sendCheckbox(writer, fieldName("gpi", number, "_active_low").c_str(), config.gpi[index].activeLow);
    writer.send(F("</td><td>"));
    sendNumberInput(writer, fieldName("gpi", number, "_debounce").c_str(), config.gpi[index].debounceMs);
    writer.send(F("</td><td>"));
    sendNumberInput(writer, fieldName("gpi", number, "_action").c_str(), config.gpi[index].assignedActionIndex);
    writer.send(F("</td><td>"));
    writer.send(onOff(status.gpi[index].currentLevel));
    writer.send(F("</td><td>"));
    writer.send(String(status.gpi[index].triggerCount));
    writer.send(F("</td></tr>"));
  }
  writer.send(F("</table></section>"));

  writer.send(F("<section class=\"panel\"><h2>Action Configuration</h2><table><tr><th>Action</th><th>Enabled</th><th>Name</th><th>Amp</th><th>Command</th><th>Value</th><th>Delay ms</th></tr>"));
  for (uint8_t index = 0; index < kActionCount; ++index) {
    const uint8_t number = index + 1;
    writer.send(F("<tr><td>"));
    writer.send(String(number));
    writer.send(F("</td><td>"));
    sendCheckbox(writer, fieldName("act", number, "_enabled").c_str(), config.actions[index].enabled);
    writer.send(F("</td><td>"));
    sendInput(writer, fieldName("act", number, "_name").c_str(), config.actions[index].name,
              sizeof(config.actions[index].name));
    writer.send(F("</td><td>"));
    sendNumberInput(writer, fieldName("act", number, "_amp").c_str(), config.actions[index].targetAmpIndex);
    writer.send(F("</td><td>"));
    sendCommandSelect(writer, fieldName("act", number, "_cmd").c_str(), config.actions[index].commandType);
    writer.send(F("</td><td>"));
    sendNumberInput(writer, fieldName("act", number, "_value").c_str(), config.actions[index].optionalValue);
    writer.send(F("</td><td>"));
    sendNumberInput(writer, fieldName("act", number, "_delay").c_str(), config.actions[index].delayAfterMs);
    writer.send(F("</td></tr>"));
  }
  writer.send(F("</table></section>"));

  writer.send(F("<section class=\"panel svc\"><h2>Footer / Service Actions</h2><button type=\"submit\">Save config</button></form>"));
  writer.send(F("<form method=\"post\" action=\"/clear-errors\"><button>Clear errors</button></form>"));
  writer.send(F("<form method=\"post\" action=\"/reset-peaks\"><button>Reset peaks</button></form>"));
  writer.send(F("<form method=\"post\" action=\"/factory-defaults\"><button>Factory defaults</button></form>"));
  writer.send(F("<form method=\"post\" action=\"/restart\"><button>Restart device</button></form>"));
  writer.send(F("<span class=\"muted small\">mode: "));
  writer.send(deviceModeName(config.gateway.deviceMode));
  writer.send(F("</span></section>"));

  for (uint8_t index = 0; index < kActionCount; ++index) {
    writer.send(F("<form id=\"trig"));
    writer.send(String(index + 1));
    writer.send(F("\" method=\"post\" action=\"/trigger"));
    writer.send(String(index + 1));
    writer.send(F("\"></form>"));
  }

  writer.send(F("</main></body></html>"));
  writer.send("");
}

}  // namespace dbcan
