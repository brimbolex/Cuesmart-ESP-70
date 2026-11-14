#include "gpi/GpiManager.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Log.h"
#include "../R70GatewayV2.h"
#include "gpi/GpiActions.h"

namespace dbcan {
namespace r70v2 {

#ifndef GPI_DIAG
#define GPI_DIAG (LOG_LEVEL >= LOG_LEVEL_DEBUG)
#endif

namespace {

struct PhysicalGpiConfig {
  uint8_t pin;
  bool activeLow;
  bool usePullup;
  uint16_t debounceMs;
};

static const PhysicalGpiConfig kPhysicalInputs[kGpiPhysicalCount] = {
    {15, true, true, 30},
    {16, true, true, 30},
    {17, true, true, 30},
    {18, true, true, 30},
};

GpiManager* gActiveManager = 0;

uint8_t parseRouteIndex(const char* text) {
  if (text == 0) {
    return 0xFF;
  }
  char* end = 0;
  const unsigned long value = strtoul(text, &end, 10);
  if (end == text || *end != '\0' || value < 1 || value > kGpiRouteCount) {
    return 0xFF;
  }
  return static_cast<uint8_t>(value - 1);
}

bool parseOnOff(const char* text, bool& value) {
  if (text == 0) {
    return false;
  }
  if (strcmp(text, "on") == 0) {
    value = true;
    return true;
  }
  if (strcmp(text, "off") == 0) {
    value = false;
    return true;
  }
  return false;
}

const char* repeatSourceFor(const char* source) {
  if (source != 0 && strcmp(source, "web") == 0) {
    return "web-repeat";
  }
  if (source != 0 && strcmp(source, "test") == 0) {
    return "test-repeat";
  }
  return "gpi-repeat";
}

void printTarget(Print& out, const GpiRouteRow& route) {
  out.print(static_cast<unsigned>(route.targetSubnet));
  out.print('.');
  if (route.targetDevice < 10) {
    out.print('0');
  }
  out.print(static_cast<unsigned>(route.targetDevice));
}

}  // namespace

GpiManager::GpiManager()
    : gateway_(0),
      log_(0),
      settings_(),
      settingsFromFactory_(true),
      physicalRuntime_(),
      routeRuntime_(),
      started_(false) {}

bool GpiManager::begin(R70GatewayV2& gateway, Stream& log) {
  gateway_ = &gateway;
  log_ = &log;
  started_ = store_.begin();
  if (!started_) {
    if (log_ != 0) {
      LOG_PRINTLN(LOG_LEVEL_ERROR, *log_, "GPI_BEGIN_FAILED reason=store_begin_failed");
    }
    return false;
  }

  LOG_BLOCK(LOG_LEVEL_DEBUG, if (log_ != 0) {
    log_->println("GPI_BEGIN_START");
    log_->println("[BOOT_STAGE] before_gpi_settings_load");
  });
  bool usedFactory = true;
  loadActiveSettings(usedFactory);
  settingsFromFactory_ = usedFactory;
  LOG_BLOCK(LOG_LEVEL_DEBUG, if (log_ != 0) {
    log_->println("[BOOT_STAGE] after_gpi_settings_load");
    log_->println("[BOOT_STAGE] before_gpi_init");
    log_->println(
        "GPI_ROUTE_MODEL targetSubnet=per_route_persistent targetDevice=per_route_persistent loadPreset=per_route_persistent backupSlot=per_route_persistent physicalGpi=per_route_persistent");
  });

  GpiActions_begin(gateway, log);
  for (uint8_t index = 0; index < kGpiPhysicalCount; ++index) {
    physicalRuntime_[index] = {};
    configurePhysicalInput(index, millis());
  }
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    routeRuntime_[index] = {};
    const char* validationReason = 0;
    uint16_t canId = 0;
    const bool valid = validateRoute(index, validationReason, canId);
    logRouteConfigLine(index, valid, validationReason, canId);
    if (settings_.route[index].enabled && !valid && gateway_ != 0) {
      ++gateway_->mutableCounters().gpi_invalid_config_count;
    }
  }
  updateEnabledMaskCounter();
  updatePendingRepeatsCounter();
  LOG_BLOCK(LOG_LEVEL_INFO, if (log_ != 0) {
    char line[192] = {};
    snprintf(line,
             sizeof(line),
            "GPI_SETTINGS source=%s enabled_mask=0x%02X store_reason=%s",
             settingsFromFactory_ ? "factory" : "nvs",
             static_cast<unsigned>(gateway_->mutableCounters().gpi_enabled_mask),
             store_.lastError());
    log_->println(line);
  });
  LOG_BLOCK(LOG_LEVEL_DEBUG, if (log_ != 0) {
    log_->println("[BOOT_STAGE] after_gpi_init");
    log_->println("GPI_BEGIN_DONE");
  });
  gActiveManager = this;
  return true;
}

bool GpiManager::loadActiveSettings(bool& usedFactory) {
  return store_.loadOrFactory(settings_, usedFactory);
}

void GpiManager::updateEnabledMaskCounter() {
  if (gateway_ == 0) {
    return;
  }
  uint32_t mask = 0;
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    if (settings_.route[index].enabled) {
      mask |= static_cast<uint32_t>(1u << index);
    }
  }
  gateway_->mutableCounters().gpi_enabled_mask = mask;
}

void GpiManager::updatePendingRepeatsCounter() {
  if (gateway_ == 0) {
    return;
  }
  uint32_t pending = 0;
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    if (routeRuntime_[index].releaseRepeatActive) {
      pending += routeRuntime_[index].releaseRepeatsRemaining;
    }
  }
  gateway_->mutableCounters().gpi_pending_repeats = pending;
  gateway_->mutableCounters().gpi_route_revert_pending = pending;
}

bool GpiManager::validateRoute(uint8_t rowIndex, const char*& reason, uint16_t& canId) const {
  reason = "invalid_index";
  canId = 0;
  if (rowIndex >= kGpiRouteCount) {
    return false;
  }
  const GpiRouteRow& route = settings_.route[rowIndex];
  canId = makePrimaryMasterCanId(route.targetSubnet, route.targetDevice);
  if (!route.enabled) {
    reason = "disabled";
    return false;
  }
  if (route.physicalGpi < 1 || route.physicalGpi > kGpiPhysicalCount) {
    reason = "invalid_input";
    return false;
  }
  if (route.targetSubnet > 7) {
    reason = "invalid_subnet";
    return false;
  }
  if (route.targetDevice > 63) {
    reason = "invalid_device";
    return false;
  }
  if (route.loadPreset < 1 || route.loadPreset > 14) {
    reason = "invalid_load_preset";
    return false;
  }
  if (route.backupSlot < 1 || route.backupSlot > 14) {
    reason = "invalid_backup_slot";
    return false;
  }
  if (route.actionMode != GpiActionMode::None &&
      route.actionMode != GpiActionMode::BackupLoadRestore) {
    reason = "invalid_action_mode";
    return false;
  }
  reason = "ok";
  return true;
}

void GpiManager::logRouteConfigLine(uint8_t rowIndex,
                                    bool valid,
                                    const char* reason,
                                    uint16_t canId) const {
  if (log_ == 0 || rowIndex >= kGpiRouteCount || !LOG_ENABLED(LOG_LEVEL_INFO)) {
    return;
  }
  const GpiRouteRow& route = settings_.route[rowIndex];
  char line[224] = {};
  if (!route.enabled) {
    snprintf(line,
             sizeof(line),
             "[GPI ROUTE] row=%u enabled=0 valid=0 reason=%s input=GPI%u target=%u.%02u canId=0x%03X load=%u backup=%u",
             static_cast<unsigned>(rowIndex + 1),
             reason != 0 ? reason : "disabled",
             static_cast<unsigned>(route.physicalGpi),
             static_cast<unsigned>(route.targetSubnet),
             static_cast<unsigned>(route.targetDevice),
             static_cast<unsigned>(canId),
             static_cast<unsigned>(route.loadPreset),
             static_cast<unsigned>(route.backupSlot));
  } else if (!valid) {
    snprintf(line,
             sizeof(line),
             "[GPI ROUTE] row=%u enabled=1 valid=0 reason=%s input=GPI%u target=%u.%02u load=%u backup=%u",
             static_cast<unsigned>(rowIndex + 1),
             reason != 0 ? reason : "invalid",
             static_cast<unsigned>(route.physicalGpi),
             static_cast<unsigned>(route.targetSubnet),
             static_cast<unsigned>(route.targetDevice),
             static_cast<unsigned>(route.loadPreset),
             static_cast<unsigned>(route.backupSlot));
  } else {
    snprintf(line,
             sizeof(line),
             "[GPI ROUTE] row=%u enabled=1 valid=1 input=GPI%u target=%u.%02u canId=0x%03X load=%u backup=%u",
             static_cast<unsigned>(rowIndex + 1),
             static_cast<unsigned>(route.physicalGpi),
             static_cast<unsigned>(route.targetSubnet),
             static_cast<unsigned>(route.targetDevice),
             static_cast<unsigned>(canId),
             static_cast<unsigned>(route.loadPreset),
             static_cast<unsigned>(route.backupSlot));
  }
  log_->println(line);
}

void GpiManager::configurePhysicalInput(uint8_t physicalIndex, uint32_t nowMs) {
  if (physicalIndex >= kGpiPhysicalCount) {
    return;
  }
  PhysicalInputRuntime& runtime = physicalRuntime_[physicalIndex];
  runtime = {};
  const PhysicalGpiConfig& input = kPhysicalInputs[physicalIndex];
  if (!isGpiPinAllowed(input.pin)) {
    if (gateway_ != 0) {
      ++gateway_->mutableCounters().gpi_pin_conflict_count;
      ++gateway_->mutableCounters().gpi_invalid_config_count;
    }
    return;
  }
  pinMode(input.pin, input.usePullup ? INPUT_PULLUP : INPUT);
  const bool active = readRawActive(physicalIndex);
  runtime.pinConfigured = true;
  runtime.rawActive = active;
  runtime.stableActive = active;
  runtime.lastRawChangeMs = nowMs;
  runtime.lastEdgeMs = nowMs;
}

bool GpiManager::readRawActive(uint8_t physicalIndex) const {
  const PhysicalGpiConfig& input = kPhysicalInputs[physicalIndex];
  const int rawLevel = digitalRead(input.pin);
  return input.activeLow ? (rawLevel == LOW) : (rawLevel == HIGH);
}

uint8_t GpiManager::countMatchingRoutes(uint8_t physicalGpi) const {
  uint8_t count = 0;
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    const GpiRouteRow& route = settings_.route[index];
    const char* reason = 0;
    uint16_t canId = 0;
    if (route.enabled && route.physicalGpi == physicalGpi &&
        validateRoute(index, reason, canId)) {
      ++count;
    }
  }
  return count;
}

void GpiManager::cancelRouteReleaseRepeats(uint8_t rowIndex) {
  if (rowIndex >= kGpiRouteCount) {
    return;
  }
  RouteRuntime& runtime = routeRuntime_[rowIndex];
  runtime.releaseRepeatActive = false;
  runtime.releaseRepeatsRemaining = 0;
  runtime.nextReleaseRepeatMs = 0;
  runtime.releaseRepeatSource = 0;
  updatePendingRepeatsCounter();
}

bool GpiManager::triggerRouteBackupRecall(uint8_t rowIndex, const char* source) {
  const char* reason = 0;
  uint16_t canId = 0;
  if (!validateRoute(rowIndex, reason, canId)) {
    return false;
  }
  const GpiRouteRow& route = settings_.route[rowIndex];
  if (log_ != 0) {
    char line[192] = {};
    const bool webSource = source != 0 && (strcmp(source, "web") == 0 || strcmp(source, "test") == 0);
    snprintf(line,
             sizeof(line),
             "%s row=%u action=backup_recall target=%u.%02u canId=0x%03X load=%u backup=%u",
             webSource ? "[WEB ROUTE TEST]" : "[GPI ROUTE TX]",
             static_cast<unsigned>(rowIndex + 1),
             static_cast<unsigned>(route.targetSubnet),
             static_cast<unsigned>(route.targetDevice),
             static_cast<unsigned>(canId),
             static_cast<unsigned>(route.loadPreset),
             static_cast<unsigned>(route.backupSlot));
    log_->println(line);
  }
  const bool ok = GpiActions_triggerPress(rowIndex, route, source);
  if (ok && gateway_ != 0) {
    ++gateway_->mutableCounters().gpi_route_backup_recall_tx;
  }
  return ok;
}

bool GpiManager::triggerRouteRevert(uint8_t rowIndex, const char* source, uint32_t nowMs) {
  const char* reason = 0;
  uint16_t canId = 0;
  if (!validateRoute(rowIndex, reason, canId)) {
    return false;
  }
  const GpiRouteRow& route = settings_.route[rowIndex];
  if (log_ != 0) {
    char line[192] = {};
    const bool webSource = source != 0 && (strcmp(source, "web") == 0 || strcmp(source, "test") == 0);
    snprintf(line,
             sizeof(line),
             "%s row=%u action=revert target=%u.%02u canId=0x%03X backup=%u",
             webSource ? "[WEB ROUTE TEST]" : "[GPI ROUTE TX]",
             static_cast<unsigned>(rowIndex + 1),
             static_cast<unsigned>(route.targetSubnet),
             static_cast<unsigned>(route.targetDevice),
             static_cast<unsigned>(canId),
             static_cast<unsigned>(route.backupSlot));
    log_->println(line);
  }
  cancelRouteReleaseRepeats(rowIndex);
  if (!GpiActions_triggerRelease(rowIndex, route, source, 1, 4)) {
    return false;
  }
  if (gateway_ != 0) {
    ++gateway_->mutableCounters().gpi_route_revert_tx;
  }
  RouteRuntime& runtime = routeRuntime_[rowIndex];
  runtime.releaseRepeatActive = true;
  runtime.releaseRepeatsRemaining = 3;
  runtime.nextReleaseRepeatMs = nowMs + 1000;
  runtime.releaseRepeatSource = repeatSourceFor(source);
  updatePendingRepeatsCounter();
  return true;
}

void GpiManager::handlePhysicalPress(uint8_t physicalIndex, const char* source, uint32_t nowMs) {
  if (gateway_ == 0) {
    return;
  }
  ++gateway_->mutableCounters().gpi_edges_press[physicalIndex];
  ++gateway_->mutableCounters().gpi_press_count;
  const uint8_t physicalGpi = static_cast<uint8_t>(physicalIndex + 1);
  const uint8_t matches = countMatchingRoutes(physicalGpi);
  if (log_ != 0) {
    char line[96] = {};
    snprintf(line,
             sizeof(line),
             "[GPI ROUTE TRIGGER] input=GPI%u edge=press matching_routes=%u",
             static_cast<unsigned>(physicalGpi),
             static_cast<unsigned>(matches));
    log_->println(line);
  }
#if GPI_DIAG
  LOG_BLOCK(LOG_LEVEL_DEBUG, if (log_ != 0) {
    char line[160] = {};
    snprintf(line,
             sizeof(line),
             "GPI_DIAG_PRESS slot=%u now_ms=%lu tcp_connected=%u pending_repeats=%lu",
             static_cast<unsigned>(physicalGpi),
             static_cast<unsigned long>(nowMs),
             gateway_->tcpControlConnected() ? 1u : 0u,
             static_cast<unsigned long>(gateway_->mutableCounters().gpi_pending_repeats));
    log_->println(line);
  });
#endif
  for (uint8_t rowIndex = 0; rowIndex < kGpiRouteCount; ++rowIndex) {
    if (settings_.route[rowIndex].enabled && settings_.route[rowIndex].physicalGpi == physicalGpi) {
      cancelRouteReleaseRepeats(rowIndex);
      if (triggerRouteBackupRecall(rowIndex, source)) {
        ++gateway_->mutableCounters().gpi_route_trigger_count;
      }
    }
  }
}

void GpiManager::handlePhysicalRelease(uint8_t physicalIndex, const char* source, uint32_t nowMs) {
  if (gateway_ == 0) {
    return;
  }
  ++gateway_->mutableCounters().gpi_edges_release[physicalIndex];
  ++gateway_->mutableCounters().gpi_release_count;
  const uint8_t physicalGpi = static_cast<uint8_t>(physicalIndex + 1);
  const uint8_t matches = countMatchingRoutes(physicalGpi);
  if (log_ != 0) {
    char line[96] = {};
    snprintf(line,
             sizeof(line),
             "[GPI ROUTE TRIGGER] input=GPI%u edge=release matching_routes=%u",
             static_cast<unsigned>(physicalGpi),
             static_cast<unsigned>(matches));
    log_->println(line);
  }
#if GPI_DIAG
  LOG_BLOCK(LOG_LEVEL_DEBUG, if (log_ != 0) {
    char line[160] = {};
    snprintf(line,
             sizeof(line),
             "GPI_DIAG_RELEASE slot=%u now_ms=%lu tcp_connected=%u pending_repeats=%lu",
             static_cast<unsigned>(physicalGpi),
             static_cast<unsigned long>(nowMs),
             gateway_->tcpControlConnected() ? 1u : 0u,
             static_cast<unsigned long>(gateway_->mutableCounters().gpi_pending_repeats));
    log_->println(line);
  });
#endif
  for (uint8_t rowIndex = 0; rowIndex < kGpiRouteCount; ++rowIndex) {
    if (settings_.route[rowIndex].enabled && settings_.route[rowIndex].physicalGpi == physicalGpi) {
      if (triggerRouteRevert(rowIndex, source, nowMs)) {
        ++gateway_->mutableCounters().gpi_route_trigger_count;
      }
    }
  }
}

void GpiManager::serviceReleaseRepeats(uint32_t nowMs) {
  for (uint8_t rowIndex = 0; rowIndex < kGpiRouteCount; ++rowIndex) {
    RouteRuntime& runtime = routeRuntime_[rowIndex];
    if (!runtime.releaseRepeatActive || runtime.releaseRepeatsRemaining == 0 ||
        static_cast<int32_t>(nowMs - runtime.nextReleaseRepeatMs) < 0) {
      continue;
    }

    const uint8_t repeatNumber = static_cast<uint8_t>(5 - runtime.releaseRepeatsRemaining);
    if (GpiActions_triggerRelease(rowIndex,
                                  settings_.route[rowIndex],
                                  runtime.releaseRepeatSource != 0 ? runtime.releaseRepeatSource
                                                                   : "gpi-repeat",
                                  repeatNumber,
                                  4) &&
        gateway_ != 0) {
      ++gateway_->mutableCounters().gpi_route_revert_tx;
    }
#if GPI_DIAG
    LOG_BLOCK(LOG_LEVEL_TRACE, if (log_ != 0) {
      char line[176] = {};
      snprintf(line,
               sizeof(line),
               "GPI_DIAG_RELEASE_REPEAT_TX row=%u repeat=%u/4 now_ms=%lu tcp_connected=%u",
               static_cast<unsigned>(rowIndex + 1),
               static_cast<unsigned>(repeatNumber),
               static_cast<unsigned long>(nowMs),
               gateway_ != 0 && gateway_->tcpControlConnected() ? 1u : 0u);
      log_->println(line);
    });
#endif
    --runtime.releaseRepeatsRemaining;
    if (runtime.releaseRepeatsRemaining == 0) {
      cancelRouteReleaseRepeats(rowIndex);
    } else {
      runtime.nextReleaseRepeatMs += 1000;
      updatePendingRepeatsCounter();
    }
  }
}

void GpiManager::poll(uint32_t nowMs) {
  if (!started_) {
    return;
  }
  const uint32_t pollStartUs = micros();
  serviceReleaseRepeats(nowMs);
  for (uint8_t index = 0; index < kGpiPhysicalCount; ++index) {
    const PhysicalGpiConfig& input = kPhysicalInputs[index];
    PhysicalInputRuntime& runtime = physicalRuntime_[index];
    if (!runtime.pinConfigured) {
      continue;
    }

    const bool rawActive = readRawActive(index);
    if (rawActive != runtime.rawActive) {
      if (static_cast<int32_t>(nowMs - runtime.lastRawChangeMs) <
          static_cast<int32_t>(input.debounceMs)) {
        ++gateway_->mutableCounters().gpi_debounce_ignored[index];
      }
      runtime.rawActive = rawActive;
      runtime.lastRawChangeMs = nowMs;
    }

    if (runtime.rawActive == runtime.stableActive) {
      continue;
    }
    if (static_cast<int32_t>(nowMs - runtime.lastRawChangeMs) <
        static_cast<int32_t>(input.debounceMs)) {
      continue;
    }

    runtime.stableActive = runtime.rawActive;
    runtime.lastEdgeMs = nowMs;
    if (runtime.stableActive) {
      handlePhysicalPress(index, "gpi", nowMs);
    } else {
      handlePhysicalRelease(index, "gpi", nowMs);
    }
  }
  const uint32_t pollDurationUs = micros() - pollStartUs;
  if (pollDurationUs > 2000 && log_ != 0 && LOG_ENABLED(LOG_LEVEL_WARN)) {
    log_->print("GPI_POLL_TOO_SLOW duration_us=");
    log_->println(static_cast<unsigned long>(pollDurationUs));
  }
}

void GpiManager::printSettings(Print& out) const {
  out.print("GPI_ACTIVE_SETTINGS source=");
  out.println(settingsFromFactory_ ? "factory" : "nvs");
  out.print("GPI_VALIDATION active_store_reason=");
  out.println(store_.lastError());
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    const GpiRouteRow& route = settings_.route[index];
    out.print("GPI_ROUTE row=");
    out.print(static_cast<unsigned>(index + 1));
    out.print(" enabled=");
    out.print(route.enabled ? 1 : 0);
    out.print(" physicalGpi=");
    out.print(static_cast<unsigned>(route.physicalGpi));
    out.print(" target=");
    printTarget(out, route);
    out.print(" loadPreset=");
    out.print(static_cast<unsigned>(route.loadPreset));
    out.print(" backupSlot=");
    out.print(static_cast<unsigned>(route.backupSlot));
    out.print(" actionMode=");
    out.println(gpiActionModeName(route.actionMode));
  }
  for (uint8_t index = 0; index < kGpiPhysicalCount; ++index) {
    const PhysicalInputRuntime& runtime = physicalRuntime_[index];
    out.print("GPI_PHYSICAL input=GPI");
    out.print(static_cast<unsigned>(index + 1));
    out.print(" pin=");
    out.print(static_cast<unsigned>(kPhysicalInputs[index].pin));
    out.print(" rawActive=");
    out.print(runtime.rawActive ? 1 : 0);
    out.print(" stableActive=");
    out.println(runtime.stableActive ? 1 : 0);
  }
}

void GpiManager::printStoredSettings(Print& out) {
  GpiSettings stored = {};
  bool usedFactory = true;
  if (!store_.loadOrFactory(stored, usedFactory)) {
    out.print("GPI_STORED_SETTINGS load_failed reason=");
    out.println(store_.lastError());
    return;
  }
  out.print("GPI_STORED_SETTINGS source=");
  out.println(usedFactory ? "factory" : "nvs");
  out.print("GPI_STORED_VALIDATION reason=");
  out.println(store_.lastError());
  for (uint8_t index = 0; index < kGpiRouteCount; ++index) {
    const GpiRouteRow& route = stored.route[index];
    out.print("GPI_STORED row=");
    out.print(static_cast<unsigned>(index + 1));
    out.print(" enabled=");
    out.print(route.enabled ? 1 : 0);
    out.print(" physicalGpi=");
    out.print(static_cast<unsigned>(route.physicalGpi));
    out.print(" target=");
    printTarget(out, route);
    out.print(" loadPreset=");
    out.print(static_cast<unsigned>(route.loadPreset));
    out.print(" backupSlot=");
    out.println(static_cast<unsigned>(route.backupSlot));
  }
}

bool GpiManager::loadStoredSettingsForEdit(GpiSettings& settings, bool& usedFactory) {
  return store_.loadOrFactory(settings, usedFactory);
}

bool GpiManager::saveEditedSettings(const GpiSettings& settings,
                                    Print& out,
                                    const char* successText) {
  if (!store_.saveUserSettings(settings)) {
    out.print("GPI_SAVE_FAILED reason=");
    out.println(store_.lastError());
    if (gateway_ != 0) {
      ++gateway_->mutableCounters().gpi_invalid_config_count;
    }
    return false;
  }
  out.println(successText);
  return true;
}

void GpiManager::noteInvalidConfigReason(const char* reason) {
  (void)reason;
}

bool GpiManager::handleCliCommand(const char* line, Print& out, uint32_t nowMs) {
  if (line == 0) {
    return false;
  }
  if (strcmp(line, "gpi-show") == 0) {
    printSettings(out);
    printStoredSettings(out);
    return true;
  }
  if (strcmp(line, "gpi-reset-factory") == 0) {
    if (!store_.resetUserSettingsToFactoryDefaults()) {
      out.print("GPI_RESET_FACTORY_FAILED reason=");
      out.println(store_.lastError());
      return true;
    }
    out.println("GPI reset to factory defaults; restart required.");
    return true;
  }
  if (strncmp(line, "gpi-enable ", 11) == 0) {
    char rowText[8] = {};
    char stateText[8] = {};
    if (sscanf(line + 11, "%7s %7s", rowText, stateText) != 2) {
      out.println("GPI_ENABLE_FAILED reason=bad_arguments");
      return true;
    }
    const uint8_t rowIndex = parseRouteIndex(rowText);
    bool enabled = false;
    if (rowIndex == 0xFF || !parseOnOff(stateText, enabled)) {
      out.println("GPI_ENABLE_FAILED reason=bad_arguments");
      return true;
    }
    GpiSettings edited = {};
    bool usedFactory = true;
    loadStoredSettingsForEdit(edited, usedFactory);
    edited.route[rowIndex].enabled = enabled ? 1 : 0;
    saveEditedSettings(edited, out, "GPI enabled flag saved; restart required.");
    return true;
  }
  if (strncmp(line, "gpi-set-pin ", 12) == 0) {
    out.println("GPI_SET_PIN_FAILED reason=unsupported_in_route_mode");
    return true;
  }
  if (strncmp(line, "gpi-set-target ", 15) == 0) {
    unsigned row = 0;
    unsigned subnet = 0;
    unsigned device = 0;
    if (sscanf(line + 15, "%u %u %u", &row, &subnet, &device) != 3 ||
        row < 1 || row > kGpiRouteCount || subnet > 7 || device > 63) {
      out.println("GPI_SET_TARGET_FAILED reason=bad_arguments");
      return true;
    }
    GpiSettings edited = {};
    bool usedFactory = true;
    loadStoredSettingsForEdit(edited, usedFactory);
    edited.route[row - 1].targetSubnet = static_cast<uint8_t>(subnet);
    edited.route[row - 1].targetDevice = static_cast<uint8_t>(device);
    saveEditedSettings(edited, out, "GPI target saved; restart required.");
    return true;
  }
  if (strncmp(line, "gpi-set-preset ", 15) == 0) {
    unsigned row = 0;
    unsigned loadPreset = 0;
    unsigned backupSlot = 0;
    if (sscanf(line + 15, "%u %u %u", &row, &loadPreset, &backupSlot) != 3 ||
        row < 1 || row > kGpiRouteCount) {
      out.println("GPI_SET_PRESET_FAILED reason=bad_arguments");
      return true;
    }
    GpiSettings edited = {};
    bool usedFactory = true;
    loadStoredSettingsForEdit(edited, usedFactory);
    edited.route[row - 1].loadPreset = static_cast<uint8_t>(loadPreset);
    edited.route[row - 1].backupSlot = static_cast<uint8_t>(backupSlot);
    saveEditedSettings(edited, out, "GPI preset settings saved; restart required.");
    return true;
  }
  if (strncmp(line, "gpi-test-press ", 15) == 0) {
    const uint8_t rowIndex = parseRouteIndex(line + 15);
    if (rowIndex == 0xFF) {
      out.println("GPI_TEST_PRESS_FAILED reason=bad_arguments");
      return true;
    }
    const char* reason = 0;
    if (!GpiManager_triggerRouteBackupRecall(rowIndex, reason)) {
      out.print("GPI_TEST_PRESS_FAILED reason=");
      out.println(reason != 0 ? reason : "route_trigger_failed");
    }
    return true;
  }
  if (strncmp(line, "gpi-test-release ", 17) == 0) {
    const uint8_t rowIndex = parseRouteIndex(line + 17);
    if (rowIndex == 0xFF) {
      out.println("GPI_TEST_RELEASE_FAILED reason=bad_arguments");
      return true;
    }
    const char* reason = 0;
    if (!GpiManager_triggerRouteRevert(rowIndex, reason)) {
      out.print("GPI_TEST_RELEASE_FAILED reason=");
      out.println(reason != 0 ? reason : "route_trigger_failed");
    }
    return true;
  }
  return false;
}

bool GpiManager::copyActiveSettings(GpiSettings& out) const {
  out = settings_;
  return true;
}

bool GpiManager::saveSettingsForRestart(const GpiSettings& settings, const char*& reason) {
  if (!store_.saveUserSettings(settings)) {
    reason = store_.lastError();
    return false;
  }
  reason = "ok";
  return true;
}

bool GpiManager::resetFactorySettings(const char*& reason) {
  if (!store_.resetUserSettingsToFactoryDefaults()) {
    reason = store_.lastError();
    return false;
  }
  reason = "ok";
  return true;
}

bool GpiManager::triggerWebRouteBackupRecall(uint8_t rowIndex, const char*& reason) {
  const char* validationReason = 0;
  uint16_t canId = 0;
  if (!validateRoute(rowIndex, validationReason, canId)) {
    reason = validationReason;
    return false;
  }
  if (!settings_.route[rowIndex].enabled) {
    reason = "disabled";
    return false;
  }
  if (!triggerRouteBackupRecall(rowIndex, "web")) {
    reason = "route_trigger_failed";
    return false;
  }
  reason = "ok";
  return true;
}

bool GpiManager::triggerWebRouteRevert(uint8_t rowIndex, const char*& reason) {
  const char* validationReason = 0;
  uint16_t canId = 0;
  if (!validateRoute(rowIndex, validationReason, canId)) {
    reason = validationReason;
    return false;
  }
  if (!settings_.route[rowIndex].enabled) {
    reason = "disabled";
    return false;
  }
  if (!triggerRouteRevert(rowIndex, "web", millis())) {
    reason = "route_trigger_failed";
    return false;
  }
  reason = "ok";
  return true;
}

bool GpiManager_handleCliCommand(const char* line, Print& out, uint32_t nowMs) {
  return gActiveManager != 0 && gActiveManager->handleCliCommand(line, out, nowMs);
}

bool GpiManager_copyActiveSettings(GpiSettings& out) {
  if (gActiveManager == 0) {
    return false;
  }
  return gActiveManager->copyActiveSettings(out);
}

bool GpiManager_saveSettingsForRestart(const GpiSettings& settings, const char*& reason) {
  if (gActiveManager == 0) {
    reason = "gpi_manager_not_started";
    return false;
  }
  return gActiveManager->saveSettingsForRestart(settings, reason);
}

bool GpiManager_resetFactorySettings(const char*& reason) {
  if (gActiveManager == 0) {
    reason = "gpi_manager_not_started";
    return false;
  }
  return gActiveManager->resetFactorySettings(reason);
}

bool GpiManager_triggerRouteBackupRecall(uint8_t rowIndex, const char*& reason) {
  if (gActiveManager == 0) {
    reason = "gpi_manager_not_started";
    return false;
  }
  return gActiveManager->triggerWebRouteBackupRecall(rowIndex, reason);
}

bool GpiManager_triggerRouteRevert(uint8_t rowIndex, const char*& reason) {
  if (gActiveManager == 0) {
    reason = "gpi_manager_not_started";
    return false;
  }
  return gActiveManager->triggerWebRouteRevert(rowIndex, reason);
}

}  // namespace r70v2
}  // namespace dbcan
