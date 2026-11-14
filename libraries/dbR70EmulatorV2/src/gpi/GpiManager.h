#ifndef DB_R70_V2_GPI_MANAGER_H
#define DB_R70_V2_GPI_MANAGER_H

#include <Print.h>
#include <Stream.h>
#include <stdint.h>

#include "gpi/GpiConfig.h"

namespace dbcan {
namespace r70v2 {

class R70GatewayV2;

class GpiManager {
 public:
  GpiManager();

  bool begin(R70GatewayV2& gateway, Stream& log);
  void poll(uint32_t nowMs);
  bool handleCliCommand(const char* line, Print& out, uint32_t nowMs);
  bool copyActiveSettings(GpiSettings& out) const;
  bool saveSettingsForRestart(const GpiSettings& settings, const char*& reason);
  bool resetFactorySettings(const char*& reason);
  bool triggerWebRouteBackupRecall(uint8_t rowIndex, const char*& reason);
  bool triggerWebRouteRevert(uint8_t rowIndex, const char*& reason);

 private:
  struct PhysicalInputRuntime {
    bool pinConfigured;
    bool rawActive;
    bool stableActive;
    uint32_t lastRawChangeMs;
    uint32_t lastEdgeMs;
  };

  struct RouteRuntime {
    bool releaseRepeatActive;
    uint8_t releaseRepeatsRemaining;
    uint32_t nextReleaseRepeatMs;
    const char* releaseRepeatSource;
  };

  bool loadActiveSettings(bool& usedFactory);
  bool validateRoute(uint8_t rowIndex, const char*& reason, uint16_t& canId) const;
  void logRouteConfigLine(uint8_t rowIndex, bool valid, const char* reason, uint16_t canId) const;
  void configurePhysicalInput(uint8_t physicalIndex, uint32_t nowMs);
  bool readRawActive(uint8_t physicalIndex) const;
  void cancelRouteReleaseRepeats(uint8_t rowIndex);
  void handlePhysicalPress(uint8_t physicalIndex, const char* source, uint32_t nowMs);
  void handlePhysicalRelease(uint8_t physicalIndex, const char* source, uint32_t nowMs);
  void serviceReleaseRepeats(uint32_t nowMs);
  uint8_t countMatchingRoutes(uint8_t physicalGpi) const;
  bool triggerRouteBackupRecall(uint8_t rowIndex, const char* source);
  bool triggerRouteRevert(uint8_t rowIndex, const char* source, uint32_t nowMs);
  void printSettings(Print& out) const;
  void printStoredSettings(Print& out);
  bool loadStoredSettingsForEdit(GpiSettings& settings, bool& usedFactory);
  void updateEnabledMaskCounter();
  void updatePendingRepeatsCounter();
  bool saveEditedSettings(const GpiSettings& settings, Print& out, const char* successText);
  void noteInvalidConfigReason(const char* reason);

  R70GatewayV2* gateway_;
  Stream* log_;
  GpiSettingsStore store_;
  GpiSettings settings_;
  bool settingsFromFactory_;
  PhysicalInputRuntime physicalRuntime_[kGpiPhysicalCount];
  RouteRuntime routeRuntime_[kGpiRouteCount];
  bool started_;
};

bool GpiManager_handleCliCommand(const char* line, Print& out, uint32_t nowMs);
bool GpiManager_copyActiveSettings(GpiSettings& out);
bool GpiManager_saveSettingsForRestart(const GpiSettings& settings, const char*& reason);
bool GpiManager_resetFactorySettings(const char*& reason);
bool GpiManager_triggerRouteBackupRecall(uint8_t rowIndex, const char*& reason);
bool GpiManager_triggerRouteRevert(uint8_t rowIndex, const char*& reason);

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_GPI_MANAGER_H
