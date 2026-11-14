#ifndef DB_CAN_ACTIONS_H
#define DB_CAN_ACTIONS_H

#include <Arduino.h>

#include "ConfigManager.h"
#include "DbCanTypes.h"

namespace dbcan {

enum class ServiceCommand : uint8_t {
  None = 0,
  SaveConfig,
  ClearErrors,
  ResetPeaks,
  FactoryDefaults,
  Restart
};

class ActionQueue {
 public:
  typedef ResultCode (*ExecuteActionHook)(const ActionConfig& action, void* userData);

  explicit ActionQueue(ConfigManager& configManager);

  void setExecuteActionHook(ExecuteActionHook hook, void* userData = nullptr);

  bool queueAction(uint8_t actionIndex);
  bool queueConfigSave(const AppConfig& config);
  bool queueService(ServiceCommand command);
  bool hasPendingConfigSave() const;

  void loop();

 private:
  enum class OperationKind : uint8_t {
    None = 0,
    TriggerAction,
    SaveConfig,
    ClearErrors,
    ResetPeaks,
    FactoryDefaults,
    Restart
  };

  struct Operation {
    OperationKind kind;
    uint8_t actionIndex;
    uint32_t dueAtMs;
  };

  static const uint8_t kMaxQueuedOperations = 16;

  bool enqueue(const Operation& operation);
  bool hasPendingOperation(OperationKind kind) const;
  uint8_t effectiveCapacity() const;
  void updateQueueStatus();
  void noteOverflow();
  void processAction(uint8_t actionIndex);
  void processSaveConfig();
  void processClearErrors();
  void processResetPeaks();
  void processFactoryDefaults();
  void processRestart();

  ConfigManager& configManager_;
  ExecuteActionHook executeActionHook_ = nullptr;
  void* executeActionUserData_ = nullptr;
  Operation operations_[kMaxQueuedOperations] = {};
  uint8_t head_ = 0;
  uint8_t count_ = 0;
  AppConfig stagedConfig_ = {};
  bool stagedConfigValid_ = false;
  uint32_t holdUntilMs_ = 0;
};

}  // namespace dbcan

#endif  // DB_CAN_ACTIONS_H
