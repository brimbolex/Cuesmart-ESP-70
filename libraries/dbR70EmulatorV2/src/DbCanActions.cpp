#include "DbCanActions.h"

#include <ESP.h>

namespace dbcan {

namespace {

bool timeReached(uint32_t dueAtMs, uint32_t nowMs) {
  return static_cast<int32_t>(nowMs - dueAtMs) >= 0;
}

}  // namespace

ActionQueue::ActionQueue(ConfigManager& configManager) : configManager_(configManager) {
  updateQueueStatus();
}

void ActionQueue::setExecuteActionHook(ExecuteActionHook hook, void* userData) {
  executeActionHook_ = hook;
  executeActionUserData_ = userData;
}

bool ActionQueue::queueAction(uint8_t actionIndex) {
  if (actionIndex >= kActionCount) {
    return false;
  }

  Operation operation = {};
  operation.kind = OperationKind::TriggerAction;
  operation.actionIndex = actionIndex;
  operation.dueAtMs = 0;
  return enqueue(operation);
}

bool ActionQueue::queueConfigSave(const AppConfig& config) {
  stagedConfig_ = config;
  stagedConfigValid_ = true;

  if (hasPendingOperation(OperationKind::SaveConfig)) {
    return true;
  }

  Operation operation = {};
  operation.kind = OperationKind::SaveConfig;
  return enqueue(operation);
}

bool ActionQueue::queueService(ServiceCommand command) {
  Operation operation = {};

  switch (command) {
    case ServiceCommand::SaveConfig:
      if (hasPendingOperation(OperationKind::SaveConfig)) {
        return true;
      }
      operation.kind = OperationKind::SaveConfig;
      break;
    case ServiceCommand::ClearErrors:
      operation.kind = OperationKind::ClearErrors;
      break;
    case ServiceCommand::ResetPeaks:
      operation.kind = OperationKind::ResetPeaks;
      break;
    case ServiceCommand::FactoryDefaults:
      operation.kind = OperationKind::FactoryDefaults;
      break;
    case ServiceCommand::Restart:
      operation.kind = OperationKind::Restart;
      operation.dueAtMs = millis() + 100U;
      break;
    case ServiceCommand::None:
    default:
      return false;
  }

  return enqueue(operation);
}

bool ActionQueue::hasPendingConfigSave() const {
  return stagedConfigValid_ || hasPendingOperation(OperationKind::SaveConfig);
}

void ActionQueue::loop() {
  updateQueueStatus();

  const uint32_t nowMs = millis();
  if (holdUntilMs_ != 0 && !timeReached(holdUntilMs_, nowMs)) {
    return;
  }
  holdUntilMs_ = 0;

  if (count_ == 0) {
    return;
  }

  const Operation operation = operations_[head_];
  if (operation.dueAtMs != 0 && !timeReached(operation.dueAtMs, nowMs)) {
    return;
  }

  head_ = static_cast<uint8_t>((head_ + 1) % kMaxQueuedOperations);
  --count_;
  updateQueueStatus();

  switch (operation.kind) {
    case OperationKind::TriggerAction:
      processAction(operation.actionIndex);
      break;
    case OperationKind::SaveConfig:
      processSaveConfig();
      break;
    case OperationKind::ClearErrors:
      processClearErrors();
      break;
    case OperationKind::ResetPeaks:
      processResetPeaks();
      break;
    case OperationKind::FactoryDefaults:
      processFactoryDefaults();
      break;
    case OperationKind::Restart:
      processRestart();
      break;
    case OperationKind::None:
    default:
      break;
  }
}

bool ActionQueue::enqueue(const Operation& operation) {
  if (count_ >= effectiveCapacity() || count_ >= kMaxQueuedOperations) {
    noteOverflow();
    return false;
  }

  const uint8_t tail = static_cast<uint8_t>((head_ + count_) % kMaxQueuedOperations);
  operations_[tail] = operation;
  ++count_;
  updateQueueStatus();
  return true;
}

bool ActionQueue::hasPendingOperation(OperationKind kind) const {
  for (uint8_t index = 0; index < count_; ++index) {
    const uint8_t slot = static_cast<uint8_t>((head_ + index) % kMaxQueuedOperations);
    if (operations_[slot].kind == kind) {
      return true;
    }
  }
  return false;
}

uint8_t ActionQueue::effectiveCapacity() const {
  const uint8_t configured = configManager_.config().gateway.actionQueueLength;
  if (configured == 0) {
    return 1;
  }
  return configured > kMaxQueuedOperations ? kMaxQueuedOperations : configured;
}

void ActionQueue::updateQueueStatus() {
  QueueStatus& status = configManager_.runtimeStatus().actionQueue;
  status.capacity = effectiveCapacity();
  status.currentFill = count_;
  if (status.currentFill > status.peakFill) {
    status.peakFill = status.currentFill;
  }
}

void ActionQueue::noteOverflow() {
  RuntimeStatus& status = configManager_.runtimeStatus();
  status.errors.actionQueueOverflow = true;
  ++status.actionQueue.overflowCount;
  ++status.actionQueue.droppedCount;
}

void ActionQueue::processAction(uint8_t actionIndex) {
  if (actionIndex >= kActionCount) {
    return;
  }

  const ActionConfig& action = configManager_.config().actions[actionIndex];
  if (!action.enabled || executeActionHook_ == nullptr) {
    return;
  }

  const ResultCode result = executeActionHook_(action, executeActionUserData_);
  if (result == ResultCode::Ok && action.delayAfterMs > 0) {
    holdUntilMs_ = millis() + action.delayAfterMs;
  }
}

void ActionQueue::processSaveConfig() {
  if (stagedConfigValid_) {
    configManager_.config() = stagedConfig_;
    stagedConfigValid_ = false;
  }

  RuntimeStatus& status = configManager_.runtimeStatus();
  const bool saved = configManager_.save();
  status.errors.configError = !saved;
  status.canRxQueue.capacity = configManager_.config().gateway.canRxQueueLength;
  status.canTxQueue.capacity = configManager_.config().gateway.canTxQueueLength;
  status.ethRxQueue.capacity = configManager_.config().gateway.ethRxQueueLength;
  status.ethTxQueue.capacity = configManager_.config().gateway.ethTxQueueLength;
  updateQueueStatus();
}

void ActionQueue::processClearErrors() {
  configManager_.runtimeStatus().errors = {};
}

void ActionQueue::processResetPeaks() {
  RuntimeStatus& status = configManager_.runtimeStatus();
  status.canRxQueue.peakFill = status.canRxQueue.currentFill;
  status.canTxQueue.peakFill = status.canTxQueue.currentFill;
  status.ethRxQueue.peakFill = status.ethRxQueue.currentFill;
  status.ethTxQueue.peakFill = status.ethTxQueue.currentFill;
  status.actionQueue.peakFill = status.actionQueue.currentFill;
}

void ActionQueue::processFactoryDefaults() {
  stagedConfigValid_ = false;
  configManager_.resetToDefaults();
  processSaveConfig();
}

void ActionQueue::processRestart() {
  ESP.restart();
}

}  // namespace dbcan
