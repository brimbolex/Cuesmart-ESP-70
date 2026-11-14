#include "gpi/GpiActions.h"

#include <stdio.h>

#include "../DbCanProtocol.h"
#include "../R70GatewayV2.h"

namespace dbcan {
namespace r70v2 {

namespace {

R70GatewayV2* gGateway = 0;
Print* gLog = 0;

void logActionLine(const char* prefix,
                   uint8_t slotIndex,
                   const char* source,
                   const char* action,
                   const GpiPresetRouteConfig& input,
                   const char* reason,
                   uint16_t canId,
                   uint8_t extraValue) {
  if (gLog == 0) {
    return;
  }
  char line[256] = {};
  if (reason != 0) {
    snprintf(line,
             sizeof(line),
             "%s reason=%s slot=%u source=%s action=%s target=%u.%02u loadPreset=%u backupSlot=%u",
             prefix,
             reason,
             static_cast<unsigned>(slotIndex + 1),
             source != 0 ? source : "unknown",
             action != 0 ? action : "unknown",
             static_cast<unsigned>(input.targetSubnet),
             static_cast<unsigned>(input.targetDevice),
             static_cast<unsigned>(input.loadPreset),
             static_cast<unsigned>(input.backupSlot));
  } else {
    snprintf(line,
             sizeof(line),
             "%s slot=%u source=%s action=%s target=%u.%02u value=%u can_id=0x%03X",
             prefix,
             static_cast<unsigned>(slotIndex + 1),
             source != 0 ? source : "unknown",
             action != 0 ? action : "unknown",
             static_cast<unsigned>(input.targetSubnet),
             static_cast<unsigned>(input.targetDevice),
             static_cast<unsigned>(extraValue),
             static_cast<unsigned>(canId));
  }
  gLog->println(line);
}

bool enqueueFrame(const Frame& frame, const char*& failureReason) {
  if (gGateway == 0) {
    failureReason = "gateway_not_bound";
    return false;
  }
  return gGateway->sendDirectPrimaryMasterFrame(frame, 0, failureReason);
}

bool canSendToTarget(const GpiPresetRouteConfig& input, const char*& reason) {
  if (input.targetSubnet > 7) {
    reason = "invalid_target";
    return false;
  }
  if (input.targetDevice > 63) {
    reason = "invalid_target_mode";
    return false;
  }
  reason = 0;
  return true;
}

Address toDbAddress(const GpiPresetRouteConfig& input) {
  Address address = {};
  address.subnet = input.targetSubnet;
  address.device = input.targetDevice;
  return address;
}

bool sendFrameAndCount(const Frame& frame, uint32_t& counter, const char*& failureReason) {
  if (!enqueueFrame(frame, failureReason)) {
    if (gGateway != 0) {
      ++gGateway->mutableCounters().gpi_action_send_fail;
      ++gGateway->mutableCounters().gpi_can_tx_failed;
    }
    return false;
  }
  ++counter;
  return true;
}

void logCanTxLine(const char* label, const Frame& frame, bool ok, const char* reason) {
  if (gLog == 0) {
    return;
  }
  gLog->print("[CAN TX] ");
  gLog->print(label != 0 ? label : "gpi");
  gLog->print(" ID=0x");
  gLog->print(frame.canId, HEX);
  gLog->print(" DLC=");
  gLog->print(frame.length);
  gLog->print(" DATA=");
  for (uint8_t index = 0; index < frame.length; ++index) {
    if (index > 0) {
      gLog->print(' ');
    }
    if (frame.data[index] < 0x10) {
      gLog->print('0');
    }
    gLog->print(frame.data[index], HEX);
  }
  gLog->print(ok ? " OK" : " FAILED");
  if (!ok && reason != 0) {
    gLog->print(" reason=");
    gLog->print(reason);
  }
  gLog->println();
}

}  // namespace

void GpiActions_begin(R70GatewayV2& gateway, Print& log) {
  gGateway = &gateway;
  gLog = &log;
}

uint16_t makePrimaryMasterCanId(uint8_t subnet, uint8_t device) {
  return protocol::makeCanId(protocol::kObjectClassPrimaryMaster, Address{subnet, device});
}

bool GpiActions_triggerPress(uint8_t slotIndex,
                             const GpiPresetRouteConfig& input,
                             const char* source) {
  const char* reason = 0;
  if (!canSendToTarget(input, reason)) {
    logActionLine("GPI_ACTION_NOT_SENT", slotIndex, source, "backup_and_load", input, reason, 0,
                  input.loadPreset);
    if (gGateway != 0) {
      ++gGateway->mutableCounters().gpi_action_send_fail;
    }
    return false;
  }
  if (input.actionMode != GpiActionMode::BackupLoadRestore || gGateway == 0) {
    return true;
  }

  const Frame backupLoadFrame =
      protocol::makePresetBackupAndLoad(toDbAddress(input),
                                        input.loadPreset,
                                        input.backupSlot);
  const char* failureReason = 0;
  char label[32] = {};
  snprintf(label, sizeof(label), "route%u_backup_recall", static_cast<unsigned>(slotIndex + 1));
  if (!sendFrameAndCount(backupLoadFrame, gGateway->mutableCounters().gpi_action_backup_tx,
                         failureReason)) {
    logCanTxLine(label, backupLoadFrame, false, failureReason);
    logActionLine("GPI_ACTION_NOT_SENT", slotIndex, source, "backup_and_load", input,
                  failureReason != 0 ? failureReason : "enqueue_failed",
                  0,
                  input.loadPreset);
    return false;
  }
  ++gGateway->mutableCounters().gpi_action_load_tx;
  logCanTxLine(label, backupLoadFrame, true, 0);
  logActionLine("GPI_ACTION_TX", slotIndex, source, "backup_and_load", input, 0,
                backupLoadFrame.canId, input.loadPreset);
  return true;
}

bool GpiActions_triggerRelease(uint8_t slotIndex,
                               const GpiPresetRouteConfig& input,
                               const char* source,
                               uint8_t repeatNumber,
                               uint8_t repeatTotal) {
  const char* reason = 0;
  if (!canSendToTarget(input, reason)) {
    logActionLine("GPI_ACTION_NOT_SENT", slotIndex, source, "restore", input, reason, 0,
                  input.backupSlot);
    if (gGateway != 0) {
      ++gGateway->mutableCounters().gpi_action_send_fail;
    }
    return false;
  }
  if (input.actionMode != GpiActionMode::BackupLoadRestore || gGateway == 0) {
    return true;
  }

  const Frame restoreFrame = protocol::makePresetLoad(toDbAddress(input), input.backupSlot);
  const char* failureReason = 0;
  char label[32] = {};
  snprintf(label,
           sizeof(label),
           "route%u_revert %u/%u",
           static_cast<unsigned>(slotIndex + 1),
           static_cast<unsigned>(repeatNumber),
           static_cast<unsigned>(repeatTotal));
  if (!sendFrameAndCount(restoreFrame, gGateway->mutableCounters().gpi_action_restore_tx,
                         failureReason)) {
    logCanTxLine(label, restoreFrame, false, failureReason);
    logActionLine("GPI_ACTION_NOT_SENT", slotIndex, source, "restore", input,
                  failureReason != 0 ? failureReason : "enqueue_failed",
                  0,
                  input.backupSlot);
    return false;
  }
  ++gGateway->mutableCounters().gpi_restore_repeat_sent;
  logCanTxLine(label, restoreFrame, true, 0);
  logActionLine("GPI_ACTION_TX", slotIndex, source, "restore", input, 0, restoreFrame.canId,
                input.backupSlot);
  return true;
}

}  // namespace r70v2
}  // namespace dbcan
