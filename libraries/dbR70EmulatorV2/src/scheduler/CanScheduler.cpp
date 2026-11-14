#include "CanScheduler.h"

#include "../DbCanProtocol.h"

namespace dbcan {
namespace r70v2 {

CanScheduler::CanScheduler()
    : transport_(0),
      minSpacingUsNormal_(kPacingSafeNormalUs),
      minSpacingUsWrite_(kPacingSafeWriteUs),
      minSpacingUsScan_(kPacingSafeScanUs),
      lastTxUs_(0),
      identWindowMs_(550),
      identQuietHoldMs_(75),
      enableWriteRetry_(false),
      writeAckTimeoutMs_(500),
      identWindowUntilMs_(0),
      identQuietUntilMs_(0),
      identSubnet_(0),
      lastNormalReadSubnet_(0xFFu),
      consecutiveNormalReadSubnetCount_(0),
      debugCli_(0),
      canTxObserver_(0),
      canTxObserverContext_(0) {
  for (uint8_t index = 0; index < kFailureBucketCount; ++index) {
    canTxEnqueueFailedBySubnetDevice_[index] = {};
    canTxEnqueueFailedByObject_[index] = {};
  }
  for (uint8_t index = 0; index < kPendingMuteWriteCount; ++index) {
    pendingMuteWrites_[index] = {};
  }
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    currentConsecutiveReadsBySubnet_[subnet] = 0;
  }
}

void CanScheduler::setCanTxObserver(CanTxObserverFn observer, void* context) {
  canTxObserver_ = observer;
  canTxObserverContext_ = context;
}

void CanScheduler::begin(CanTwaiTransport& transport,
                         uint32_t minSpacingUsNormal,
                         uint32_t minSpacingUsWrite,
                         uint32_t minSpacingUsScan,
                         uint32_t identWindowMs,
                         uint32_t identQuietHoldMs,
                         bool enableWriteRetry,
                         uint32_t writeAckTimeoutMs,
                         const DebugCli& debugCli) {
  transport_ = &transport;
  minSpacingUsNormal_ = minSpacingUsNormal == 0 ? kPacingSafeNormalUs : minSpacingUsNormal;
  minSpacingUsWrite_ = minSpacingUsWrite == 0 ? kPacingSafeWriteUs : minSpacingUsWrite;
  minSpacingUsScan_ = minSpacingUsScan == 0 ? kPacingSafeScanUs : minSpacingUsScan;
  identWindowMs_ = identWindowMs == 0 ? 550 : identWindowMs;
  identQuietHoldMs_ = identQuietHoldMs;
  enableWriteRetry_ = enableWriteRetry;
  writeAckTimeoutMs_ = writeAckTimeoutMs == 0 ? 500 : writeAckTimeoutMs;
  queues_.clear();
  lastTxUs_ = 0;
  identWindowUntilMs_ = 0;
  identQuietUntilMs_ = 0;
  identSubnet_ = 0;
  lastNormalReadSubnet_ = 0xFFu;
  consecutiveNormalReadSubnetCount_ = 0;
  debugCli_ = &debugCli;
  for (uint8_t index = 0; index < kPendingMuteWriteCount; ++index) {
    pendingMuteWrites_[index] = {};
  }
  for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
    currentConsecutiveReadsBySubnet_[subnet] = 0;
  }
}

void CanScheduler::noteQueueDepth(DebugCounters& counters, uint32_t nowMs) const {
  counters.can_tx_queue_depth = queues_.size();
  counters.can_tx_write_queue_depth = queues_.writeSize();
  counters.can_tx_maintenance_queue_depth = queues_.maintenanceSize();
  counters.can_tx_ident_queue_depth = queues_.identSize();
  counters.can_tx_normal_queue_depth = queues_.normalReadSize();
  counters.normal_read_queue_depth = queues_.normalReadSize();
  counters.normal_read_queue_oldest_age_ms = queues_.oldestAgeMs(QueueClass::NormalRead, nowMs);
  if (counters.can_tx_queue_depth > counters.can_tx_queue_highwater) {
    counters.can_tx_queue_highwater = counters.can_tx_queue_depth;
  }
  if (counters.normal_read_queue_depth > counters.normal_read_queue_highwater) {
    counters.normal_read_queue_highwater = counters.normal_read_queue_depth;
  }
}

void CanScheduler::noteFailureKey(FailureKeyCount* buckets, uint8_t bucketCount, uint16_t key) {
  for (uint8_t index = 0; index < bucketCount; ++index) {
    if (buckets[index].used && buckets[index].key == key) {
      ++buckets[index].count;
      return;
    }
  }
  for (uint8_t index = 0; index < bucketCount; ++index) {
    if (!buckets[index].used) {
      buckets[index].used = true;
      buckets[index].key = key;
      buckets[index].count = 1;
      return;
    }
  }
  ++buckets[0].count;
}

void CanScheduler::noteEnqueueFailure(const NormalizedRequest& request, DebugCounters& counters) {
  ++counters.can_tx_queue_full_total;
  const uint16_t subnetDeviceKey =
      static_cast<uint16_t>((static_cast<uint16_t>(request.canId.subnet) << 6) | request.canId.device);
  noteFailureKey(canTxEnqueueFailedBySubnetDevice_, kFailureBucketCount, subnetDeviceKey);
  if (request.frame.length >= 3) {
    const uint16_t objectKey =
        static_cast<uint16_t>((static_cast<uint16_t>(request.frame.data[1]) << 8) | request.frame.data[2]);
    noteFailureKey(canTxEnqueueFailedByObject_, kFailureBucketCount, objectKey);
  }
}

bool CanScheduler::isIdentScanFrame(const NormalizedRequest& request) {
  return request.valid &&
         request.canId.objectClass == 1 &&
         request.canId.device == 0 &&
         request.frame.length == 2 &&
         request.frame.data[0] == 0x00 &&
         request.frame.data[1] == 0x01;
}

bool CanScheduler::isWriteCandidateFrame(const NormalizedRequest& request) {
  if (!request.valid) {
    return false;
  }
  if (request.frame.length > 0 && request.frame.data[0] == protocol::kAdminPresetCommand) {
    return true;
  }
  if (request.admin.valid) {
    if (request.admin.functionCode == FunctionCode::WriteObject) {
      return true;
    }
    // A field can be both readable and writable.  Once the admin header has
    // positively identified a read, the field-id fallback below must not
    // promote that read into the high-priority write queue.  Keep the fallback
    // only for legacy or partially decoded frames without a usable function.
    if (isReadFunction(request.admin.functionCode)) {
      return false;
    }
  }
  switch (request.fieldId) {
    case FieldId::settings_power:
    case FieldId::config_cut_a:
    case FieldId::config_cut_b:
    case FieldId::config_hfa_a:
    case FieldId::config_hfa_b:
    case FieldId::config_delay_on_a:
    case FieldId::config_delay_on_b:
    case FieldId::config_mute_a:
    case FieldId::config_mute_b:
    case FieldId::config_gain_a:
    case FieldId::config_gain_b:
    case FieldId::config_delay_value_a:
    case FieldId::config_delay_value_b:
      return true;
    default:
      return false;
  }
}

bool CanScheduler::isCoalescingEligibleRead(const NormalizedRequest& request) {
  return request.valid &&
         request.tcpWrappedCan &&
         request.origin == RequestOrigin::Tcp &&
         request.admin.valid &&
         request.admin.functionCode == FunctionCode::ReadObject &&
         request.admin.telegramType == TelegramType::Unstructured &&
         !isIdentScanFrame(request) &&
         !isWriteCandidateFrame(request);
}

bool CanScheduler::isMuteWriteFrame(const NormalizedRequest& request, char& channel, uint8_t& value) {
  channel = '?';
  value = 0;
  if (!request.valid || request.frame.length < 4 || !request.admin.valid ||
      request.admin.functionCode != FunctionCode::WriteObject ||
      request.admin.telegramType != TelegramType::Unstructured) {
    return false;
  }
  if ((request.frame.data[1] == 0x22 || request.frame.data[1] == 0x42) &&
      request.frame.data[2] == 0x05 &&
      (request.frame.data[3] == 0x00 || request.frame.data[3] == 0x01)) {
    channel = request.frame.data[1] == 0x22 ? 'A' : 'B';
    value = request.frame.data[3];
    return true;
  }
  return false;
}

bool CanScheduler::isMuteAckFrame(const NormalizedRequest& request, char& channel, uint8_t& value) {
  channel = '?';
  value = 0;
  if (!request.valid || request.frame.length < 4 || !request.admin.valid) {
    return false;
  }
  if ((request.frame.data[1] == 0x22 || request.frame.data[1] == 0x42) &&
      request.frame.data[2] == 0x05 &&
      (request.frame.data[3] == 0x00 || request.frame.data[3] == 0x01)) {
    channel = request.frame.data[1] == 0x22 ? 'A' : 'B';
    value = request.frame.data[3];
    return true;
  }
  return false;
}

QueueClass CanScheduler::classifyQueueClass(const NormalizedRequest& request) const {
  if (isWriteCandidateFrame(request)) {
    return QueueClass::Write;
  }
  if (isIdentScanFrame(request)) {
    return QueueClass::Ident;
  }
  return QueueClass::NormalRead;
}

void CanScheduler::noteCanTx(const NormalizedRequest& request, uint32_t nowMs, DebugCounters& counters) {
  if (isIdentScanFrame(request)) {
    identWindowUntilMs_ = nowMs + identWindowMs_;
    identQuietUntilMs_ = nowMs + identQuietHoldMs_;
    identSubnet_ = request.canId.subnet;
    ++counters.ident_tx_count;
    if (debugCli_ != 0) {
      debugCli_->logIdentTx(request.canId.subnet, request.frame);
    }
    return;
  }

  if (isWriteCandidateFrame(request)) {
    ++counters.write_tx_count;
  }

  if (request.valid &&
      request.fieldId == FieldId::fixed_firmware_version &&
      request.admin.valid &&
      request.admin.functionCode == FunctionCode::ReadObject &&
      request.admin.telegramType == TelegramType::Segmented) {
    ++counters.firmware_segment_can_tx;
  }
  if (request.valid &&
      request.admin.valid &&
      request.admin.functionCode == FunctionCode::ReadObject &&
      request.admin.telegramType == TelegramType::Unstructured) {
    if (request.fieldId == FieldId::config_speaker_id_a ||
        request.fieldId == FieldId::config_speaker_id_b) {
      ++counters.speaker_id_can_tx;
    } else if (request.fieldId == FieldId::settings_input_mode) {
      ++counters.input_mode_can_tx;
    } else if (request.fieldId == FieldId::settings_data_link_preset) {
      ++counters.datalink_preset_can_tx;
    }
  }

  char channel = '?';
  uint8_t value = 0;
  if (!isMuteWriteFrame(request, channel, value)) {
    return;
  }

  ++counters.mute_write_tx_count;
  if (debugCli_ != 0) {
    debugCli_->logMuteWriteTx(request.canId.subnet, request.canId.device, channel, value, request.frame);
  }

  PendingMuteWrite* empty = 0;
  for (uint8_t index = 0; index < kPendingMuteWriteCount; ++index) {
    PendingMuteWrite& pending = pendingMuteWrites_[index];
    if (pending.used &&
        pending.subnet == request.canId.subnet &&
        pending.device == request.canId.device &&
        pending.channel == channel) {
      pending.value = value;
      pending.sentMs = nowMs;
      return;
    }
    if (!pending.used && empty == 0) {
      empty = &pending;
    }
  }

  if (empty != 0) {
    empty->used = true;
    empty->subnet = request.canId.subnet;
    empty->device = request.canId.device;
    empty->channel = channel;
    empty->value = value;
    empty->sentMs = nowMs;
  }
}

void CanScheduler::expireMuteWrites(uint32_t nowMs, DebugCounters& counters) {
  for (uint8_t index = 0; index < kPendingMuteWriteCount; ++index) {
    PendingMuteWrite& pending = pendingMuteWrites_[index];
    if (!pending.used || static_cast<int32_t>(nowMs - (pending.sentMs + writeAckTimeoutMs_)) < 0) {
      continue;
    }
    ++counters.write_timeout_count;
    ++counters.mute_write_timeout_count;
    if (debugCli_ != 0) {
      debugCli_->logMuteWriteTimeout(pending.subnet, pending.device, pending.channel,
                                     pending.value, nowMs - pending.sentMs);
    }
    pending = {};
  }
}

bool CanScheduler::enqueue(const NormalizedRequest& request,
                           uint8_t tcpNode,
                           bool expectReply,
                           uint32_t nowMs,
                           DebugCounters& counters,
                           const char*& failureReason) {
  failureReason = 0;
  const QueueClass queueClass = classifyQueueClass(request);
  if (queueClass == QueueClass::Ident && queues_.containsExact(QueueClass::Ident, request)) {
    return true;
  }
  if (queueClass == QueueClass::NormalRead &&
      isCoalescingEligibleRead(request) &&
      queues_.containsExact(QueueClass::NormalRead, request)) {
    ++counters.read_coalesced_duplicate_count;
    if (debugCli_ != 0 && (counters.read_coalesced_duplicate_count % 32u) == 0) {
      debugCli_->logReadCoalesced(request, counters.read_coalesced_duplicate_count);
    }
    noteQueueDepth(counters, nowMs);
    return true;
  }

  QueueEntry entry = {};
  entry.request = request;
  entry.tcpNode = tcpNode;
  entry.expectReply = expectReply;
  entry.enqueuedMs = nowMs;
  if (!queues_.enqueue(queueClass, entry)) {
    failureReason = "queue_full";
    noteEnqueueFailure(request, counters);
    ++counters.can_tx_enqueue_failed_queue_full;
    if (queueClass == QueueClass::Write) {
      ++counters.write_queue_full;
      if (debugCli_ != 0) {
        debugCli_->logWriteQueueFull(request.frame);
      }
    } else if (queueClass == QueueClass::Ident) {
      ++counters.ident_queue_full;
    } else {
      ++counters.normal_queue_full;
    }
    noteQueueDepth(counters, nowMs);
    return false;
  }
  if (queueClass == QueueClass::Ident && debugCli_ != 0) {
    debugCli_->logIdentQueued(request.canId.subnet, request.frame);
  }
  char channel = '?';
  uint8_t value = 0;
  if (isMuteWriteFrame(request, channel, value) && debugCli_ != 0) {
    debugCli_->logMuteWriteQueued(request.canId.subnet, request.canId.device, channel, value,
                                  request.frame);
  }
  noteQueueDepth(counters, nowMs);
  return true;
}

bool CanScheduler::enqueueMaintenance(const NormalizedRequest& request,
                                      uint32_t nowMs,
                                      DebugCounters& counters,
                                      const char*& failureReason) {
  failureReason = 0;
  if (!request.valid ||
      !request.admin.valid ||
      !isReadFunction(request.admin.functionCode) ||
      isWriteCandidateFrame(request) ||
      isIdentScanFrame(request)) {
    failureReason = "invalid_maintenance_refresh";
    return false;
  }
  if (queues_.containsExact(QueueClass::Maintenance, request)) {
    noteQueueDepth(counters, nowMs);
    return true;
  }

  QueueEntry entry = {};
  entry.request = request;
  entry.tcpNode = 0;
  entry.expectReply = false;
  entry.enqueuedMs = nowMs;
  if (!queues_.enqueue(QueueClass::Maintenance, entry)) {
    failureReason = "maintenance_queue_full";
    noteEnqueueFailure(request, counters);
    ++counters.can_tx_enqueue_failed_queue_full;
    ++counters.maintenance_queue_full;
    noteQueueDepth(counters, nowMs);
    return false;
  }

  noteQueueDepth(counters, nowMs);
  return true;
}

bool CanScheduler::enqueueBootstrapRetry(const NormalizedRequest& request,
                                         uint32_t nowMs,
                                         DebugCounters& counters,
                                         const char*& failureReason) {
  failureReason = 0;
  if (!request.valid ||
      !request.admin.valid ||
      !isReadFunction(request.admin.functionCode) ||
      isWriteCandidateFrame(request) ||
      isIdentScanFrame(request)) {
    failureReason = "invalid_bootstrap_retry";
    return false;
  }
  if (queues_.containsExact(QueueClass::NormalRead, request)) {
    return true;
  }

  QueueEntry entry = {};
  entry.request = request;
  entry.tcpNode = 0;
  entry.expectReply = false;
  entry.enqueuedMs = nowMs;
  if (!queues_.enqueueFront(QueueClass::NormalRead, entry)) {
    failureReason = "queue_full";
    noteEnqueueFailure(request, counters);
    ++counters.can_tx_enqueue_failed_queue_full;
    ++counters.normal_queue_full;
    noteQueueDepth(counters, nowMs);
    return false;
  }

  noteQueueDepth(counters, nowMs);
  return true;
}

void CanScheduler::tick(GatewayState& state, uint32_t nowMs, DebugCounters& counters) {
  if (transport_ == 0) {
    return;
  }
  expireMuteWrites(nowMs, counters);

  QueueEntry entry = {};
  QueueClass queueClass = QueueClass::NormalRead;
  if (!queues_.peek(entry, queueClass)) {
    noteQueueDepth(counters, nowMs);
    return;
  }

  const uint32_t spacingUs = queueClass == QueueClass::Write ? minSpacingUsWrite_ :
                             (queueClass == QueueClass::Ident ? minSpacingUsScan_ : minSpacingUsNormal_);
  const uint32_t nowUs = micros();
  // Unsigned elapsed-time comparison remains correct across micros() wrap and
  // after idle periods longer than INT32_MAX microseconds (~35.8 minutes).
  if (lastTxUs_ != 0 && static_cast<uint32_t>(nowUs - lastTxUs_) < spacingUs) {
    return;
  }

  if (!queues_.dequeue(entry, &queueClass)) {
    noteQueueDepth(counters, nowMs);
    return;
  }
  noteQueueDepth(counters, nowMs);

  if (transport_->send(entry.request.frame)) {
    ++counters.can_tx_frames;
    if (debugCli_ != 0) {
      debugCli_->logCanFrame("CAN_TX", entry.request.frame, &entry.request);
    }
    if (canTxObserver_ != 0) {
      canTxObserver_(canTxObserverContext_, entry.request, queueClass, entry.enqueuedMs, nowMs);
    }
    if (isReadFunction(entry.request.admin.functionCode)) {
      ++counters.can_poll_scheduled;
      state.noteCanTxPollForNode(entry.request.canId.subnet, entry.request.canId.device, nowMs);
    }
    if (entry.expectReply && isReadFunction(entry.request.admin.functionCode)) {
      state.addPending(entry.request, entry.tcpNode, nowMs, 1000);
    }
    if ((queueClass == QueueClass::NormalRead || queueClass == QueueClass::Maintenance) &&
        entry.request.valid &&
        entry.request.canId.device != 0 &&
        isReadFunction(entry.request.admin.functionCode)) {
      for (uint8_t subnet = 0; subnet < DebugCounters::kSubnetCount; ++subnet) {
        currentConsecutiveReadsBySubnet_[subnet] = 0;
      }
      if (entry.request.canId.subnet == lastNormalReadSubnet_) {
        if (consecutiveNormalReadSubnetCount_ != 0xFFu) {
          ++consecutiveNormalReadSubnetCount_;
        }
      } else {
        lastNormalReadSubnet_ = entry.request.canId.subnet;
        consecutiveNormalReadSubnetCount_ = 1u;
      }
      if (entry.request.canId.subnet < DebugCounters::kSubnetCount) {
        currentConsecutiveReadsBySubnet_[entry.request.canId.subnet] = consecutiveNormalReadSubnetCount_;
      }
    }
    noteCanTx(entry.request, nowMs, counters);
    lastTxUs_ = nowUs;
  } else if (entry.request.tcpWrappedCan && debugCli_ != 0) {
    debugCli_->logTcpCanTxFail(entry.request, "transport_send_false");
  } else if (debugCli_ != 0) {
    debugCli_->logTcpCanTxFail(entry.request, "transport_send_false");
  }
}

void CanScheduler::observeCanRx(const NormalizedRequest& request, uint32_t nowMs, DebugCounters& counters) {
  (void)nowMs;
  if (!request.valid) {
    return;
  }

  if (request.canId.objectClass == 2 &&
      request.frame.length > 0 &&
      request.frame.data[0] == 0x00 &&
      request.canId.subnet < DebugCounters::kSubnetCount) {
    ++counters.ident_rx_count;
    ++counters.ident_rx_count_by_subnet[request.canId.subnet];
    if (debugCli_ != 0) {
      debugCli_->logIdentRx(request.canId.subnet, request.canId.device, request.frame);
    }
  }

  char channel = '?';
  uint8_t value = 0;
  if (!isMuteAckFrame(request, channel, value)) {
    return;
  }

  for (uint8_t index = 0; index < kPendingMuteWriteCount; ++index) {
    PendingMuteWrite& pending = pendingMuteWrites_[index];
    if (!pending.used ||
        pending.subnet != request.canId.subnet ||
        pending.device != request.canId.device ||
        pending.channel != channel ||
        pending.value != value) {
      continue;
    }
    ++counters.mute_write_ack_count;
    ++counters.write_ack_count;
    if (debugCli_ != 0) {
      debugCli_->logMuteWriteAck(request.canId.subnet, request.canId.device, channel, value,
                                 request.frame);
    }
    pending = {};
    return;
  }
}

uint32_t CanScheduler::currentSpacingUs(uint32_t nowMs) const {
  QueueEntry entry = {};
  QueueClass queueClass = QueueClass::NormalRead;
  if (!queues_.peek(entry, queueClass)) {
    return minSpacingUsNormal_;
  }
  if (queueClass == QueueClass::Write) {
    return minSpacingUsWrite_;
  }
  if (queueClass == QueueClass::Ident) {
    return minSpacingUsScan_;
  }
  return minSpacingUsNormal_;
}

bool CanScheduler::identPhaseActive(uint32_t nowMs) const {
  return static_cast<int32_t>(nowMs - identWindowUntilMs_) < 0;
}

uint8_t CanScheduler::identSubnet() const {
  return identSubnet_;
}

uint32_t CanScheduler::identWindowRemainingMs(uint32_t nowMs) const {
  if (!identPhaseActive(nowMs)) {
    return 0;
  }
  return identWindowUntilMs_ - nowMs;
}

uint16_t CanScheduler::queueDepth() const {
  return queues_.size();
}

uint16_t CanScheduler::writeQueueDepth() const {
  return queues_.writeSize();
}

uint16_t CanScheduler::maintenanceQueueDepth() const {
  return queues_.maintenanceSize();
}

uint16_t CanScheduler::identQueueDepth() const {
  return queues_.identSize();
}

uint16_t CanScheduler::normalQueueDepth() const {
  return queues_.normalReadSize();
}

uint8_t CanScheduler::consecutiveNormalReadsForSubnet(uint8_t subnet) const {
  return subnet < DebugCounters::kSubnetCount ? currentConsecutiveReadsBySubnet_[subnet] : 0u;
}

uint16_t CanScheduler::queueHighWatermark() const {
  return static_cast<uint16_t>((HotWarmColdQueues::kTotalCapacity * 3u) / 4u);
}

uint16_t CanScheduler::queueLowWatermark() const {
  return static_cast<uint16_t>((HotWarmColdQueues::kTotalCapacity * 2u) / 5u);
}

uint32_t CanScheduler::normalReadQueueOldestAgeMs(uint32_t nowMs) const {
  return queues_.oldestAgeMs(QueueClass::NormalRead, nowMs);
}

}  // namespace r70v2
}  // namespace dbcan
