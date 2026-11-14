#include <Arduino.h>
#include "SerialDiagnostics.h"
#include <Adafruit_NeoPixel.h>
#include <esp_system.h>

#ifndef ENABLE_GENTLE_STATUS_SHAPING
#define ENABLE_GENTLE_STATUS_SHAPING 1
#endif

#ifndef ENABLE_BOOTSTRAP_PROTECTION
#define ENABLE_BOOTSTRAP_PROTECTION 1
#endif

#ifndef ENABLE_BOOTSTRAP_RETRY
#define ENABLE_BOOTSTRAP_RETRY 1
#endif

#ifndef ENABLE_SEGMENTED_DAMPENING
#define ENABLE_SEGMENTED_DAMPENING 0
#endif

#ifndef ENABLE_FANOUT_ATTACH
#define ENABLE_FANOUT_ATTACH 1
#endif

#include <dbR70EmulatorV2.h>
#include <Log.h>
#include <gpi/GpiManager.h>
#include <network/NetworkConfig.h>
#include <web/WebConfigServer.h>

using namespace dbcan::r70v2;

#define ENABLE_GPI_MODULE 1

#define R70_BUILD_ID "r1-read-history-v156"

#ifndef GPI_DIAG
#define GPI_DIAG 0
#endif

static const gpio_num_t CAN_TX_PIN = GPIO_NUM_2;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_1;
// The normal reset button is wired directly between EN/RST and GND.
// It deliberately consumes no GPIO and therefore needs no polling here.
static const uint8_t FACTORY_RESET_PIN = 4;
static const uint8_t RGB_LED_PIN = 21;
static const uint8_t RGB_LED_COUNT = 1;
static const uint8_t RGB_LED_BRIGHTNESS = 255;
static const uint32_t FACTORY_RESET_HOLD_MS = 5000;
static const uint32_t RGB_STATUS_INTERVAL_MS = 40;

R70GatewayV2 gateway;
Adafruit_NeoPixel statusLeds(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
#if ENABLE_GPI_MODULE
GpiManager gpiManager;
#endif
uint32_t lastCountersMs = 0;
uint32_t lastWebPollMs = 0;
uint32_t factoryResetPressedSinceMs = 0;
uint32_t lastRgbStatusMs = 0;
uint32_t lastRgbCanFrames = 0;
uint32_t canFlashStartedMs = 0;
bool canFlashActive = false;
bool factoryResetTriggered = false;
#if ENABLE_SERIAL_DIAGNOSTICS && GPI_DIAG
uint32_t lastGpiDiagHeartbeatMs = 0;
bool lastTcpConnected = false;
uint32_t lastLoopAliveMs = 0;
#endif

namespace {

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:
      return "UNKNOWN";
    case ESP_RST_POWERON:
      return "POWERON_RESET";
    case ESP_RST_EXT:
      return "EXT_RESET";
    case ESP_RST_SW:
      return "SW_CPU_RESET";
    case ESP_RST_PANIC:
      return "PANIC_RESET";
    case ESP_RST_INT_WDT:
      return "INT_WDT_RESET";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT_RESET";
    case ESP_RST_WDT:
      return "TG_WDT_RESET";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP_RESET";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT_RESET";
    case ESP_RST_SDIO:
      return "SDIO_RESET";
    case ESP_RST_USB:
      return "USB_RESET";
    case ESP_RST_JTAG:
      return "JTAG_RESET";
    case ESP_RST_EFUSE:
      return "EFUSE_RESET";
    case ESP_RST_PWR_GLITCH:
      return "POWER_GLITCH_RESET";
    case ESP_RST_CPU_LOCKUP:
      return "CPU_LOCKUP_RESET";
    default:
      return "UNMAPPED_RESET";
  }
}

void printBootStage(const char* stage) {
  LOG_BLOCK(LOG_LEVEL_DEBUG, {
    DiagnosticSerial.print("[BOOT_STAGE] ");
    DiagnosticSerial.println(stage != 0 ? stage : "unknown");
  });
}

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
  statusLeds.setPixelColor(0, red, green, blue);
  statusLeds.show();
}

void blinkFactoryResetLed() {
  for (uint8_t i = 0; i < 3; ++i) {
    setStatusLed(128, 0, 255);
    delay(200);
    setStatusLed(0, 0, 0);
    delay(200);
  }
}

void beginFrontPanelIo() {
  pinMode(FACTORY_RESET_PIN, INPUT_PULLUP);
  statusLeds.begin();
  statusLeds.setBrightness(RGB_LED_BRIGHTNESS);
  setStatusLed(255, 0, 0);  // Ethernet not ready yet.
  WebConfigServer_setFactoryResetIndicator(blinkFactoryResetLed);
  DiagnosticSerial.println(
      "FRONT_PANEL_IO gpi1=15 gpi2=16 gpi3=17 gpi4=18 reset=EN factory_reset=4 ws2812=21 count=1");
}

void performPhysicalFactoryReset() {
  if (factoryResetTriggered) {
    return;
  }
  factoryResetTriggered = true;
  DiagnosticSerial.println("PHYSICAL_FACTORY_RESET hold_ms=5000 action=reset_network_and_gpi");

  const char* gpiReason = 0;
  const bool networkOk = NetworkConfig_resetUserSettingsToFactoryDefaults(gpiReason);
  const char* routeReason = 0;
#if ENABLE_GPI_MODULE
  const bool gpiOk = GpiManager_resetFactorySettings(routeReason);
#else
  const bool gpiOk = true;
#endif
  DiagnosticSerial.print("PHYSICAL_FACTORY_RESET_RESULT network=");
  DiagnosticSerial.print(networkOk ? "ok" : (gpiReason != 0 ? gpiReason : "failed"));
  DiagnosticSerial.print(" gpi=");
  DiagnosticSerial.println(gpiOk ? "ok" : (routeReason != 0 ? routeReason : "failed"));
  blinkFactoryResetLed();
  ESP.restart();
}

void pollFactoryResetButton(uint32_t nowMs) {
  if (factoryResetTriggered) {
    return;
  }
  if (digitalRead(FACTORY_RESET_PIN) != LOW) {
    factoryResetPressedSinceMs = 0;
    return;
  }
  if (factoryResetPressedSinceMs == 0) {
    factoryResetPressedSinceMs = nowMs;
    DiagnosticSerial.println("PHYSICAL_FACTORY_RESET_ARMED release_to_cancel=1 hold_ms=5000");
    return;
  }

  const uint32_t heldMs = nowMs - factoryResetPressedSinceMs;
  if (heldMs >= FACTORY_RESET_HOLD_MS) {
    performPhysicalFactoryReset();
    return;
  }
}

void updateStatusLeds(uint32_t nowMs) {
  if (factoryResetTriggered ||
      static_cast<uint32_t>(nowMs - lastRgbStatusMs) < RGB_STATUS_INTERVAL_MS) {
    return;
  }
  lastRgbStatusMs = nowMs;
  const DebugCounters& counters = gateway.counters();
  const uint32_t canFrames = counters.can_rx_frames + counters.can_tx_frames;
  const bool canActivity = canFrames != lastRgbCanFrames;
  lastRgbCanFrames = canFrames;
  // A 120 ms flash followed by 120 ms of base color stays visible under load.
  if (canFlashActive && static_cast<uint32_t>(nowMs - canFlashStartedMs) >= 240) {
    canFlashActive = false;
  }
  if (canActivity && !canFlashActive) {
    canFlashActive = true;
    canFlashStartedMs = nowMs;
  }
  const bool linkUp = strcmp(NetworkConfig_linkStatusName(), "LinkON") == 0;
  const bool hasIp = NetworkConfig_localIp() != IPAddress(0, 0, 0, 0);
  // Ethernet failure takes priority over CAN activity.
  if (!linkUp || !hasIp) {
    setStatusLed(255, 0, 0);
  } else if (canFlashActive && static_cast<uint32_t>(nowMs - canFlashStartedMs) < 120) {
    setStatusLed(0, 255, 0);
  } else if (NetworkConfig_activeIpSource() == ActiveIpSource::DhcpLease) {
    setStatusLed(0, 255, 255);
  } else {
    setStatusLed(0, 0, 255);  // Static address, including DHCP fallback.
  }
}

}  // namespace

void setup() {
#if ENABLE_SERIAL_DIAGNOSTICS
  Serial.begin(115200);
  delay(500);
#endif
  const esp_reset_reason_t resetReason = esp_reset_reason();
  DiagnosticSerial.print("RESET_REASON=");
  DiagnosticSerial.print(resetReasonName(resetReason));
  DiagnosticSerial.print(" code=");
  DiagnosticSerial.println(static_cast<int>(resetReason));
  DiagnosticSerial.print("R70_BUILD_ID=");
  DiagnosticSerial.println(R70_BUILD_ID);
  beginFrontPanelIo();
  NetworkConfig_begin(DiagnosticSerial);

  GatewayConfig config = {};
  config.profile = GatewayProfile::PROFILE_GATE31_STABLE;
  config.canTxPin = CAN_TX_PIN;
  config.canRxPin = CAN_RX_PIN;
  config.canBitrate = 100000;
  config.canListenOnly = false;
  config.udpPort = 33333;
  config.tcpPort = 30000;
  config.canTxGapMs = 2;
  config.canTxMinSpacingUs = 0;
  config.canTxMinSpacingUsNormal = kDefaultCanTxMinSpacingUsNormal;
  config.canTxMinSpacingUsWrite = kDefaultCanTxMinSpacingUsWrite;
  config.canTxMinSpacingUsScan = kDefaultCanTxMinSpacingUsScan;
  config.identWindowMs = 550;
  config.identQuietHoldMs = 75;
  config.enableWriteRetry = false;
  config.writeAckTimeoutMs = 500;
  config.enableEmergencyTcpBackpressure = true;
  config.maxTcpBackpressureBurstMs = 50;
  config.maxTcpBackpressureTotalPerSecondMs = 150;
  config.enableConfirmedReadCache = R70_CACHE_CONFIRMED_READS_EXPERIMENT != 0;
  // Keep controls write-through, but absorb repeated R1 monitoring reads so
  // they do not build up on the 100 kbit/s CAN bus. Critical watchdog/ISP
  // fields retain their dedicated shorter TTLs below; temperature and stable
  // setup fields retain their longer field-specific TTLs.
  config.cacheTtlDynamicMs = 1000;
  config.enableConfigScalarTtlExperiment = true;
  config.configScalarTtlMs = 1000;
  config.configScalarTtlExperimentMs = 1500;
  config.configScalarTtlPreset = 1000;
  // Channel/device names change only through explicit configuration writes.
  // Error texts use the same complete segmented cache; a changed associated
  // error flag or number invalidates its text immediately.
  // A short common TTL made 2215/4215 expire for every amplifier together,
  // flooding CAN and cascading into R1 device timeouts. Writes still update or
  // invalidate the cached value immediately.
  config.cacheTtlNamesMs = 300000;
  config.nameSegmentTtlMs = 300000;
  config.nameSegmentMaxSegments = 4;
  config.nameSegmentPayloadMaxLen = 8;
  config.enableFirmwareSegmentCache = true;
  config.firmwareSegmentTtlMs = 60000;
  config.firmwareSegmentMaxSegments = 4;
  config.enableUnknown0111ExactCache = true;
  // 0111 is polled very frequently by R1 but changes slowly. Reuse its exact
  // observed reply longer so critical alive/error objects get CAN bandwidth.
  // Startup still requires one real 0111 reply per amplifier.
  config.unknown0111TtlMs = 15000;
  config.enableUnknown0111ExactCacheDebug = false;
  config.watchdogStatusTtlMs = 1500;
  // ISP drives the live channel indication in R1. Keep cached replies fast,
  // and refresh it aggressively without blocking R1 behind the CAN FIFO.
  config.ispTtlMs = 500;
  config.enableCriticalStatusCache = true;
  config.criticalStatusTtlMs = 750;
  config.criticalStatusTtlExperimentMs = 1000;
  config.criticalStatusTtlPreset = 750;
  config.enableCriticalStatusTtlExperiment = true;
  config.enableStableSetupCache = true;
  config.enableSpeakerIdCache = true;
  config.enableInputModeCache = true;
  config.enableDataLinkPresetCache = false;
  // Speaker selection also changes only through tuning writes.  Keep the
  // learned payload and use the amplifier-wide CAN freshness at runtime.
  config.speakerIdTtlMs = 300000;
  config.inputModeTtlMs = 30000;
  config.datalinkPresetTtlMs = 30000;
  config.enableSmallStatusCacheExperiment = false;
  config.enableExpandedStatusCache = true;
  config.enableErrorStatusCache = true;
  config.enableRemHoldCache = true;
  config.enableNameSegmentCacheReplies = true;
  config.enableVerboseFrameLog = false;
  // The capture script sends `mark timeout <nodes>` as soon as R1 reports a
  // timeout. Emit the already available per-node correlation snapshot then;
  // this does not alter request handling or CAN scheduling.
  config.enableTimeoutCorrelation = ENABLE_SERIAL_DIAGNOSTICS != 0;
  config.enableAutoGroupTimeoutSnapshot = false;
  config.enablePerNodeRecentRequestRing = true;
  config.enableLowRateSummary = false;
  config.enableSoftInflightCollapse = ENABLE_FANOUT_ATTACH != 0;
  // Do not let an attached read wait long enough to create an R1 watchdog gap.
  config.softInflightCollapseTimeoutMs = 500;
  config.enableGentleStatusShaping = ENABLE_GENTLE_STATUS_SHAPING != 0;
  config.identity.ip = NetworkConfig_localIp();
  NetworkConfig_getMac(config.identity.mac);
  strncpy(config.identity.product, "R70 ", sizeof(config.identity.product) - 1);
  strncpy(config.identity.hostname,
          NetworkConfig_activeConfig().deviceName,
          sizeof(config.identity.hostname) - 1);
  config.identity.hostname[sizeof(config.identity.hostname) - 1] = '\0';

  printBootStage("before_r70_ready");
  if (!gateway.begin(config, DiagnosticSerial)) {
    DiagnosticSerial.println("R70_EMULATOR_V2_BEGIN_FAILED");
  }
  printBootStage("after_r70_ready");
#if ENABLE_GPI_MODULE
  printBootStage("before_gpi_settings_load");
  gpiManager.begin(gateway, DiagnosticSerial);
  printBootStage("after_gpi_init");
#else
  printBootStage("before_gpi_settings_load");
  DiagnosticSerial.println("GPI_MODULE_DISABLED");
  printBootStage("after_gpi_settings_load");
  printBootStage("before_gpi_init");
  printBootStage("after_gpi_init");
#endif
  WebConfigServer_begin(DiagnosticSerial, &gateway.mutableCounters());
#if ENABLE_SERIAL_DIAGNOSTICS && GPI_DIAG
  lastTcpConnected = gateway.tcpControlConnected();
  LOG_BLOCK(LOG_LEVEL_DEBUG, {
    DiagnosticSerial.print("GPI_DIAG_BOOT uptime_ms=");
    DiagnosticSerial.print(millis());
    DiagnosticSerial.print(" ip=");
    DiagnosticSerial.print(NetworkConfig_localIp());
    DiagnosticSerial.print(" link=");
    DiagnosticSerial.print(NetworkConfig_linkStatusName());
    DiagnosticSerial.print(" tcp_connected=");
    DiagnosticSerial.println(lastTcpConnected ? 1 : 0);
  });
#endif

  printBootStage("before_main_loop");

  // TODO investigate write ACK matching separately from read/cache performance.
}

void loop() {
  if (NetworkConfig_poll()) {
    gateway.restartNetworkServices();
    WebConfigServer_restart();
  }
  gateway.poll();
#if ENABLE_GPI_MODULE
  gpiManager.poll(millis());
#endif
  const uint32_t nowMs = millis();
  pollFactoryResetButton(nowMs);
  updateStatusLeds(nowMs);
  if (static_cast<int32_t>(nowMs - (lastWebPollMs + 25)) >= 0) {
    lastWebPollMs = nowMs;
    WebConfigServer_poll();
  }
#if ENABLE_SERIAL_DIAGNOSTICS
  if (static_cast<int32_t>(nowMs - (lastCountersMs + 10000)) >= 0) {
    lastCountersMs = nowMs;
    gateway.printCounters(DiagnosticSerial);
    if (gateway.counters().bootstrap_nodes_incomplete > 0) {
      gateway.printBootstrapDiagnostics(DiagnosticSerial);
    }
  }
#endif
#if ENABLE_SERIAL_DIAGNOSTICS && GPI_DIAG
  const bool tcpConnected = gateway.tcpControlConnected();
  if (tcpConnected != lastTcpConnected) {
    lastTcpConnected = tcpConnected;
    LOG_BLOCK(LOG_LEVEL_DEBUG, {
      DiagnosticSerial.print("GPI_DIAG_TCP_STATE uptime_ms=");
      DiagnosticSerial.print(nowMs);
      DiagnosticSerial.print(" connected=");
      DiagnosticSerial.print(tcpConnected ? 1 : 0);
      DiagnosticSerial.print(" link=");
      DiagnosticSerial.print(NetworkConfig_linkStatusName());
      DiagnosticSerial.print(" can_tx=");
      DiagnosticSerial.print(gateway.counters().can_tx_frames);
      DiagnosticSerial.print(" can_rx=");
      DiagnosticSerial.print(gateway.counters().can_rx_frames);
      DiagnosticSerial.print(" tcp_clients_connected=");
      DiagnosticSerial.print(gateway.counters().tcp_clients_connected);
      DiagnosticSerial.print(" tcp_frames_parsed=");
      DiagnosticSerial.println(gateway.counters().tcp_frames_parsed);
    });
  }
  if (static_cast<int32_t>(nowMs - (lastGpiDiagHeartbeatMs + 1000)) >= 0) {
    lastGpiDiagHeartbeatMs = nowMs;
    LOG_BLOCK(LOG_LEVEL_TRACE, {
      DiagnosticSerial.print("GPI_DIAG_HEARTBEAT uptime_ms=");
      DiagnosticSerial.print(nowMs);
      DiagnosticSerial.print(" link=");
      DiagnosticSerial.print(NetworkConfig_linkStatusName());
      DiagnosticSerial.print(" ip=");
      DiagnosticSerial.print(NetworkConfig_localIp());
      DiagnosticSerial.print(" tcp_connected=");
      DiagnosticSerial.print(gateway.tcpControlConnected() ? 1 : 0);
      DiagnosticSerial.print(" tcp_clients_connected=");
      DiagnosticSerial.print(gateway.counters().tcp_clients_connected);
      DiagnosticSerial.print(" tcp_frames_parsed=");
      DiagnosticSerial.print(gateway.counters().tcp_frames_parsed);
      DiagnosticSerial.print(" tcp_tx_frames=");
      DiagnosticSerial.print(gateway.counters().tcp_tx_frames);
      DiagnosticSerial.print(" can_tx_frames=");
      DiagnosticSerial.print(gateway.counters().can_tx_frames);
      DiagnosticSerial.print(" can_rx_frames=");
      DiagnosticSerial.print(gateway.counters().can_rx_frames);
      DiagnosticSerial.print(" active_amp_count=");
      DiagnosticSerial.print(gateway.counters().active_amp_count);
      DiagnosticSerial.print(" gpi_pending_repeats=");
      DiagnosticSerial.print(gateway.counters().gpi_pending_repeats);
      DiagnosticSerial.print(" gpi_can_tx_failed=");
      DiagnosticSerial.println(gateway.counters().gpi_can_tx_failed);
    });
  }
  if (static_cast<int32_t>(nowMs - (lastLoopAliveMs + 2000)) >= 0) {
    lastLoopAliveMs = nowMs;
    LOG_BLOCK(LOG_LEVEL_TRACE, {
      printBootStage("main_loop_alive");
      DiagnosticSerial.print("LOOP_ALIVE uptime_ms=");
      DiagnosticSerial.print(nowMs);
      DiagnosticSerial.print(" ip=");
      DiagnosticSerial.print(NetworkConfig_localIp());
      DiagnosticSerial.print(" link=");
      DiagnosticSerial.println(NetworkConfig_linkStatusName());
    });
  }
#endif
}
