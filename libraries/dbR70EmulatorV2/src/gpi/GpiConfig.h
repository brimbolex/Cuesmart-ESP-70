#ifndef DB_R70_V2_GPI_CONFIG_H
#define DB_R70_V2_GPI_CONFIG_H

#include <stdint.h>

namespace dbcan {
namespace r70v2 {

enum class GpiActionMode : uint8_t {
  None = 0,
  BackupLoadRestore = 1,
};

static const uint8_t kGpiPhysicalCount = 4;
static const uint8_t kGpiRouteCount = 8;

struct GpiRouteRow {
  uint8_t enabled;
  uint8_t physicalGpi;
  uint8_t targetSubnet;
  uint8_t targetDevice;
  uint8_t loadPreset;
  uint8_t backupSlot;
  GpiActionMode actionMode;
};

using GpiPresetRouteConfig = GpiRouteRow;
using GpiInputConfig = GpiRouteRow;

struct GpiSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc32;
  GpiRouteRow route[kGpiRouteCount];
};

bool isGpiPinAllowed(uint8_t pin);
const char* gpiActionModeName(GpiActionMode mode);

class GpiSettingsStore {
 public:
  GpiSettingsStore();

  bool begin();
  bool loadOrFactory(GpiSettings& out, bool& usedFactory);
  bool saveUserSettings(const GpiSettings& in);
  bool resetUserSettingsToFactoryDefaults();
  void makeFactoryDefaults(GpiSettings& out);
 bool isValid(const GpiSettings& settings);
  const char* lastError() const;

 private:
  uint32_t computeCrc(const GpiSettings& settings) const;
  void copyWithCrc(GpiSettings& dest, const GpiSettings& src) const;

  const char* lastError_;
  bool started_;
};

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_GPI_CONFIG_H
