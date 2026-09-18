#include "ConfigStore.h"
#include <EEPROM.h>

namespace {
constexpr uint32_t CONFIG_MAGIC = 0x43565444; // "CVTD" -- distinguishes real saved data from erased/blank flash
constexpr uint16_t CONFIG_VERSION = 1;        // bump whenever PersistedConfig's layout changes
constexpr int EEPROM_RESERVED_SIZE = 256;     // minimum allowed by the RP2040 emulated-EEPROM library; plenty here
constexpr unsigned long SAVE_DEBOUNCE_MS = 2000;

struct __attribute__((packed)) PersistedConfig {
  uint32_t magic = 0;
  uint16_t version = 0;
  uint8_t  write_en[5] = {0}; // stored as bytes rather than bool for well-defined packed layout
  uint16_t freq[5] = {0};
  uint16_t rpm1_spokes = 0;
  uint16_t rpm2_spokes = 0;
  uint8_t  checksum = 0; // crc8 over every byte above, computed last
};

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

uint8_t computeChecksum(const PersistedConfig& cfg) {
  return crc8(reinterpret_cast<const uint8_t*>(&cfg), sizeof(PersistedConfig) - sizeof(cfg.checksum));
}

void toPersisted(const ConfigStore::RuntimeConfig& in, PersistedConfig& out) {
  out.magic = CONFIG_MAGIC;
  out.version = CONFIG_VERSION;
  for (int i = 0; i < 5; i++) {
    out.write_en[i] = in.write_en[i] ? 1 : 0;
    out.freq[i] = in.freq[i];
  }
  out.rpm1_spokes = in.rpm1_spokes;
  out.rpm2_spokes = in.rpm2_spokes;
  out.checksum = 0; // filled in by the caller once the rest of the struct is final
}

bool persistedMatchesRuntime(const PersistedConfig& persisted, const ConfigStore::RuntimeConfig& runtime) {
  for (int i = 0; i < 5; i++) {
    if (persisted.write_en[i] != (runtime.write_en[i] ? 1 : 0)) return false;
    if (persisted.freq[i] != runtime.freq[i]) return false;
  }
  return persisted.rpm1_spokes == runtime.rpm1_spokes && persisted.rpm2_spokes == runtime.rpm2_spokes;
}

bool runtimeChanged(const ConfigStore::RuntimeConfig& a, const ConfigStore::RuntimeConfig& b) {
  for (int i = 0; i < 5; i++) {
    if (a.write_en[i] != b.write_en[i]) return true;
    if (a.freq[i] != b.freq[i]) return true;
  }
  return a.rpm1_spokes != b.rpm1_spokes || a.rpm2_spokes != b.rpm2_spokes;
}

// --- poll()/load() bookkeeping, all core0-only (both are only ever called from loop()/setup()) ---
ConfigStore::RuntimeConfig lastSeen;
bool haveLastSeen = false;
bool dirty = false;
unsigned long lastChangeMs = 0;
// Tracks what's actually on flash right now, so poll() never re-writes a value that already
// matches what's saved (e.g. right after load(), or if a change is reverted before the debounce
// window elapses).
PersistedConfig lastSavedPersisted;
bool haveLastSavedPersisted = false;

void writeToFlash(const ConfigStore::RuntimeConfig& current) {
  PersistedConfig cfg;
  toPersisted(current, cfg);
  cfg.checksum = computeChecksum(cfg);
  EEPROM.put(0, cfg);
  EEPROM.commit();
  lastSavedPersisted = cfg;
  haveLastSavedPersisted = true;
}
}

namespace ConfigStore {
void begin() {
  EEPROM.begin(EEPROM_RESERVED_SIZE);
}

bool load(RuntimeConfig& out) {
  PersistedConfig cfg;
  EEPROM.get(0, cfg);
  if (cfg.magic != CONFIG_MAGIC || cfg.version != CONFIG_VERSION) return false;
  if (computeChecksum(cfg) != cfg.checksum) return false;

  for (int i = 0; i < 5; i++) {
    out.write_en[i] = cfg.write_en[i] != 0;
    out.freq[i] = cfg.freq[i];
  }
  out.rpm1_spokes = cfg.rpm1_spokes > 0 ? cfg.rpm1_spokes : 1;
  out.rpm2_spokes = cfg.rpm2_spokes > 0 ? cfg.rpm2_spokes : 1;

  lastSeen = out;
  haveLastSeen = true;
  lastSavedPersisted = cfg;
  haveLastSavedPersisted = true;
  dirty = false;
  return true;
}

void poll(const RuntimeConfig& current) {
  if (!haveLastSeen || runtimeChanged(current, lastSeen)) {
    lastSeen = current;
    haveLastSeen = true;
    lastChangeMs = millis();
    dirty = true;
  }

  if (dirty && (millis() - lastChangeMs >= SAVE_DEBOUNCE_MS)) {
    if (!haveLastSavedPersisted || !persistedMatchesRuntime(lastSavedPersisted, current)) {
      writeToFlash(current);
    }
    dirty = false;
  }
}
}
