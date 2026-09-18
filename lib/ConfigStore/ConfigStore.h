#pragma once

#include <Arduino.h>

// Persists the operational (non-diagnostic) live configuration -- per-channel enable/Hz and RPM
// spoke counts -- to the RP2040's emulated flash EEPROM, so settings survive a power cycle instead
// of resetting to the compiled-in defaults every boot. Diagnostic/bench toggles (demo mode,
// pin/interrupt/count test modes) are intentionally NOT persisted here -- those are transient
// debugging aids, not something you'd want silently surviving a power cycle.
namespace ConfigStore {
struct RuntimeConfig {
  bool write_en[5] = {true, true, true, true, true};
  uint16_t freq[5] = {0, 0, 0, 0, 0};
  uint16_t rpm1_spokes = 1;
  uint16_t rpm2_spokes = 1;
};

// Must be called once at startup (before load()/poll()) -- reserves and loads the emulated EEPROM
// flash sector into RAM.
void begin();

// Fills `out` and returns true only if a valid previously-saved configuration was found in flash
// (magic number, layout version, and checksum all match). Returns false and leaves `out` untouched
// on first boot ever, or after a firmware update that changed the saved layout -- the caller should
// keep using its compiled-in defaults in that case.
bool load(RuntimeConfig& out);

// Call once per loop() iteration with the current live configuration. Internally detects changes
// against what was last seen, debounces them, and only actually writes to flash once settings have
// been stable for that debounce period -- coalescing a burst of rapid changes (e.g. an app
// configuring every channel right after connecting) into a single flash write, since flash has a
// limited erase/write cycle lifetime and a hardware program/erase op briefly pauses both cores.
void poll(const RuntimeConfig& current);
}
