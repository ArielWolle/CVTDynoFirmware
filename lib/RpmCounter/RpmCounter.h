#pragma once

#include <Arduino.h>
#include <pico/time.h> // time_us_64()

namespace RpmCounter {
void begin(uint8_t primaryPin, uint8_t secondaryPin, uint16_t primarySpokes, uint16_t secondarySpokes, uint32_t windowMs);
// primaryCaptureUs/secondaryCaptureUs report the firmware time_us_64() timestamp the returned RPM
// value is actually valid as-of (the last edge used in the measurement, or "now" for a stale/zero
// reading) -- callers should stamp their outgoing packet with this, not the time update() happened
// to be called, so multi-rate consumers get an accurate per-sample timeline.
bool update(uint32_t& primaryRpm, uint32_t& secondaryRpm, uint64_t& primaryCaptureUs, uint64_t& secondaryCaptureUs);
void readDiagnostics(uint32_t& primaryEdges, uint32_t& secondaryEdges);
void readWindowCounts(uint32_t& primaryCount, uint32_t& secondaryCount);
void readAndClearInterruptEvents(uint32_t& primaryEvents, uint32_t& secondaryEvents);
void setSpokes(uint8_t channel, uint16_t spokes);
void getSpokes(uint16_t& primarySpokes, uint16_t& secondarySpokes);
void setInterruptTestMode(bool enabled);
}
