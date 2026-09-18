#pragma once

#include <Arduino.h>
#include <pico/time.h> // time_us_64()

namespace RpmCounter {
// edgesPerUpdate controls how many consecutive edges each reciprocal-counting computation spans:
// 1 (default) recomputes RPM on every single edge for the lowest possible latency (edge period ~1ms
// at typical dyno RPM/tooth counts, i.e. up to ~1kHz updates); a higher value trades latency for
// noise immunity against tooth-spacing manufacturing tolerance by averaging over more edges.
void begin(uint8_t primaryPin, uint8_t secondaryPin, uint16_t primarySpokes, uint16_t secondarySpokes,
           uint16_t primaryEdgesPerUpdate = 1, uint16_t secondaryEdgesPerUpdate = 1);
// primaryCaptureUs/secondaryCaptureUs report the firmware time_us_64() timestamp the returned RPM
// value is actually valid as-of (the last edge used in the measurement, or "now" for a stale/zero
// reading) -- callers should stamp their outgoing packet with this, not the time update() happened
// to be called, so multi-rate consumers get an accurate per-sample timeline.
// primaryReady/secondaryReady are set independently -- each channel produces a fresh edge-triggered
// value on its own schedule, so a burst of edges on one channel never gates the other.
void update(uint32_t& primaryRpm, uint32_t& secondaryRpm, uint64_t& primaryCaptureUs, uint64_t& secondaryCaptureUs,
            bool& primaryReady, bool& secondaryReady);
void readDiagnostics(uint32_t& primaryEdges, uint32_t& secondaryEdges);
// Read-and-clear edge counters intended for periodic (e.g. 100ms) diagnostic polling -- reports
// edges seen since the last call, not a fixed measurement window.
void readWindowCounts(uint32_t& primaryCount, uint32_t& secondaryCount);
void readAndClearInterruptEvents(uint32_t& primaryEvents, uint32_t& secondaryEvents);
void setSpokes(uint8_t channel, uint16_t spokes);
void getSpokes(uint16_t& primarySpokes, uint16_t& secondarySpokes);
void setEdgesPerUpdate(uint8_t channel, uint16_t edgesPerUpdate);
void getEdgesPerUpdate(uint16_t& primaryOut, uint16_t& secondaryOut);
void setInterruptTestMode(bool enabled);
}
