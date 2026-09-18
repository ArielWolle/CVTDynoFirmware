#pragma once

#include <Arduino.h>
#include <pico/time.h> // time_us_64()

// Edge-triggered RPM tooth streaming: every pin edge (tooth) is captured with its exact
// microsecond timestamp and the raw period since the previous edge on that channel, then queued
// for the caller to drain via popEdge() -- one event per physical tooth, not a polled/windowed
// average. Spoke count (teeth-per-revolution) and any RPM smoothing are NOT computed here; that's
// left entirely to the consumer (see popEdge()) so a spoke-count change never needs a firmware
// round-trip and the consumer can choose its own smoothing without the firmware pre-processing
// the signal away.
namespace RpmCounter {
// Sets up the two RPM pins as edge-triggered (RISING) inputs.
void begin(uint8_t primaryPin, uint8_t secondaryPin);

// Pops the oldest pending edge event for the given channel (0=primary, 1=secondary), if any.
//   edgeUs   -- time_us_64() timestamp the edge was captured.
//   periodUs -- elapsed time since the previous edge on this channel. 0 means "no previous edge to
//               compare against" (the very first edge since begin(), or the first edge after a
//               gap -- see NOTE below) -- callers should treat periodUs==0 as a reset/discontinuity
//               marker, not as an actual zero-length period.
// Returns false when there are no more pending events for this channel right now. Intended to be
// called in a tight "while (popEdge(...))" loop so a channel is always fully drained -- the ring
// this reads from is sized to absorb realistic bursts (see EVENT_RING_SIZE in the .cpp), but a
// consumer that doesn't drain promptly will eventually see dropped events (readAndClearDropped()).
//
// NOTE: unlike the previous polled implementation, there is no on-device staleness/idle timeout --
// if the wheel stops turning, no more events arrive, period. Detecting "engine stopped" from a
// gap in incoming events is left to the consumer, which is better positioned to pick a timeout
// appropriate for its own display/logging cadence than the firmware is.
bool popEdge(uint8_t channel, uint64_t& edgeUs, uint32_t& periodUs);

// Bench/demo-mode support: inject a synthetic edge on the given channel with the given period, as
// if it had physically arrived just now. Goes through the exact same ring buffer as real edges, so
// demo mode exercises the identical streaming path a real sensor would.
void injectSyntheticEdge(uint8_t channel, uint32_t periodUs);

// Read-and-clear count of edges that were dropped because a channel's ring buffer was full (the
// consumer wasn't draining fast enough for that channel's edge rate). Surfaced as a diagnostic so
// a real backlog is visible instead of silently invisible.
uint32_t readAndClearDropped(uint8_t channel);

void readDiagnostics(uint32_t& primaryEdges, uint32_t& secondaryEdges);
// Read-and-clear edge counters intended for periodic (e.g. 100ms) diagnostic polling -- reports
// edges seen since the last call, not a fixed measurement window.
void readWindowCounts(uint32_t& primaryCount, uint32_t& secondaryCount);
void readAndClearInterruptEvents(uint32_t& primaryEvents, uint32_t& secondaryEvents);
void setInterruptTestMode(bool enabled);
}
