#include "RpmCounter.h"

namespace {
// --- Per-window edge accounting (reset every `update()` call) ---
volatile uint32_t primaryPulses = 0;
volatile uint32_t secondaryPulses = 0;
volatile uint64_t primaryWindowFirstEdgeUs = 0;   // 0 == "no edge yet this window"
volatile uint64_t secondaryWindowFirstEdgeUs = 0;
volatile uint64_t primaryWindowLastEdgeUs = 0;
volatile uint64_t secondaryWindowLastEdgeUs = 0;

// --- Cross-window state (persists) ---
volatile uint64_t primaryLastEdgeUs = 0;          // most recent edge ever, for staleness checks
volatile uint64_t secondaryLastEdgeUs = 0;
volatile uint32_t primaryRawPeriodUs = 0;         // most recent single inter-edge interval, unsmoothed
volatile uint32_t secondaryRawPeriodUs = 0;
volatile uint32_t primaryTotalEdges = 0;
volatile uint32_t secondaryTotalEdges = 0;
volatile uint32_t primaryInterruptEvents = 0;
volatile uint32_t secondaryInterruptEvents = 0;

uint8_t primaryPin = 0;
uint8_t secondaryPin = 0;
uint16_t primarySpokes = 1;
uint16_t secondarySpokes = 1;
uint32_t windowMs = 50;
uint32_t windowStartMs = 0;
bool interruptTestMode = false;

// A stale reading (no edges for this long) reports 0 RPM instead of holding a frozen value.
constexpr uint64_t RPM_STALE_TIMEOUT_US = 500000;

void primaryEdge() {
  const uint64_t nowUs = time_us_64();
  if (primaryLastEdgeUs != 0) {
    const uint64_t dt = nowUs - primaryLastEdgeUs;
    primaryRawPeriodUs = (uint32_t)dt;
  }
  primaryLastEdgeUs = nowUs;
  if (primaryWindowFirstEdgeUs == 0) primaryWindowFirstEdgeUs = nowUs;
  primaryWindowLastEdgeUs = nowUs;
  primaryPulses++;
  primaryTotalEdges++;
  if (interruptTestMode) primaryInterruptEvents++;
}

void secondaryEdge() {
  const uint64_t nowUs = time_us_64();
  if (secondaryLastEdgeUs != 0) {
    const uint64_t dt = nowUs - secondaryLastEdgeUs;
    secondaryRawPeriodUs = (uint32_t)dt;
  }
  secondaryLastEdgeUs = nowUs;
  if (secondaryWindowFirstEdgeUs == 0) secondaryWindowFirstEdgeUs = nowUs;
  secondaryWindowLastEdgeUs = nowUs;
  secondaryPulses++;
  secondaryTotalEdges++;
  if (interruptTestMode) secondaryInterruptEvents++;
}

// Reciprocal-counting frequency measurement: given N edges spanning a known elapsed time, the
// average period is elapsed / (N-1) -- exact and unbiased regardless of RPM, with no smoothing lag.
// This replaces an older exponential moving average of single-edge periods, which systematically
// lagged behind RPM during acceleration/deceleration (exactly the part of a dyno pull that
// matters most). When too few edges land in a window to do reciprocal counting (very low RPM
// relative to the window length), we fall back to the single most recent raw inter-edge period
// with no smoothing applied, so the reported value is always the least-processed accurate number
// available rather than a filtered/lagged one -- any desired smoothing is left to the app layer,
// which can see the full raw stream and choose a window.
uint32_t computeRpm(uint32_t edgeCount, uint64_t windowFirstUs, uint64_t windowLastUs, uint32_t rawPeriodUs, uint16_t spokes) {
  if (edgeCount >= 2 && windowLastUs > windowFirstUs) {
    const double elapsedUs = (double)(windowLastUs - windowFirstUs);
    const double periodUs = elapsedUs / (double)(edgeCount - 1);
    return (uint32_t)(60000000.0 / (periodUs * (double)spokes));
  }
  if (rawPeriodUs > 0) {
    return (uint32_t)(60000000.0 / ((double)rawPeriodUs * (double)spokes));
  }
  return 0;
}
}

namespace RpmCounter {
void begin(uint8_t newPrimaryPin, uint8_t newSecondaryPin, uint16_t newPrimarySpokes, uint16_t newSecondarySpokes, uint32_t newWindowMs) {
  primaryPin = newPrimaryPin;
  secondaryPin = newSecondaryPin;
  primarySpokes = newPrimarySpokes > 0 ? newPrimarySpokes : 1;
  secondarySpokes = newSecondarySpokes > 0 ? newSecondarySpokes : 1;
  windowMs = newWindowMs;
  windowStartMs = millis();
  primaryPulses = 0;
  secondaryPulses = 0;
  primaryWindowFirstEdgeUs = 0;
  secondaryWindowFirstEdgeUs = 0;
  primaryWindowLastEdgeUs = 0;
  secondaryWindowLastEdgeUs = 0;
  primaryTotalEdges = 0;
  secondaryTotalEdges = 0;
  primaryInterruptEvents = 0;
  secondaryInterruptEvents = 0;
  primaryLastEdgeUs = 0;
  secondaryLastEdgeUs = 0;
  primaryRawPeriodUs = 0;
  secondaryRawPeriodUs = 0;

  // Optoisolator outputs are normally open-collector/open-drain.
  pinMode(primaryPin, INPUT_PULLUP);
  pinMode(secondaryPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(primaryPin), primaryEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(secondaryPin), secondaryEdge, RISING);
}

bool update(uint32_t& primaryRpm, uint32_t& secondaryRpm, uint64_t& primaryCaptureUs, uint64_t& secondaryCaptureUs) {
  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - windowStartMs;
  if (elapsedMs < windowMs) return false;
  const uint64_t nowUs = time_us_64();

  noInterrupts();
  const uint32_t primaryCount = primaryPulses;
  const uint32_t secondaryCount = secondaryPulses;
  const uint64_t primaryFirstUs = primaryWindowFirstEdgeUs;
  const uint64_t secondaryFirstUs = secondaryWindowFirstEdgeUs;
  const uint64_t primaryLastUs = primaryWindowLastEdgeUs;
  const uint64_t secondaryLastUs = secondaryWindowLastEdgeUs;
  const uint64_t primaryLastEverUs = primaryLastEdgeUs;
  const uint64_t secondaryLastEverUs = secondaryLastEdgeUs;
  const uint32_t primaryRawUs = primaryRawPeriodUs;
  const uint32_t secondaryRawUs = secondaryRawPeriodUs;
  primaryPulses = 0;
  secondaryPulses = 0;
  primaryWindowFirstEdgeUs = 0;
  secondaryWindowFirstEdgeUs = 0;
  interrupts();

  const bool primaryRecent = primaryLastEverUs != 0 && (nowUs - primaryLastEverUs) <= RPM_STALE_TIMEOUT_US;
  const bool secondaryRecent = secondaryLastEverUs != 0 && (nowUs - secondaryLastEverUs) <= RPM_STALE_TIMEOUT_US;

  primaryRpm = primaryRecent ? computeRpm(primaryCount, primaryFirstUs, primaryLastUs, primaryRawUs, primarySpokes) : 0;
  secondaryRpm = secondaryRecent ? computeRpm(secondaryCount, secondaryFirstUs, secondaryLastUs, secondaryRawUs, secondarySpokes) : 0;

  // Stamp the capture time as the last edge actually used in the measurement (or "now" for a
  // stale/zero reading) so the value is timestamped when it was physically true, not when this
  // polling function happened to run.
  primaryCaptureUs = primaryRecent ? (primaryCount >= 1 ? primaryLastUs : primaryLastEverUs) : nowUs;
  secondaryCaptureUs = secondaryRecent ? (secondaryCount >= 1 ? secondaryLastUs : secondaryLastEverUs) : nowUs;

  windowStartMs = nowMs;
  return true;
}

void readDiagnostics(uint32_t& primaryEdges, uint32_t& secondaryEdges) {
  noInterrupts();
  primaryEdges = primaryTotalEdges;
  secondaryEdges = secondaryTotalEdges;
  interrupts();
}

void readWindowCounts(uint32_t& primaryCount, uint32_t& secondaryCount) {
  noInterrupts();
  primaryCount = primaryPulses;
  secondaryCount = secondaryPulses;
  interrupts();
}

void readAndClearInterruptEvents(uint32_t& primaryEvents, uint32_t& secondaryEvents) {
  noInterrupts();
  primaryEvents = primaryInterruptEvents;
  secondaryEvents = secondaryInterruptEvents;
  primaryInterruptEvents = 0;
  secondaryInterruptEvents = 0;
  interrupts();
}

void setSpokes(uint8_t channel, uint16_t spokes) {
  noInterrupts();
  if (channel == 0) {
    primarySpokes = spokes > 0 ? spokes : 1;
  } else if (channel == 1) {
    secondarySpokes = spokes > 0 ? spokes : 1;
  }
  interrupts();
}

void getSpokes(uint16_t& primaryOut, uint16_t& secondaryOut) {
  noInterrupts();
  primaryOut = primarySpokes;
  secondaryOut = secondarySpokes;
  interrupts();
}

void setInterruptTestMode(bool enabled) {
  interruptTestMode = enabled;
}
}
