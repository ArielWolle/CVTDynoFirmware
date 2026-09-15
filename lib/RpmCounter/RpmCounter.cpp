#include "RpmCounter.h"

namespace {
volatile uint32_t primaryPulses = 0;
volatile uint32_t secondaryPulses = 0;
volatile uint32_t primaryTotalEdges = 0;
volatile uint32_t secondaryTotalEdges = 0;
volatile uint32_t primaryInterruptEvents = 0;
volatile uint32_t secondaryInterruptEvents = 0;
volatile uint32_t primaryLastEdgeUs = 0;
volatile uint32_t secondaryLastEdgeUs = 0;
volatile uint32_t primaryPeriodUs = 0;
volatile uint32_t secondaryPeriodUs = 0;
uint8_t primaryPin = 0;
uint8_t secondaryPin = 0;
uint16_t primarySpokes = 1;
uint16_t secondarySpokes = 1;
uint32_t windowMs = 50;
uint32_t windowStartMs = 0;
bool interruptTestMode = false;

constexpr uint32_t RPM_STALE_TIMEOUT_US = 500000;

void primaryEdge() {
  const uint32_t nowUs = micros();
  if (primaryLastEdgeUs != 0) {
    const uint32_t dt = nowUs - primaryLastEdgeUs;
    primaryPeriodUs = primaryPeriodUs == 0 ? dt : (primaryPeriodUs * 3 + dt) / 4;
  }
  primaryLastEdgeUs = nowUs;
  primaryPulses++;
  primaryTotalEdges++;
  if (interruptTestMode) primaryInterruptEvents++;
}

void secondaryEdge() {
  const uint32_t nowUs = micros();
  if (secondaryLastEdgeUs != 0) {
    const uint32_t dt = nowUs - secondaryLastEdgeUs;
    secondaryPeriodUs = secondaryPeriodUs == 0 ? dt : (secondaryPeriodUs * 3 + dt) / 4;
  }
  secondaryLastEdgeUs = nowUs;
  secondaryPulses++;
  secondaryTotalEdges++;
  if (interruptTestMode) secondaryInterruptEvents++;
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
  primaryTotalEdges = 0;
  secondaryTotalEdges = 0;
  primaryInterruptEvents = 0;
  secondaryInterruptEvents = 0;
  primaryLastEdgeUs = 0;
  secondaryLastEdgeUs = 0;
  primaryPeriodUs = 0;
  secondaryPeriodUs = 0;

  // Optoisolator outputs are normally open-collector/open-drain.
  pinMode(primaryPin, INPUT_PULLUP);
  pinMode(secondaryPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(primaryPin), primaryEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(secondaryPin), secondaryEdge, RISING);
}

bool update(uint32_t& primaryRpm, uint32_t& secondaryRpm) {
  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - windowStartMs;
  if (elapsedMs < windowMs) return false;
  const uint32_t nowUs = micros();

  noInterrupts();
  const uint32_t primaryCount = primaryPulses;
  const uint32_t secondaryCount = secondaryPulses;
  const uint32_t primaryLastUs = primaryLastEdgeUs;
  const uint32_t secondaryLastUs = secondaryLastEdgeUs;
  const uint32_t primaryDtUs = primaryPeriodUs;
  const uint32_t secondaryDtUs = secondaryPeriodUs;
  primaryPulses = 0;
  secondaryPulses = 0;
  interrupts();

  const bool primaryRecent = primaryLastUs != 0 && (nowUs - primaryLastUs) <= RPM_STALE_TIMEOUT_US;
  const bool secondaryRecent = secondaryLastUs != 0 && (nowUs - secondaryLastUs) <= RPM_STALE_TIMEOUT_US;

  if (primaryRecent && primaryDtUs > 0) {
    primaryRpm = (uint32_t)(60000000.0 / ((double)primaryDtUs * (double)primarySpokes));
  } else {
    primaryRpm = primaryCount > 0
      ? (uint32_t)(((double)primaryCount * 60000.0) / ((double)elapsedMs * (double)primarySpokes))
      : 0;
  }

  if (secondaryRecent && secondaryDtUs > 0) {
    secondaryRpm = (uint32_t)(60000000.0 / ((double)secondaryDtUs * (double)secondarySpokes));
  } else {
    secondaryRpm = secondaryCount > 0
      ? (uint32_t)(((double)secondaryCount * 60000.0) / ((double)elapsedMs * (double)secondarySpokes))
      : 0;
  }

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