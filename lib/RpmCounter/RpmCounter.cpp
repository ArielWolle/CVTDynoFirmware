#include "RpmCounter.h"

namespace {
volatile uint32_t primaryPulses = 0;
volatile uint32_t secondaryPulses = 0;
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
uint32_t lastPrimaryEdgesForRpm = 0;
uint32_t lastSecondaryEdgesForRpm = 0;
bool interruptTestMode = false;

void primaryEdge() {
  primaryPulses++;
  primaryTotalEdges++;
  if (interruptTestMode) primaryInterruptEvents++;
}

void secondaryEdge() {
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
  lastPrimaryEdgesForRpm = 0;
  lastSecondaryEdgesForRpm = 0;

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

  noInterrupts();
  const uint32_t primaryCount = primaryPulses;
  const uint32_t secondaryCount = secondaryPulses;
  const uint32_t primaryEdges = primaryTotalEdges;
  const uint32_t secondaryEdges = secondaryTotalEdges;
  primaryPulses = 0;
  secondaryPulses = 0;
  interrupts();

  // Use total-edge deltas for RPM so reporting remains stable even when
  // test modes inspect/reset per-window pulse counters.
  const uint32_t primaryDelta = primaryEdges - lastPrimaryEdgesForRpm;
  const uint32_t secondaryDelta = secondaryEdges - lastSecondaryEdgesForRpm;
  lastPrimaryEdgesForRpm = primaryEdges;
  lastSecondaryEdgesForRpm = secondaryEdges;

  // Keep a tiny floor path using per-window counts to avoid reporting
  // persistent zero if edge counters are delayed but pulse counters moved.
  const uint32_t primaryForCalc = primaryDelta > 0 ? primaryDelta : primaryCount;
  const uint32_t secondaryForCalc = secondaryDelta > 0 ? secondaryDelta : secondaryCount;

  // RPM = (pulses * 60000) / (elapsed_ms * spokes)
  primaryRpm = (uint32_t)(((double)primaryForCalc * 60000.0) / ((double)elapsedMs * (double)primarySpokes));
  secondaryRpm = (uint32_t)(((double)secondaryForCalc * 60000.0) / ((double)elapsedMs * (double)secondarySpokes));
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