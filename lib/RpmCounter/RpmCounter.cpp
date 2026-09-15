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
  primaryPulses = 0;
  secondaryPulses = 0;
  interrupts();

  // pulses / elapsed-ms * 60,000 converts pulse frequency to RPM.
  // Formula: RPM = (pulses / spokes) / (elapsed_ms / 60000) = (pulses * 60000) / (elapsed_ms * spokes)
  primaryRpm = (uint32_t)(((float)primaryCount * 60000.0f) / ((float)elapsedMs * (float)primarySpokes));
  secondaryRpm = (uint32_t)(((float)secondaryCount * 60000.0f) / ((float)elapsedMs * (float)secondarySpokes));
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