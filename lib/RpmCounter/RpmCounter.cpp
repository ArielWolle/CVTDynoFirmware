#include "RpmCounter.h"

namespace {
volatile uint32_t primaryPulses = 0;
volatile uint32_t secondaryPulses = 0;
volatile uint32_t primaryTotalEdges = 0;
volatile uint32_t secondaryTotalEdges = 0;
uint8_t primaryPin = 0;
uint8_t secondaryPin = 0;
uint16_t primarySpokes = 1;
uint16_t secondarySpokes = 1;
uint32_t windowMs = 50;
uint32_t windowStartMs = 0;

void primaryEdge() {
  primaryPulses++;
  primaryTotalEdges++;
}

void secondaryEdge() {
  secondaryPulses++;
  secondaryTotalEdges++;
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
  primaryRpm = (primaryCount * 60000.0f) / (elapsedMs * primarySpokes);
  secondaryRpm = (secondaryCount * 60000.0f) / (elapsedMs * secondarySpokes);
  windowStartMs = nowMs;
  return true;
}

void readDiagnostics(uint32_t& primaryEdges, uint32_t& secondaryEdges) {
  noInterrupts();
  primaryEdges = primaryTotalEdges;
  secondaryEdges = secondaryTotalEdges;
  interrupts();
}
}