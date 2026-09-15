#pragma once

#include <Arduino.h>

namespace RpmCounter {
void begin(uint8_t primaryPin, uint8_t secondaryPin, uint16_t primarySpokes, uint16_t secondarySpokes, uint32_t windowMs);
bool update(uint32_t& primaryRpm, uint32_t& secondaryRpm);
void readDiagnostics(uint32_t& primaryEdges, uint32_t& secondaryEdges);
void setSpokes(uint8_t channel, uint16_t spokes);
void getSpokes(uint16_t& primarySpokes, uint16_t& secondarySpokes);
void setInterruptTestMode(bool enabled);
}