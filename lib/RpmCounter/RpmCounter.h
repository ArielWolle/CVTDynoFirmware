#pragma once

#include <Arduino.h>

namespace RpmCounter {
void begin(uint8_t primaryPin, uint8_t secondaryPin, uint16_t primarySpokes, uint16_t secondarySpokes, uint32_t windowMs);
bool update(uint32_t& primaryRpm, uint32_t& secondaryRpm);
}