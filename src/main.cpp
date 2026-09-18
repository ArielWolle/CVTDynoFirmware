#include <SPI.h>
#include <string.h>
#include <pico/time.h> // time_us_64()
#include <RpmCounter.h>

// --- PIN DEFINITIONS ---
const int PIN_RPM1   = 3;  // Must be ODD
const int PIN_RPM2   = 1;  // Must be ODD
const int PIN_SHIFT  = 26; // Analog A0
const int PIN_ADS_CS   = 17; 
const int PIN_ADS_DRDY = 20; 
const int PIN_ADS_RST  = 21; 
const int PIN_FULL_THROTTLE = 0; // Binary input, pulled up; grounded (LOW) = full throttle

// Number of optoisolator pulses generated per wheel revolution.
// Set these to the physical spoke count for each RPM wheel.
const uint16_t RPM1_SPOKES = 16;
const uint16_t RPM2_SPOKES = 12;

#define ADS_CMD_RDATA  0x01
#define ADS_REG_MUX    0x01

// =========================================================================
// SERIAL PROTOCOL v2
// =========================================================================
// Telemetry packet (17 bytes), sent independently per channel so each channel keeps its own
// configurable rate (see cfg_freq[] below) -- e.g. torque can run much faster than RPM without
// affecting RPM's cadence, and vice versa.
//   [0]    0xAA  sync byte 0
//   [1]    0x55  sync byte 1
//   [2]    channel id (0..5)
//   [3]    per-channel rolling sequence number (drop detection on the receiving end)
//   [4..7] int32 value, little-endian
//   [8..15] uint64 firmware capture timestamp (time_us_64()), little-endian --
//           stamped at the moment the value was physically true (e.g. the last RPM edge used in
//           its reciprocal-counting span), not when this packet happened to be sent, so a receiver
//           can rebuild an accurate per-channel timeline even though channels arrive at different
//           rates.
//   [16]   CRC-8 (poly 0x07, init 0x00) over bytes [2..15]
//
// Channel 5 (full throttle) is a binary input reported purely on change, not on a schedule --
// it has no entry in cfg_write_en[]/cfg_freq[] (there is no "rate" to configure) and is instead
// pushed immediately whenever the pin transitions, so the receiver sees the exact moment it
// happened rather than waiting for the next poll.
#define TELEMETRY_SYNC0 0xAA
#define TELEMETRY_SYNC1 0x55
#define TELEMETRY_PACKET_LEN 17
#define TELEMETRY_CRC_SPAN 14 // bytes [2..15]
#define CHANNEL_FULL_THROTTLE 5

// Command packet (5 bytes) -- a leading sync byte lets the parser resync after any dropped or
// corrupted byte instead of permanently misaligning every subsequent command.
//   [0] 0xC0 sync
//   [1] command id
//   [2] channel
//   [3] value high byte
//   [4] value low byte
#define COMMAND_SYNC 0xC0
#define COMMAND_PACKET_LEN 5

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// =========================================================================
// RUNTIME LIVE CONFIGURATION BUFFER (Shared dynamically between cores)
// =========================================================================
volatile bool cfg_write_en[5] = {true, true, true, true, true}; // RPM1, RPM2, SHIFT, T1, T2
volatile uint16_t cfg_freq[5] = {20, 20, 10, 50, 50};           // Frequencies in Hz, independent per channel
volatile bool demo_mode = false;                                // Command 0x04: synthetic bench data
volatile bool rpm_pin_test = false;                             // Command 0x05: diagnostic pin reports
volatile bool rpm_interrupt_test = false;                       // Command 0x07: real-time interrupt logging
volatile bool rpm_count_test = false;                           // Command 0x08: pulse count diagnostics

// Microsecond tracking variables for independent scheduling on Core 0
unsigned long last_tx_us[5]  = {0, 0, 0, 0, 0};
volatile unsigned long intervals_us[5]; 
uint8_t tx_seq[6] = {0, 0, 0, 0, 0, 0}; // channels 0-4 (scheduled) + 5 (full throttle, on-change)
unsigned long last_rpm_pin_test_ms = 0;
unsigned long last_rpm_count_test_ms = 0;
unsigned long last_rpm_interrupt_test_ms = 0;

// --- Full throttle input (channel 5) -------------------------------------------------------
// Reported purely on change via interrupt, with no polling rate to configure. The ISR only
// captures the new state and timestamp and sets a pending flag -- it deliberately does not call
// Serial.write() itself, since that could interrupt an in-progress write from the main scheduler
// loop below and corrupt both packets. loop() checks the flag every iteration and sends
// immediately, so the added latency versus writing directly from the ISR is negligible (at most
// one loop() iteration, typically well under a millisecond) while staying safe.
volatile bool full_throttle_pending = false;
volatile uint8_t full_throttle_value = 0;
volatile uint64_t full_throttle_capture_us = 0;
volatile uint64_t full_throttle_last_edge_us = 0;
constexpr uint64_t FULL_THROTTLE_DEBOUNCE_US = 5000; // ignore bounces within 5ms of the last edge

void fullThrottleChange() {
  const uint64_t nowUs = time_us_64();
  if (nowUs - full_throttle_last_edge_us < FULL_THROTTLE_DEBOUNCE_US) return;
  full_throttle_last_edge_us = nowUs;
  // Pulled up + grounded-when-active: LOW means the switch/sensor is asserting full throttle.
  full_throttle_value = (digitalRead(PIN_FULL_THROTTLE) == LOW) ? 1 : 0;
  full_throttle_capture_us = nowUs;
  full_throttle_pending = true;
}

// =========================================================================
// SENSOR DATA TYPES & BUFFERING
// =========================================================================
struct __attribute__((__packed__)) SensorPacket {
  uint16_t header = 0xAABB;  
  uint16_t rpm1   = 0;       
  uint16_t rpm2   = 0;       
  uint16_t shift  = 0;       
  int32_t  torq1  = 0;       
  int32_t  torq2  = 0;       
  // Per-channel capture timestamps: when each value was physically measured, independent of when
  // it's read here or transmitted. RPM channels get this from RpmCounter (last edge used in the
  // reciprocal-counting window); shift/torque get it at the moment they're read below.
  uint64_t t_rpm1  = 0;
  uint64_t t_rpm2  = 0;
  uint64_t t_shift = 0;
  uint64_t t_torq1 = 0;
  uint64_t t_torq2 = 0;
};

volatile SensorPacket shared_data;

void updateIntervals();

// =========================================================================
// CORE 1: DYNAMIC HARDWARE SENSOR COLLECTOR
// =========================================================================
void setup1() {
  analogReadResolution(12);

  // Optoisolator outputs are expected to be open-collector/open-drain.
  // Keep both frequency inputs high when the optocoupler is off.
  pinMode(PIN_RPM1, INPUT_PULLUP);
  pinMode(PIN_RPM2, INPUT_PULLUP);
  
  RpmCounter::begin(PIN_RPM1, PIN_RPM2, RPM1_SPOKES, RPM2_SPOKES);

  pinMode(PIN_ADS_CS, OUTPUT);
  pinMode(PIN_ADS_RST, OUTPUT);
  pinMode(PIN_ADS_DRDY, INPUT);
  digitalWrite(PIN_ADS_CS, HIGH);
  digitalWrite(PIN_ADS_RST, HIGH);
  delay(10);
  SPI.begin(); 
}

int32_t readADS1256(uint8_t channel) {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_ADS_CS, LOW);
  SPI.transfer(0x50 | ADS_REG_MUX);
  SPI.transfer(0x00);
  SPI.transfer((channel << 4) | 0x08);
  delayMicroseconds(5); 
  SPI.transfer(ADS_CMD_RDATA);
  delayMicroseconds(7);
  uint8_t b1 = SPI.transfer(0);
  uint8_t b2 = SPI.transfer(0);
  uint8_t b3 = SPI.transfer(0);
  digitalWrite(PIN_ADS_CS, HIGH);
  SPI.endTransaction();
  
  int32_t regData = ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | b3;
  if (regData & 0x00800000) regData |= 0xFF000000; 
  return regData;
}

void loop1() {
  SensorPacket local_packet;
  noInterrupts();
  local_packet.rpm1 = shared_data.rpm1;
  local_packet.rpm2 = shared_data.rpm2;
  local_packet.shift = shared_data.shift;
  local_packet.torq1 = shared_data.torq1;
  local_packet.torq2 = shared_data.torq2;
  local_packet.t_rpm1 = shared_data.t_rpm1;
  local_packet.t_rpm2 = shared_data.t_rpm2;
  local_packet.t_shift = shared_data.t_shift;
  local_packet.t_torq1 = shared_data.t_torq1;
  local_packet.t_torq2 = shared_data.t_torq2;
  interrupts();

  uint32_t measuredRpm1 = 0;
  uint32_t measuredRpm2 = 0;
  uint64_t rpm1CaptureUs = 0;
  uint64_t rpm2CaptureUs = 0;
  bool rpm1Ready = false;
  bool rpm2Ready = false;
  RpmCounter::update(measuredRpm1, measuredRpm2, rpm1CaptureUs, rpm2CaptureUs, rpm1Ready, rpm2Ready);

  // Bench mode leaves the hardware setup intact but bypasses all sensor reads.
  // Disable it over serial to return to the real sensor path without reflashing.
  if (demo_mode) {
    const float phase = millis() / 1000.0f;
    const uint64_t nowUs = time_us_64();
    local_packet.rpm1 = 4200 + (sin(phase) * 1100) + (phase * 18);
    local_packet.rpm2 = 2850 + (sin(phase - 0.55f) * 720) + (phase * 12);
    local_packet.shift = 1800 + (sin(phase * 0.45f) * 850);
    local_packet.torq1 = 420 + (sin(phase * 0.8f) * 105);
    local_packet.torq2 = 335 + (sin((phase * 0.8f) - 0.3f) * 88);
    local_packet.t_rpm1 = nowUs;
    local_packet.t_rpm2 = nowUs;
    local_packet.t_shift = nowUs;
    local_packet.t_torq1 = nowUs;
    local_packet.t_torq2 = nowUs;
  } else {
    // RPM1/RPM2 are independently edge-triggered (see RpmCounter) -- each is only overwritten
    // here when its own channel actually produced a fresh reading, never gated by the other.
    if (rpm1Ready) {
      local_packet.rpm1 = measuredRpm1;
      local_packet.t_rpm1 = rpm1CaptureUs;
    }
    if (rpm2Ready) {
      local_packet.rpm2 = measuredRpm2;
      local_packet.t_rpm2 = rpm2CaptureUs;
    }

    local_packet.shift = analogRead(PIN_SHIFT);
    local_packet.t_shift = time_us_64();

    if (digitalRead(PIN_ADS_DRDY) == LOW) {
      local_packet.torq1 = readADS1256(0);
      local_packet.torq2 = readADS1256(1);
      const uint64_t torqueUs = time_us_64();
      local_packet.t_torq1 = torqueUs;
      local_packet.t_torq2 = torqueUs;
    }
  }

  noInterrupts();
  shared_data.rpm1  = local_packet.rpm1;
  shared_data.rpm2  = local_packet.rpm2;
  shared_data.shift = local_packet.shift;
  shared_data.torq1 = local_packet.torq1;
  shared_data.torq2 = local_packet.torq2;
  shared_data.t_rpm1  = local_packet.t_rpm1;
  shared_data.t_rpm2  = local_packet.t_rpm2;
  shared_data.t_shift = local_packet.t_shift;
  shared_data.t_torq1 = local_packet.t_torq1;
  shared_data.t_torq2 = local_packet.t_torq2;
  interrupts();
}

// =========================================================================
// CORE 0: TELEMETRY STREAMER & LIVE COMMAND PARSER
// =========================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize intervals based on default startup matrix
  updateIntervals();

  pinMode(PIN_FULL_THROTTLE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FULL_THROTTLE), fullThrottleChange, CHANGE);
  // Seed the current state as a pending send so a receiver learns it immediately at connect
  // instead of waiting for the first actual transition (which, depending on the run, might not
  // happen for a while).
  full_throttle_value = (digitalRead(PIN_FULL_THROTTLE) == LOW) ? 1 : 0;
  full_throttle_capture_us = time_us_64();
  full_throttle_pending = true;
}

void updateIntervals() {
  noInterrupts();
  for (int i = 0; i < 5; i++) {
    if (cfg_freq[i] > 0) {
      intervals_us[i] = (1.0 / (float)cfg_freq[i]) * 1000000.0;
    } else {
      intervals_us[i] = 4294967295; // Max out interval if frequency is set to 0
    }
  }
  interrupts();
}

// Helper to print out human-readable configuration profiles back over serial
void printCurrentConfig() {
  const char* labels[] = {"RPM1", "RPM2", "SHIFT", "TORQ1", "TORQ2"};
  Serial.println("\n--- CURRENT CONFIGURATION STATUS ---");
  Serial.print("Bench mode: "); Serial.println(demo_mode ? "ENABLED" : "DISABLED");
  Serial.print("RPM pin test: "); Serial.println(rpm_pin_test ? "ENABLED" : "DISABLED");
  Serial.print("RPM interrupt test: "); Serial.println(rpm_interrupt_test ? "ENABLED" : "DISABLED");
  Serial.print("RPM count test: "); Serial.println(rpm_count_test ? "ENABLED" : "DISABLED");
  
  uint16_t primarySpokes = 1;
  uint16_t secondarySpokes = 1;
  RpmCounter::getSpokes(primarySpokes, secondarySpokes);
  Serial.print("RPM Spokes - PRIMARY: "); Serial.print(primarySpokes);
  Serial.print(" | SECONDARY: "); Serial.println(secondarySpokes);

  uint16_t primaryEdgesPerUpdate = 1;
  uint16_t secondaryEdgesPerUpdate = 1;
  RpmCounter::getEdgesPerUpdate(primaryEdgesPerUpdate, secondaryEdgesPerUpdate);
  Serial.print("RPM Edges/Update - PRIMARY: "); Serial.print(primaryEdgesPerUpdate);
  Serial.print(" | SECONDARY: "); Serial.println(secondaryEdgesPerUpdate);
  Serial.print("Full throttle input: "); Serial.println((digitalRead(PIN_FULL_THROTTLE) == LOW) ? "FULL THROTTLE" : "NOT FULL THROTTLE");
  
  for (int i = 0; i < 5; i++) {
    Serial.print("Channel ["); Serial.print(i); Serial.print("] ("); Serial.print(labels[i]); Serial.print("): ");
    Serial.print(cfg_write_en[i] ? "ENABLED" : "DISABLED");
    Serial.print(" | Target Tx Freq: "); Serial.print(cfg_freq[i]); Serial.println(" Hz");
  }
  Serial.println("------------------------------------\n");
}

void printRpmPinDiagnostics() {
  uint32_t primaryEdges = 0;
  uint32_t secondaryEdges = 0;
  RpmCounter::readDiagnostics(primaryEdges, secondaryEdges);
  Serial.print("RPM TEST | PIN_RPM1=");
  Serial.print(digitalRead(PIN_RPM1) == HIGH ? "HIGH" : "LOW");
  Serial.print(" edges=");
  Serial.print(primaryEdges);
  Serial.print(" | PIN_RPM2=");
  Serial.print(digitalRead(PIN_RPM2) == HIGH ? "HIGH" : "LOW");
  Serial.print(" edges=");
  Serial.println(secondaryEdges);
}

void printRpmCountDiagnostics() {
  uint32_t primaryCount = 0;
  uint32_t secondaryCount = 0;
  RpmCounter::readWindowCounts(primaryCount, secondaryCount);
  Serial.print("RPM COUNT TEST | RPM1 count=");
  Serial.print(primaryCount);
  Serial.print(" | RPM2 count=");
  Serial.println(secondaryCount);
}

void printRpmInterruptDiagnostics() {
  uint32_t primaryEvents = 0;
  uint32_t secondaryEvents = 0;
  RpmCounter::readAndClearInterruptEvents(primaryEvents, secondaryEvents);
  Serial.print("RPM INTERRUPT TEST | PRIMARY edges=");
  Serial.print(primaryEvents);
  Serial.print(" | SECONDARY edges=");
  Serial.println(secondaryEvents);
}

void handleIncomingCommands() {
  // Discard any bytes that aren't the command sync byte first, so a single dropped/corrupted byte
  // can only cost the one malformed command instead of permanently misaligning every command
  // parsed afterward (the previous fixed-4-byte parser had no way to recover from that).
  while (Serial.available() > 0 && Serial.peek() != COMMAND_SYNC) {
    Serial.read();
  }
  if (Serial.available() < COMMAND_PACKET_LEN) return;

  Serial.read(); // consume sync byte
  uint8_t cmd  = Serial.read();
  uint8_t ch   = Serial.read();
  uint8_t valH = Serial.read();
  uint8_t valL = Serial.read();
  uint16_t combined_val = ((uint16_t)valH << 8) | valL;

  if (cmd == 0x04) {
    demo_mode = (combined_val == 1);
    Serial.println(demo_mode ? "BENCH MODE ENABLED" : "BENCH MODE DISABLED");
  } else if (cmd == 0x05) {
    rpm_pin_test = (combined_val == 1);
    Serial.println(rpm_pin_test ? "RPM PIN TEST ENABLED" : "RPM PIN TEST DISABLED");
  } else if (ch <= 4) {
    if (cmd == 0x01) { 
      // Command 1: Toggle stream active states
      cfg_write_en[ch] = (combined_val == 1);
    } 
    else if (cmd == 0x02) { 
      // Command 2: Re-map target execution speeds (independent per channel)
      cfg_freq[ch] = combined_val;
      updateIntervals();
    }
  }
  
  if (cmd == 0x03) {
    // Command 3: Return text-dump overview profile
    printCurrentConfig();
  } else if (cmd == 0x06) {
    // Command 6: Set RPM spoke counts (channel 0 or 1, value is spoke count)
    if (ch <= 1) {
      RpmCounter::setSpokes(ch, combined_val);
      Serial.print("RPM Spokes updated - Channel ");
      Serial.print(ch == 0 ? "PRIMARY" : "SECONDARY");
      Serial.print(": ");
      Serial.println(combined_val);
    }
  } else if (cmd == 0x07) {
    // Command 7: Toggle RPM interrupt test mode
    rpm_interrupt_test = (combined_val == 1);
    RpmCounter::setInterruptTestMode(rpm_interrupt_test);
    Serial.println(rpm_interrupt_test ? "RPM INTERRUPT TEST ENABLED" : "RPM INTERRUPT TEST DISABLED");
  } else if (cmd == 0x08) {
    // Command 8: Toggle RPM count test mode
    rpm_count_test = (combined_val == 1);
    Serial.println(rpm_count_test ? "RPM COUNT TEST ENABLED" : "RPM COUNT TEST DISABLED");
  } else if (cmd == 0x09) {
    // Command 9: Set RPM edges-per-update (channel 0 or 1, value is edge count spanned per
    // reciprocal-counting computation). 1 = recompute on every edge (fastest, default); higher
    // values trade update latency for immunity to tooth-spacing manufacturing tolerance.
    if (ch <= 1) {
      RpmCounter::setEdgesPerUpdate(ch, combined_val);
      Serial.print("RPM Edges/Update updated - Channel ");
      Serial.print(ch == 0 ? "PRIMARY" : "SECONDARY");
      Serial.print(": ");
      Serial.println(combined_val);
    }
  }
}

void loop() {
  SensorPacket local_packet;
  noInterrupts();
  local_packet.rpm1 = shared_data.rpm1;
  local_packet.rpm2 = shared_data.rpm2;
  local_packet.shift = shared_data.shift;
  local_packet.torq1 = shared_data.torq1;
  local_packet.torq2 = shared_data.torq2;
  local_packet.t_rpm1 = shared_data.t_rpm1;
  local_packet.t_rpm2 = shared_data.t_rpm2;
  local_packet.t_shift = shared_data.t_shift;
  local_packet.t_torq1 = shared_data.t_torq1;
  local_packet.t_torq2 = shared_data.t_torq2;
  interrupts();
  handleIncomingCommands();

  if (rpm_pin_test && millis() - last_rpm_pin_test_ms >= 100) {
    last_rpm_pin_test_ms = millis();
    printRpmPinDiagnostics();
  }

  if (rpm_count_test && millis() - last_rpm_count_test_ms >= 100) {
    last_rpm_count_test_ms = millis();
    printRpmCountDiagnostics();
  }

  if (rpm_interrupt_test && millis() - last_rpm_interrupt_test_ms >= 100) {
    last_rpm_interrupt_test_ms = millis();
    printRpmInterruptDiagnostics();
  }

  unsigned long now = micros();

  // Each enabled channel is scheduled and transmitted completely independently -- one channel
  // running at a high rate (e.g. torque in the future) never throttles or is throttled by another
  // channel's rate, since each has its own interval, own timestamp, and its own packet.
  for (int i = 0; i < 5; i++) {
    if (cfg_write_en[i] && (now - last_tx_us[i] >= intervals_us[i])) {
      last_tx_us[i] = now;

      int32_t payload_val = 0;
      uint64_t capture_us = 0;

      switch (i) {
        case 0: payload_val = (int32_t)local_packet.rpm1;  capture_us = local_packet.t_rpm1;  break;
        case 1: payload_val = (int32_t)local_packet.rpm2;  capture_us = local_packet.t_rpm2;  break;
        case 2: payload_val = (int32_t)local_packet.shift; capture_us = local_packet.t_shift; break;
        case 3: payload_val = (int32_t)local_packet.torq1; capture_us = local_packet.t_torq1; break;
        case 4: payload_val = (int32_t)local_packet.torq2; capture_us = local_packet.t_torq2; break;
      }

      uint8_t packet[TELEMETRY_PACKET_LEN];
      packet[0] = TELEMETRY_SYNC0;
      packet[1] = TELEMETRY_SYNC1;
      packet[2] = (uint8_t)i;
      packet[3] = tx_seq[i]++;
      memcpy(&packet[4], &payload_val, 4);
      memcpy(&packet[8], &capture_us, 8);
      packet[16] = crc8(&packet[2], TELEMETRY_CRC_SPAN);

      Serial.write(packet, TELEMETRY_PACKET_LEN);
    }
  }

  // Full throttle (channel 5): sent immediately on change, not on a schedule -- checked and
  // cleared every loop() iteration rather than written directly from the ISR (see the comment on
  // full_throttle_pending above).
  noInterrupts();
  const bool throttlePending = full_throttle_pending;
  const uint8_t throttleValue = full_throttle_value;
  const uint64_t throttleCaptureUs = full_throttle_capture_us;
  full_throttle_pending = false;
  interrupts();
  if (throttlePending) {
    int32_t payload_val = (int32_t)throttleValue;
    uint8_t packet[TELEMETRY_PACKET_LEN];
    packet[0] = TELEMETRY_SYNC0;
    packet[1] = TELEMETRY_SYNC1;
    packet[2] = CHANNEL_FULL_THROTTLE;
    packet[3] = tx_seq[CHANNEL_FULL_THROTTLE]++;
    memcpy(&packet[4], &payload_val, 4);
    memcpy(&packet[8], &throttleCaptureUs, 8);
    packet[16] = crc8(&packet[2], TELEMETRY_CRC_SPAN);
    Serial.write(packet, TELEMETRY_PACKET_LEN);
  }
  // No per-iteration Serial.flush(): on USB-CDC that blocks until the host has drained the
  // buffer, adding needless latency/jitter to every loop iteration. Let the USB stack batch
  // writes naturally -- at these data rates (well under 1% of USB-CDC's throughput) nothing is
  // lost, it's just no longer forced out packet-by-packet.
}
