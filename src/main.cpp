#include <SPI.h>
#include <string.h>
#include <pico/time.h> // time_us_64()
#include <Adafruit_TinyUSB.h> // Vendor-class (WebUSB/WinUSB) USB transport -- see USB TELEMETRY
                              // PROTOCOL comment below for why this replaces USB-CDC ("Serial").
#include <RpmCounter.h>
#include <ConfigStore.h>

// Injected by extra_script_version.py (see platformio.ini) from `git rev-parse`; falls back here
// only if built outside a git checkout (e.g. from a source zip with no .git directory).
#ifndef FIRMWARE_GIT_SHA
#define FIRMWARE_GIT_SHA "unknown"
#endif

// Bump this whenever a change alters wire-level semantics the viewer must know about (e.g. a
// channel's payload meaning changes, like RPM's periodUs=0 meaning shifting from "first edge,
// nothing to diff against" to an explicit "stopped" report) -- NOT for purely additive/internal
// changes that don't change how existing bytes should be interpreted. The viewer reports its own
// expected value (see EXPECTED_PROTOCOL_VERSION in protocol.ts) and warns loudly on a mismatch, so
// a stale-firmware-vs-viewer mismatch is caught immediately on connect instead of silently
// misbehaving in a way that takes a live debugging session to track down (as happened once before
// this existed).
#define PROTOCOL_VERSION 1

// --- PIN DEFINITIONS ---
const int PIN_RPM1   = 3;  // Must be ODD
const int PIN_RPM2   = 1;  // Must be ODD
const int PIN_SHIFT  = 26; // Analog A0
const int PIN_ADS_CS   = 17; 
const int PIN_ADS_DRDY = 20; 
const int PIN_ADS_RST  = 21; 
const int PIN_FULL_THROTTLE = 0; // Binary input, pulled up; grounded (LOW) = full throttle

// Bench/demo-mode only: synthetic tooth counts used purely to generate a realistic per-tooth
// cadence for the sine-wave demo RPM signal (see loop1()'s demo_mode branch). The real hardware
// path does NOT use a spoke/tooth count on-device at all -- RPM channels stream a raw inter-edge
// period per physical tooth (see RpmCounter::popEdge()), and reconstructing RPM from that period
// (rpm = 60e6 / (period_us * spokes)) is entirely a host-side concern, so a spoke-count change
// never needs a firmware round-trip.
const uint16_t DEMO_RPM1_TEETH = 16;
const uint16_t DEMO_RPM2_TEETH = 12;

#define ADS_CMD_RDATA  0x01
#define ADS_REG_MUX    0x01

// USB transport: vendor-class WebUSB interface (see USB TELEMETRY PROTOCOL comment below). This
// is a Stream, exposing the same available()/peek()/read()/write()/print() API "Serial" did, so
// the rest of this file uses it as a near drop-in replacement for the old CDC object.
Adafruit_USBD_WebUSB usb_web;

// =========================================================================
// USB TELEMETRY PROTOCOL v2 (transport: WebUSB vendor interface, not USB-CDC)
// =========================================================================
// This device deliberately does NOT expose a virtual COM port. Serial.begin() is never called, so
// the CDC interface (Adafruit_USBD_CDC, aka "Serial" under Adafruit TinyUSB) never gets added to
// the enumerated USB descriptor -- only the vendor-class WebUSB interface (`usb_web` below) is
// exposed. On Windows this auto-binds to WinUSB via WebUSB's MS OS 2.0 descriptors (no Zadig, no
// COM port). The byte-level framing below (sync bytes, seq, CRC-8) is unchanged from the original
// CDC-based design and read/written the same way, just over `usb_web` (a Stream, like Serial was)
// instead of `Serial`.
//
// Telemetry packet (17 bytes). Channels 2-4 (shift/torque) are sent on their own configurable
// polling rate (see cfg_freq[] below) -- e.g. torque can run much faster than shift without
// affecting shift's cadence, and vice versa. Channels 0-1 (RPM1/RPM2) are edge-triggered instead
// of polled: one packet is sent per physical tooth as soon as it's detected, with NO fixed rate --
// cfg_freq[0]/cfg_freq[1] have no effect (see the comment on cfg_freq[] below).
//   [0]    0xAA  sync byte 0
//   [1]    0x55  sync byte 1
//   [2]    channel id (0..5)
//   [3]    per-channel rolling sequence number (drop detection on the receiving end)
//   [4..7] int32 value, little-endian --
//           for channels 0-1 (RPM1/RPM2) this is the raw inter-edge period in microseconds since
//           the previous tooth on that channel, NOT an RPM value -- the host reconstructs RPM
//           itself: rpm = 60,000,000 / (period_us * teeth_per_revolution), since spoke/tooth count
//           is a host-side display concern, not firmware state. A value of exactly 0 is an
//           explicit "this channel has stopped" report (see RpmCounter::pollStale()), pushed once
//           after RPM_STALE_TIMEOUT_US with no real edge -- a real reading meant to be applied as
//           RPM==0, not a marker to be ignored. A period is only ever computed from two actual
//           edges, so periodUs is never fabricated as 0 for "first edge, nothing to diff against"
//           the way an earlier version of this protocol did.
//           for channels 2-4 (shift/torque) this is the polled sensor value, unchanged.
//   [8..15] uint64 firmware capture timestamp (time_us_64()), little-endian --
//           stamped at the moment the value was physically true (the exact tooth-edge timestamp
//           for RPM channels, or the moment read for shift/torque), not when this packet happened
//           to be sent, so a receiver can rebuild an accurate per-channel timeline even though
//           channels arrive at different rates.
//   [16]   CRC-8 (poly 0x07, init 0x00) over bytes [2..15]
//
// Channel 5 (full throttle) is also a binary input reported purely on change, not on a schedule --
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
//
// Command IDs 0x06 (set RPM spoke count) and 0x09 (set RPM edges-per-update) are RESERVED/removed
// -- both were device-side RPM computation knobs that no longer apply now that RPM channels stream
// a raw per-tooth period and the host does all RPM reconstruction/smoothing (see the telemetry
// packet comment above). Sending them is a no-op (unrecognized command id, silently ignored).
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
// Frequencies in Hz, independent per channel -- but cfg_freq[0]/cfg_freq[1] (RPM1/RPM2) are
// vestigial: still settable via command 0x02 for wire compatibility, but have no effect, since RPM
// channels stream one packet per physical tooth (edge-triggered) rather than being polled at a
// configured rate. Only indices 2-4 (SHIFT, T1, T2) actually drive scheduling in loop() below.
volatile uint16_t cfg_freq[5] = {0, 0, 10, 50, 50};
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

// Tracked purely for the "Persisted config" line in printCurrentConfig() -- true if a valid
// previously-saved configuration was found in flash at boot (see ConfigStore::load()).
bool config_loaded_from_flash = false;

// --- Full throttle input (channel 5) -------------------------------------------------------
// Reported purely on change via interrupt, with no polling rate to configure. The ISR only
// captures the new state and timestamp and sets a pending flag -- it deliberately does not call
// usb_web.write() itself, since that could interrupt an in-progress write from the main scheduler
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
// RPM (channels 0/1) is intentionally NOT part of this struct -- it no longer flows through a
// polled shared_data snapshot at all. Real RPM edges go straight from the pin ISR into
// RpmCounter's per-channel ring buffers (see RpmCounter::popEdge()), which loop() (core0) drains
// and transmits directly, decoupled from this core1->core0 polling handoff entirely. This also
// removes a hazard the old design had: a single-slot rpm1/rpm2 here could only ever hold the most
// recent value, silently overwriting/losing any edge that landed between loop1() iterations.
struct __attribute__((__packed__)) SensorPacket {
  uint16_t header = 0xAABB;  
  uint16_t shift  = 0;       
  int32_t  torq1  = 0;       
  int32_t  torq2  = 0;       
  // Per-channel capture timestamps: when each value was physically measured, independent of when
  // it's read here or transmitted.
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
  
  RpmCounter::begin(PIN_RPM1, PIN_RPM2);

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

// Demo-mode-only synthetic tooth generator (bench testing without real hardware). Injects
// synthetic edges through the exact same RpmCounter ring buffer real edges use, gated by elapsed
// time so it authentically produces one edge every `periodUs` rather than flooding the ring at
// loop1()'s full iteration rate. State is core1-only (loop1() never runs anywhere else), so no
// locking is needed here.
void injectDemoRpmEdges(float phase) {
  static uint64_t lastDemoEdgeUs[2] = {0, 0};
  const uint64_t nowUs = time_us_64();

  // Two superimposed sine waves (a fast oscillation plus a much slower "drift" one) instead of a
  // fast oscillation plus a raw linear `phase * k` term -- the linear term grew without bound for
  // as long as bench mode stayed enabled (confirmed live: primary RPM reaching ~35,000+ after a
  // few minutes, since phase is elapsed seconds and never resets). A slow sine wave gives the same
  // "not just a fixed, repetitive oscillation" demo character while staying bounded indefinitely,
  // however long a bench session runs.
  const float demoRpm1 = 4200 + (sin(phase) * 1100) + (sin(phase * 0.015f) * 500);
  const float demoRpm2 = 2850 + (sin(phase - 0.55f) * 720) + (sin((phase * 0.011f) - 0.3f) * 350);
  // One "tooth" every 60e6 / (rpm * teeth) us -- mirrors the real reciprocal-counting relationship
  // (see the telemetry packet comment on channels 0/1) so demo mode exercises a realistic cadence.
  const uint32_t periodUs1 = (demoRpm1 > 0) ? (uint32_t)(60000000.0f / (demoRpm1 * DEMO_RPM1_TEETH)) : 0;
  const uint32_t periodUs2 = (demoRpm2 > 0) ? (uint32_t)(60000000.0f / (demoRpm2 * DEMO_RPM2_TEETH)) : 0;

  if (periodUs1 > 0 && (nowUs - lastDemoEdgeUs[0]) >= periodUs1) {
    RpmCounter::injectSyntheticEdge(0, periodUs1);
    lastDemoEdgeUs[0] = nowUs;
  }
  if (periodUs2 > 0 && (nowUs - lastDemoEdgeUs[1]) >= periodUs2) {
    RpmCounter::injectSyntheticEdge(1, periodUs2);
    lastDemoEdgeUs[1] = nowUs;
  }
}

void loop1() {
  SensorPacket local_packet;
  noInterrupts();
  local_packet.shift = shared_data.shift;
  local_packet.torq1 = shared_data.torq1;
  local_packet.torq2 = shared_data.torq2;
  local_packet.t_shift = shared_data.t_shift;
  local_packet.t_torq1 = shared_data.t_torq1;
  local_packet.t_torq2 = shared_data.t_torq2;
  interrupts();

  // Checked every iteration regardless of demo_mode -- pushes an explicit "stopped" (periodUs=0)
  // event for either RPM channel once it's gone RPM_STALE_TIMEOUT_US without a real edge. See
  // RpmCounter::pollStale()/popEdge() for why this makes RPM actually reach zero on the wire
  // instead of the last nonzero reading being displayed forever after the wheel stops turning.
  RpmCounter::pollStale();

  // Bench mode leaves the hardware setup intact but bypasses all sensor reads. RPM is NOT
  // computed/gated here at all anymore (real or demo) -- real edges flow ISR-straight into
  // RpmCounter's ring buffers, and demo edges are injected into the exact same rings by
  // injectDemoRpmEdges() below, so both paths are drained identically by loop() on core0.
  // Disable demo mode over USB to return to the real sensor path without reflashing.
  if (demo_mode) {
    const float phase = millis() / 1000.0f;
    const uint64_t nowUs = time_us_64();
    injectDemoRpmEdges(phase);
    local_packet.shift = 1800 + (sin(phase * 0.45f) * 850);
    local_packet.torq1 = 420 + (sin(phase * 0.8f) * 105);
    local_packet.torq2 = 335 + (sin((phase * 0.8f) - 0.3f) * 88);
    local_packet.t_shift = nowUs;
    local_packet.t_torq1 = nowUs;
    local_packet.t_torq2 = nowUs;
  } else {
    // Real sensor reads used to run completely unconditionally here -- analogRead(PIN_SHIFT) on
    // every single loop1() iteration (as fast as core1 could spin, regardless of whether shift was
    // even enabled or what rate it was configured to send at) and the ADS1256/SPI torque read
    // whenever DRDY happened to be ready, again regardless of cfg_write_en[]. Besides being wasted
    // work when a channel is disabled, continuous unthrottled ADC/SPI bus activity is a plausible
    // contributor to the electrical noise observed live on the physically-nearby RPM1 input (see
    // MIN_VALID_PERIOD_US in RpmCounter.cpp) -- these reads now only happen when their channel is
    // actually enabled, and no faster than intervals_us[] (the same per-channel rate cfg_freq[]
    // already drives on the TX side in loop()), instead of firing at whatever rate the loop
    // happens to spin or DRDY happens to toggle.
    static unsigned long last_shift_read_us = 0;
    static unsigned long last_torque_read_us = 0;
    const unsigned long nowUsCore1 = micros();

    if (cfg_write_en[2] && (nowUsCore1 - last_shift_read_us >= intervals_us[2])) {
      last_shift_read_us = nowUsCore1;
      local_packet.shift = analogRead(PIN_SHIFT);
      local_packet.t_shift = time_us_64();
    }

    // Both torque channels share one physical ADS1256 conversion/SPI transaction, so they're
    // gated together at whichever enabled channel wants the faster rate -- reading at a rate
    // neither enabled channel actually wants transmitted would be pure wasted SPI bus activity.
    const bool torqueWanted = cfg_write_en[3] || cfg_write_en[4];
    const unsigned long torqueIntervalUs = !cfg_write_en[3] ? intervals_us[4]
                                          : !cfg_write_en[4] ? intervals_us[3]
                                          : min(intervals_us[3], intervals_us[4]);
    if (torqueWanted && (nowUsCore1 - last_torque_read_us >= torqueIntervalUs) && digitalRead(PIN_ADS_DRDY) == LOW) {
      last_torque_read_us = nowUsCore1;
      const uint64_t torqueUs = time_us_64();
      if (cfg_write_en[3]) { local_packet.torq1 = readADS1256(0); local_packet.t_torq1 = torqueUs; }
      if (cfg_write_en[4]) { local_packet.torq2 = readADS1256(1); local_packet.t_torq2 = torqueUs; }
    }
  }

  noInterrupts();
  shared_data.shift = local_packet.shift;
  shared_data.torq1 = local_packet.torq1;
  shared_data.torq2 = local_packet.torq2;
  shared_data.t_shift = local_packet.t_shift;
  shared_data.t_torq1 = local_packet.t_torq1;
  shared_data.t_torq2 = local_packet.t_torq2;
  interrupts();
}

// =========================================================================
// CORE 0: TELEMETRY STREAMER & LIVE COMMAND PARSER
// =========================================================================
void setup() {
  // Load any previously-saved config before anything else needs it. write_en[]/freq[] are owned
  // by core0 and can be applied immediately -- unlike before, there's no RPM-spoke handshake with
  // core1 to stage anymore, since spoke count no longer lives on-device at all (see ConfigStore.h).
  ConfigStore::begin();
  ConfigStore::RuntimeConfig loadedConfig;
  if (ConfigStore::load(loadedConfig)) {
    for (int i = 0; i < 5; i++) {
      cfg_write_en[i] = loadedConfig.write_en[i];
      cfg_freq[i] = loadedConfig.freq[i];
    }
    config_loaded_from_flash = true;
  }

  // Bring up the vendor-class WebUSB interface. Deliberately no Serial.begin() anywhere in this
  // sketch -- that's what keeps the CDC/COM-port interface out of the enumerated USB descriptor
  // (see USB TELEMETRY PROTOCOL comment above `usb_web`'s declaration).
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  usb_web.begin();
  // If TinyUSB already auto-enumerated before usb_web.begin() added its interface, force a
  // re-enumeration so the host actually sees the vendor interface in the descriptor set.
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
  while (!TinyUSBDevice.mounted()) { delay(1); }

  // Initialize intervals based on the (possibly just-loaded) startup matrix
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

// Helper to print out human-readable configuration profiles back over USB
void printCurrentConfig() {
  const char* labels[] = {"RPM1", "RPM2", "SHIFT", "TORQ1", "TORQ2"};
  usb_web.println("\n--- CURRENT CONFIGURATION STATUS ---");
  // Printed first and unconditionally (not gated on any test/diagnostic flag) since command 0x03
  // is sent automatically right after every connect -- see FIRMWARE_GIT_SHA/PROTOCOL_VERSION above
  // for why the viewer needs these on every single connection, not just when a human asks for them.
  usb_web.print("Firmware git: "); usb_web.println(FIRMWARE_GIT_SHA);
  usb_web.print("Protocol version: "); usb_web.println(PROTOCOL_VERSION);
  usb_web.print("Bench mode: "); usb_web.println(demo_mode ? "ENABLED" : "DISABLED");
  usb_web.print("RPM pin test: "); usb_web.println(rpm_pin_test ? "ENABLED" : "DISABLED");
  usb_web.print("RPM interrupt test: "); usb_web.println(rpm_interrupt_test ? "ENABLED" : "DISABLED");
  usb_web.print("RPM count test: "); usb_web.println(rpm_count_test ? "ENABLED" : "DISABLED");
  usb_web.print("Persisted config: "); usb_web.println(config_loaded_from_flash ? "LOADED FROM FLASH" : "DEFAULTS (no valid saved config found)");
  usb_web.print("Full throttle input: "); usb_web.println((digitalRead(PIN_FULL_THROTTLE) == LOW) ? "FULL THROTTLE" : "NOT FULL THROTTLE");
  
  for (int i = 0; i < 5; i++) {
    usb_web.print("Channel ["); usb_web.print(i); usb_web.print("] ("); usb_web.print(labels[i]); usb_web.print("): ");
    usb_web.print(cfg_write_en[i] ? "ENABLED" : "DISABLED");
    // RPM channels (0/1) are edge-triggered, not polled -- cfg_freq[] is vestigial for them (see
    // its declaration comment), so printing "Target Tx Freq: 0 Hz" would misleadingly look like a
    // broken/disabled polling rate instead of correctly reflecting "no fixed rate applies here".
    if (i <= 1) {
      usb_web.println(" | Edge-triggered (one packet per tooth, no fixed rate)");
    } else {
      usb_web.print(" | Target Tx Freq: "); usb_web.print(cfg_freq[i]); usb_web.println(" Hz");
    }
  }
  usb_web.println("------------------------------------\n");
  // Adafruit_USBD_WebUSB::write() only auto-flushes its internal FIFO once a full USB packet
  // (BULK_PACKET_SIZE, 64 bytes at full speed) has accumulated -- see the "no per-iteration flush"
  // comment on the telemetry writes in loop(), which deliberately relies on that for throughput.
  // A one-shot text reply like this has no such guarantee: with nothing else writing to the same
  // endpoint to eventually push it over that threshold, it can sit buffered for an arbitrarily
  // long and unpredictable time (confirmed live: the same command's round-trip time varied from
  // ~100ms to ~1s purely based on whether a telemetry write happened to land soon afterward).
  // Explicit flush() here forces it out immediately regardless -- worth it for a low-frequency,
  // latency-sensitive reply, unlike the high-frequency telemetry stream this optimization exists
  // for in the first place.
  usb_web.flush();
}

void printRpmPinDiagnostics() {
  uint32_t primaryEdges = 0;
  uint32_t secondaryEdges = 0;
  RpmCounter::readDiagnostics(primaryEdges, secondaryEdges);
  usb_web.print("RPM TEST | PIN_RPM1=");
  usb_web.print(digitalRead(PIN_RPM1) == HIGH ? "HIGH" : "LOW");
  usb_web.print(" edges=");
  usb_web.print(primaryEdges);
  usb_web.print(" | PIN_RPM2=");
  usb_web.print(digitalRead(PIN_RPM2) == HIGH ? "HIGH" : "LOW");
  usb_web.print(" edges=");
  usb_web.println(secondaryEdges);
  usb_web.flush(); // see printCurrentConfig()'s flush() comment -- same reasoning applies here
}

void printRpmCountDiagnostics() {
  uint32_t primaryCount = 0;
  uint32_t secondaryCount = 0;
  RpmCounter::readWindowCounts(primaryCount, secondaryCount);
  // Dropped counts surface a real per-tooth streaming backlog (the USB side wasn't draining a
  // channel's ring fast enough) -- should be 0 under normal operation; see RpmCounter::popEdge().
  const uint32_t primaryDropped = RpmCounter::readAndClearDropped(0);
  const uint32_t secondaryDropped = RpmCounter::readAndClearDropped(1);
  // Rejected counts surface implausibly-fast pulses being filtered out (see MIN_VALID_PERIOD_US in
  // RpmCounter.cpp) -- should be 0 with a real sensor under normal operation; a sustained nonzero
  // count here means something is inducing noise on that channel's input pin.
  const uint32_t primaryRejected = RpmCounter::readAndClearRejectedNoise(0);
  const uint32_t secondaryRejected = RpmCounter::readAndClearRejectedNoise(1);
  usb_web.print("RPM COUNT TEST | RPM1 count=");
  usb_web.print(primaryCount);
  usb_web.print(" dropped=");
  usb_web.print(primaryDropped);
  usb_web.print(" rejected=");
  usb_web.print(primaryRejected);
  usb_web.print(" | RPM2 count=");
  usb_web.print(secondaryCount);
  usb_web.print(" dropped=");
  usb_web.print(secondaryDropped);
  usb_web.print(" rejected=");
  usb_web.println(secondaryRejected);
  usb_web.flush(); // see printCurrentConfig()'s flush() comment -- same reasoning applies here
}

void printRpmInterruptDiagnostics() {
  uint32_t primaryEvents = 0;
  uint32_t secondaryEvents = 0;
  RpmCounter::readAndClearInterruptEvents(primaryEvents, secondaryEvents);
  usb_web.print("RPM INTERRUPT TEST | PRIMARY edges=");
  usb_web.print(primaryEvents);
  usb_web.print(" | SECONDARY edges=");
  usb_web.println(secondaryEvents);
  usb_web.flush(); // see printCurrentConfig()'s flush() comment -- same reasoning applies here
}

void handleIncomingCommands() {
  // Discard any bytes that aren't the command sync byte first, so a single dropped/corrupted byte
  // can only cost the one malformed command instead of permanently misaligning every command
  // parsed afterward (the previous fixed-4-byte parser had no way to recover from that).
  while (usb_web.available() > 0 && usb_web.peek() != COMMAND_SYNC) {
    usb_web.read();
  }
  if (usb_web.available() < COMMAND_PACKET_LEN) return;

  usb_web.read(); // consume sync byte
  uint8_t cmd  = usb_web.read();
  uint8_t ch   = usb_web.read();
  uint8_t valH = usb_web.read();
  uint8_t valL = usb_web.read();
  uint16_t combined_val = ((uint16_t)valH << 8) | valL;

  if (cmd == 0x04) {
    demo_mode = (combined_val == 1);
    usb_web.println(demo_mode ? "BENCH MODE ENABLED" : "BENCH MODE DISABLED");
  } else if (cmd == 0x05) {
    rpm_pin_test = (combined_val == 1);
    usb_web.println(rpm_pin_test ? "RPM PIN TEST ENABLED" : "RPM PIN TEST DISABLED");
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
  } else if (cmd == 0x07) {
    // Command 7: Toggle RPM interrupt test mode
    rpm_interrupt_test = (combined_val == 1);
    RpmCounter::setInterruptTestMode(rpm_interrupt_test);
    usb_web.println(rpm_interrupt_test ? "RPM INTERRUPT TEST ENABLED" : "RPM INTERRUPT TEST DISABLED");
  } else if (cmd == 0x08) {
    // Command 8: Toggle RPM count test mode
    rpm_count_test = (combined_val == 1);
    usb_web.println(rpm_count_test ? "RPM COUNT TEST ENABLED" : "RPM COUNT TEST DISABLED");
  }
  // Command IDs 0x06 and 0x09 are reserved/removed (see COMMAND_SYNC comment above) -- any other
  // unrecognized cmd value is silently ignored, same as before.

  // Unconditional (a no-op if this call didn't write anything, e.g. cmd was 0x01/0x02) -- see
  // printCurrentConfig()'s flush() comment for why a command reply needs this explicitly instead
  // of relying on the "let telemetry batch naturally" behavior the high-frequency writes in
  // loop() rely on. printCurrentConfig() (cmd 0x03) already flushes itself at the end of its own
  // much longer output, so this is harmless-redundant for that path, not double-work of any kind.
  usb_web.flush();
}

void loop() {
  SensorPacket local_packet;
  noInterrupts();
  local_packet.shift = shared_data.shift;
  local_packet.torq1 = shared_data.torq1;
  local_packet.torq2 = shared_data.torq2;
  local_packet.t_shift = shared_data.t_shift;
  local_packet.t_torq1 = shared_data.t_torq1;
  local_packet.t_torq2 = shared_data.t_torq2;
  interrupts();
  handleIncomingCommands();

  // Auto-save: build a snapshot of the current live (non-diagnostic) config every iteration and
  // hand it to ConfigStore, which debounces and only actually writes to flash once settings have
  // been stable for a bit -- see ConfigStore::poll() for why.
  {
    ConfigStore::RuntimeConfig snapshot;
    for (int i = 0; i < 5; i++) {
      snapshot.write_en[i] = cfg_write_en[i];
      snapshot.freq[i] = cfg_freq[i];
    }
    ConfigStore::poll(snapshot);
  }

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

  // RPM channels (0/1) are edge-triggered, not scheduled: fully drain each channel's ring buffer
  // every loop() iteration and send exactly one packet per physical tooth, with the tooth's own
  // capture timestamp and raw inter-edge period as the payload (see the telemetry packet comment
  // near the top of this file). cfg_write_en[] still gates whether we bother sending -- if
  // disabled, events are still drained (so the ring doesn't build up stale backlog) but discarded
  // rather than transmitted.
  for (uint8_t ch = 0; ch <= 1; ch++) {
    uint64_t edgeUs = 0;
    uint32_t periodUs = 0;
    while (RpmCounter::popEdge(ch, edgeUs, periodUs)) {
      if (!cfg_write_en[ch]) continue;

      int32_t payload_val = (int32_t)periodUs;
      uint8_t packet[TELEMETRY_PACKET_LEN];
      packet[0] = TELEMETRY_SYNC0;
      packet[1] = TELEMETRY_SYNC1;
      packet[2] = ch;
      packet[3] = tx_seq[ch]++;
      memcpy(&packet[4], &payload_val, 4);
      memcpy(&packet[8], &edgeUs, 8);
      packet[16] = crc8(&packet[2], TELEMETRY_CRC_SPAN);

      usb_web.write(packet, TELEMETRY_PACKET_LEN);
    }
  }

  // Shift/torque (channels 2-4) stay on their own configurable polling rate -- each enabled
  // channel is scheduled and transmitted completely independently, so one channel running at a
  // high rate never throttles or is throttled by another channel's rate.
  for (int i = 2; i <= 4; i++) {
    if (cfg_write_en[i] && (now - last_tx_us[i] >= intervals_us[i])) {
      last_tx_us[i] = now;

      int32_t payload_val = 0;
      uint64_t capture_us = 0;

      switch (i) {
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

      usb_web.write(packet, TELEMETRY_PACKET_LEN);
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
    usb_web.write(packet, TELEMETRY_PACKET_LEN);
  }
  // No per-iteration usb_web.flush(): let the vendor bulk endpoint batch writes naturally instead
  // of forcing a transfer out packet-by-packet, avoiding needless latency/jitter on every loop
  // iteration. At these data rates (well under the full-speed USB bulk endpoint's throughput)
  // nothing is lost by not flushing eagerly.
}
