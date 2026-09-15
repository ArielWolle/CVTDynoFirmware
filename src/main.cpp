#include <SPI.h>
#include <RpmCounter.h>

// --- PIN DEFINITIONS ---
const int PIN_RPM1   = 1;  // Must be ODD
const int PIN_RPM2   = 3;  // Must be ODD
const int PIN_SHIFT  = 26; // Analog A0
const int PIN_ADS_CS   = 17; 
const int PIN_ADS_DRDY = 20; 
const int PIN_ADS_RST  = 21; 

// Number of optoisolator pulses generated per wheel revolution.
// Set these to the physical spoke count for each RPM wheel.
const uint16_t RPM1_SPOKES = 16;
const uint16_t RPM2_SPOKES = 12;

#define ADS_CMD_RDATA  0x01
#define ADS_REG_MUX    0x01

// =========================================================================
// RUNTIME LIVE CONFIGURATION BUFFER (Shared dynamically between cores)
// =========================================================================
volatile bool cfg_write_en[5] = {true, true, true, true, true}; // RPM1, RPM2, SHIFT, T1, T2
volatile uint16_t cfg_freq[5] = {20, 20, 10, 50, 50};           // Frequencies in Hz
volatile bool demo_mode = false;                                // Command 0x04: synthetic bench data
volatile bool rpm_pin_test = false;                             // Command 0x05: diagnostic pin reports
volatile bool rpm_interrupt_test = false;                       // Command 0x07: real-time interrupt logging

// Microsecond tracking variables for independent scheduling on Core 0
unsigned long last_tx_us[5]  = {0, 0, 0, 0, 0};
volatile unsigned long intervals_us[5]; 
unsigned long last_rpm_test_ms = 0;

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
  
  RpmCounter::begin(PIN_RPM1, PIN_RPM2, RPM1_SPOKES, RPM2_SPOKES, 50);

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
  uint32_t measuredRpm1 = 0;
  uint32_t measuredRpm2 = 0;
  const bool rpmReady = RpmCounter::update(measuredRpm1, measuredRpm2);

  // Bench mode leaves the hardware setup intact but bypasses all sensor reads.
  // Disable it over serial to return to the real sensor path without reflashing.
  if (demo_mode) {
    const float phase = millis() / 1000.0f;
    local_packet.rpm1 = 4200 + (sin(phase) * 1100) + (phase * 18);
    local_packet.rpm2 = 2850 + (sin(phase - 0.55f) * 720) + (phase * 12);
    local_packet.shift = 1800 + (sin(phase * 0.45f) * 850);
    local_packet.torq1 = 420 + (sin(phase * 0.8f) * 105);
    local_packet.torq2 = 335 + (sin((phase * 0.8f) - 0.3f) * 88);
  } else {
    if (rpmReady) {
      local_packet.rpm1 = measuredRpm1;
      local_packet.rpm2 = measuredRpm2;
    }

    local_packet.shift = analogRead(PIN_SHIFT);

    if (digitalRead(PIN_ADS_DRDY) == LOW) {
      local_packet.torq1 = readADS1256(0);
      local_packet.torq2 = readADS1256(1);
    }
  }

  noInterrupts();
  shared_data.rpm1  = local_packet.rpm1;
  shared_data.rpm2  = local_packet.rpm2;
  shared_data.shift = local_packet.shift;
  shared_data.torq1 = local_packet.torq1;
  shared_data.torq2 = local_packet.torq2;
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
  
  uint16_t primarySpokes = 1;
  uint16_t secondarySpokes = 1;
  RpmCounter::getSpokes(primarySpokes, secondarySpokes);
  Serial.print("RPM Spokes - PRIMARY: "); Serial.print(primarySpokes);
  Serial.print(" | SECONDARY: "); Serial.println(secondarySpokes);
  
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

void handleIncomingCommands() {
  // Check if a full 4-byte command packet has fully loaded into the USB FIFO cache
  if (Serial.available() >= 4) {
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
        // Command 2: Re-map target execution speeds
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
    }
  }
}

void loop() {
  // Handle any inbound configuration parameters instantly
  handleIncomingCommands();

  if (rpm_pin_test && millis() - last_rpm_test_ms >= 100) {
    last_rpm_test_ms = millis();
    printRpmPinDiagnostics();
  }

  unsigned long now = micros();
  SensorPacket packet_to_send;
  bool data_copied = false;

  // Track if any enabled channel is ready to send
  for (int i = 0; i < 5; i++) {
    if (cfg_write_en[i] && (now - last_tx_us[i] >= intervals_us[i])) {
      last_tx_us[i] = now;
      
      if (!data_copied) {
        noInterrupts();
        packet_to_send.rpm1 = shared_data.rpm1;
        packet_to_send.rpm2 = shared_data.rpm2;
        packet_to_send.shift = shared_data.shift;
        packet_to_send.torq1 = shared_data.torq1;
        packet_to_send.torq2 = shared_data.torq2;
        interrupts();
        data_copied = true;
      }

      // To preserve structure parsing speed, instead of full structs, we send 
      // individual small 8-byte typed packages containing specific item states
      // Structure: [Header 0xAABB] [Channel ID] [Padding Byte] [4 Bytes Integer Data Payload]
      uint16_t sync_head = 0xAABB;
      uint8_t padding = 0x00;
      int32_t payload_val = 0;

      switch(i) {
        case 0: payload_val = (int32_t)packet_to_send.rpm1;  break;
        case 1: payload_val = (int32_t)packet_to_send.rpm2;  break;
        case 2: payload_val = (int32_t)packet_to_send.shift; break;
        case 3: payload_val = (int32_t)packet_to_send.torq1; break;
        case 4: payload_val = (int32_t)packet_to_send.torq2; break;
      }

      Serial.write((uint8_t*)&sync_head, 2);
      uint8_t channel = (uint8_t)i;
      Serial.write(&channel, 1);
      Serial.write(&padding, 1);
      Serial.write((uint8_t*)&payload_val, 4);
    }
  }
  
  if (data_copied) {
    Serial.flush(); // Commit data out immediately if anything was queued
  }
}
