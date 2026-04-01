#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

#define SDA_PIN_1        8
#define SCL_PIN_1        9
#define XSHUT            1
#define SCAN_INTERVAL_MS 3000

// ── Clock Speeds ───────────────────────────────────────────────────
const uint32_t CLOCK_SPEEDS[] = { 10000, 50000, 100000, 400000 };
const char*    CLOCK_LABELS[] = { "10 kHz (slow)", "50 kHz", "100 kHz (standard)", "400 kHz (fast)" };
const uint8_t  NUM_SPEEDS     = 4;

uint32_t scanCount  = 0;
uint32_t g_halfUs   = 5; // half-period in µs (default 100 kHz)

// ── Bit-Bang I2C Primitives (open-drain style) ────────────────────
// Open-drain: driving LOW = pull pin to GND (OUTPUT LOW)
//             releasing = let pull-up do the work (INPUT)
inline void sda_high() { pinMode(SDA_PIN_1, INPUT);               }
inline void sda_low()  { pinMode(SDA_PIN_1, OUTPUT); digitalWrite(SDA_PIN_1, LOW); }
inline void scl_high() { pinMode(SCL_PIN_1, INPUT);               }
inline void scl_low()  { pinMode(SCL_PIN_1, OUTPUT); digitalWrite(SCL_PIN_1, LOW); }
inline bool sda_read() { pinMode(SDA_PIN_1, INPUT); return digitalRead(SDA_PIN_1); }
inline void bb_dly()   { delayMicroseconds(g_halfUs);             }

void bb_setFreq(uint32_t hz) {
  // half-period µs = 500000 / hz  (min 1µs → ~500 kHz ceiling)
  g_halfUs = max(1UL, 500000UL / (unsigned long)hz);
}

void bb_start() {
  sda_high(); bb_dly();
  scl_high(); bb_dly();
  sda_low();  bb_dly(); // SDA falls while SCL high → START
  scl_low();  bb_dly();
}

void bb_stop() {
  sda_low();  bb_dly();
  scl_high(); bb_dly();
  sda_high(); bb_dly(); // SDA rises while SCL high → STOP
}

// Returns true if slave ACKed
bool bb_writeByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    (b & (1 << i)) ? sda_high() : sda_low();
    bb_dly();
    scl_high(); bb_dly();
    scl_low();  bb_dly();
  }
  sda_high();   // release SDA so slave can pull low for ACK
  bb_dly();
  scl_high(); bb_dly();
  bool ack = !sda_read(); // ACK = slave pulls SDA LOW
  scl_low();  bb_dly();
  return ack;
}

// Full address probe: returns true if device ACKed
bool bb_probe(uint8_t addr) {
  bb_start();
  bool acked = bb_writeByte((addr << 1) | 0x00); // write-mode address byte
  bb_stop();
  delayMicroseconds(50); // bus free time between transactions
  return acked;
}

// ── Device Identifier ─────────────────────────────────────────────
void identifyDevice(byte address) {
  switch (address) {
    case 0x0C: Serial.print(" (MLX90393SLW Magnetometer)");  break;
    case 0x10: Serial.print(" (VEML7700-TT LUX)");           break;
    case 0x19: Serial.print(" (LIS2DH12 Accelerometer)");    break;
    case 0x20: Serial.print(" (MCP23017 GPIO Expander)");    break;
    case 0x29: Serial.print(" (VL53L4CDV0DH/1 ToF)");        break;
    case 0x36: Serial.print(" (MAX17048 Battery Monitor)");  break;
    case 0x5A: Serial.print(" (STHS34PF80TR Motion)");       break;
    case 0x68: Serial.print(" (RTC)");                        break;
    case 0x76: Serial.print(" (BME680 Temp)");                break;
    default:   Serial.print(" (Unknown device)");            break;
  }
}

// ── Full Multi-Speed Scan via Bit-Bang ────────────────────────────
void runScan() {
  scanCount++;
  bool responded[128] = { false };

  Serial.println(F("============================================"));
  Serial.print(F("  SCAN #")); Serial.println(scanCount);
  Serial.println(F("  [bit-bang I2C — guaranteed clock control]"));
  Serial.println(F("============================================"));

  for (uint8_t s = 0; s < NUM_SPEEDS; s++) {
    bb_setFreq(CLOCK_SPEEDS[s]);

    Serial.println(F("--------------------------------------------"));
    Serial.print(F("Scanning at ")); Serial.print(CLOCK_LABELS[s]);
    Serial.print(F("  (half-period: ")); Serial.print(g_halfUs); Serial.println(F(" µs)"));

    uint8_t found = 0;
    for (byte addr = 1; addr < 127; addr++) {
      if (bb_probe(addr)) {
        Serial.print(F("  [FOUND]   0x"));
        if (addr < 0x10) Serial.print("0");
        Serial.print(addr, HEX);
        identifyDevice(addr);
        if (!responded[addr]) {
          Serial.print(F("  ← FIRST SEEN at this speed!"));
          responded[addr] = true;
        }
        Serial.println();
        found++;
      }
    }

    Serial.print(F("  → Total found: ")); Serial.println(found);
    delay(50);
  }

  // ── Per-Cycle Summary ──────────────────────────────────────────
  Serial.println(F("============================================"));
  Serial.print(F("  Summary (Scan #")); Serial.print(scanCount); Serial.println(F(")"));
  uint8_t total = 0;
  for (byte addr = 1; addr < 127; addr++) {
    if (responded[addr]) {
      Serial.print(F("  0x"));
      if (addr < 0x10) Serial.print("0");
      Serial.print(addr, HEX);
      identifyDevice(addr);
      Serial.println();
      total++;
    }
  }
  if (total == 0) Serial.println(F("  No devices found this cycle!"));
  Serial.println(F("============================================\n"));
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(47, OUTPUT);
  digitalWrite(47, HIGH);

  // Wire owns the hardware I2C peripheral — used only for MCP23017
  Wire.begin(SDA_PIN_1, SCL_PIN_1, 100000);
  if (!mcp.begin_I2C()) {
    Serial.println(F("Error initializing MCP23017."));
    while (1);
  }
  mcp.pinMode(XSHUT, OUTPUT);
  mcp.digitalWrite(XSHUT, HIGH);
  delay(10);

  // Pre-set pins as INPUT (released/high) before bit-bang takes over
  pinMode(SDA_PIN_1, INPUT);
  pinMode(SCL_PIN_1, INPUT);

  Serial.println(F("============================================"));
  Serial.println(F("  I2C Multi-Speed Scanner  [ESP32 bit-bang] "));
  Serial.print  (F("  SDA: pin ")); Serial.print(SDA_PIN_1);
  Serial.print  (F("  |  SCL: pin ")); Serial.println(SCL_PIN_1);
  Serial.println(F("============================================\n"));
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  runScan();
  delay(SCAN_INTERVAL_MS);
}