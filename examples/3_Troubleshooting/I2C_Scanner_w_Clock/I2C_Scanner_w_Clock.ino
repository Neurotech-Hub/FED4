#include <Wire.h>

// ── Pin Definitions ───────────────────────────────────────────────
#define SDA_PIN_1 8
#define SCL_PIN_1 9

// ── Scan Repeat Interval ──────────────────────────────────────────
#define SCAN_INTERVAL_MS 1000  // pause between full scan cycles

// ── Clock Speeds to Test ──────────────────────────────────────────
const uint32_t CLOCK_SPEEDS[] = { 10000, 50000, 100000, 400000 };
const char*    CLOCK_LABELS[] = { "10 kHz (slow)", "50 kHz", "100 kHz (standard)", "400 kHz (fast)" };
const uint8_t  NUM_SPEEDS     = 4;

uint32_t scanCount = 0; // tracks how many full cycles have run

// ── Device Identifier ─────────────────────────────────────────────
void identifyDevice(byte address) {
  switch (address) {
    case 0x0C: Serial.print(" (MLX90393SLW Magnetometer)");    break;
    case 0x10: Serial.print(" (VEML7700-TT LUX)");             break;
    case 0x19: Serial.print(" (LIS2DH12 Accelerometer)");      break;
    case 0x20: Serial.print(" (MCP23017 GPIO Expander)");      break;
    case 0x29: Serial.print(" (VL53L4CDV0DH/1 ToF)");          break;
    case 0x36: Serial.print(" (MAX17048 Battery Monitor)");    break;
    case 0x5A: Serial.print(" (STHS34PF80TR Motion)");         break;
    case 0x68: Serial.print(" (RTC)");                          break;
    case 0x76: Serial.print(" (BME680 Temp)");                  break;
    default:   Serial.print(" (Unknown device)");              break;
  }
}

// ── Full Multi-Speed Scan (called every loop iteration) ───────────
void runScan() {
  scanCount++;
  bool responded[128] = { false };

  Serial.println(F("============================================"));
  Serial.print(F("  SCAN #")); Serial.println(scanCount);
  Serial.println(F("============================================"));

  for (uint8_t s = 0; s < NUM_SPEEDS; s++) {
    Wire.setClock(CLOCK_SPEEDS[s]);
    delay(20); // let bus settle after clock change

    Serial.println(F("--------------------------------------------"));
    Serial.print(F("Scanning at "));
    Serial.println(CLOCK_LABELS[s]);

    uint8_t found = 0;
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      byte error = Wire.endTransmission();

      if (error == 0) {
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

      } else if (error == 4) {
        Serial.print(F("  [BUS ERR] 0x"));
        if (addr < 0x10) Serial.print("0");
        Serial.println(addr, HEX);
      }

      delay(3);
    }

    Serial.print(F("  → Total found: "));
    Serial.println(found);
  }

  // ── Per-Cycle Summary ──
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
  Serial.println(F("============================================"));
  Serial.print(F("  Next scan in "));
  Serial.print(SCAN_INTERVAL_MS / 1000);
  Serial.println(F("s ..."));
  Serial.println();
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(1000);

  Wire.begin(SDA_PIN_1, SCL_PIN_1); // ESP32: dynamic pin assignment

  Serial.println(F("============================================"));
  Serial.println(F("    I2C Multi-Speed Scanner  [ESP32]        "));
  Serial.print  (F("    SDA: pin ")); Serial.print(SDA_PIN_1);
  Serial.print  (F("  |  SCL: pin ")); Serial.println(SCL_PIN_1);
  Serial.println(F("============================================"));
  Serial.println();
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  runScan();
  delay(SCAN_INTERVAL_MS);
}