#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_LIS3DH accel = Adafruit_LIS3DH();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("LIS3DH Test on Custom I2C Pins");

  // Configure I2C to use custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize LIS3DH with I2C address 0x19 (default for LIS3DH)
  if (!accel.begin(0x19)) {
    Serial.println("Problem starting LIS3DH on custom pins.");
    Serial.println("Check wiring and I2C address!");
    while (1);
  }
  
  // Set data rate and range
  accel.setDataRate(LIS3DH_DATARATE_50_HZ);
  accel.setRange(LIS3DH_RANGE_2_G);
  
  Serial.println("LIS3DH found!");
}

void loop() {
  // Read acceleration values
  accel.read();
  
  // Get sensor event with normalized values in m/s^2
  sensors_event_t event;
  accel.getEvent(&event);
  
  // Convert from m/s^2 to g (1g = 9.80665 m/s^2)
  float accelX = event.acceleration.x / 9.80665;
  float accelY = event.acceleration.y / 9.80665;
  float accelZ = event.acceleration.z / 9.80665;

  Serial.print("X: ");
  Serial.print(accelX, 3);
  Serial.print(" g, Y: ");
  Serial.print(accelY, 3);
  Serial.print(" g, Z: ");
  Serial.print(accelZ, 3);
  Serial.println(" g");

  delay(500);
}
