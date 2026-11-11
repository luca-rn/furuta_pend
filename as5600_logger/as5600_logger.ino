// AS5600 swing logger
// Streams "micros,raw" over serial at a fixed rate, for analysis on the PC.
// Raw is the 12-bit angle (0..4095 = 0..360 deg).

#include <Wire.h>

// ---- Settings ---------------------------------------------------------------
const int SDA_PIN = 21;            // ESP32 default I2C pins; change if yours differ
const int SCL_PIN = 22;
const uint32_t PERIOD_US = 2000;   // 2000 us = 500 Hz
const uint8_t AS5600_ADDR = 0x36;

// ---- AS5600 -----------------------------------------------------------------
// Reads a 2-byte register. Returns -1 on an I2C error.
int readReg16(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return -1;
  uint8_t hi = Wire.read();  // separate reads: C++ doesn't fix the evaluation
  uint8_t lo = Wire.read();  // order of two calls inside one expression
  return ((hi << 8) | lo) & 0x0FFF;
}

int readRawAngle() { return readReg16(0x0C); }  // RAW ANGLE register

// STATUS register bits: MD = magnet detected, ML = too weak, MH = too strong
void printMagnetStatus() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) {
    Serial.println("# ERROR: AS5600 not responding. Check wiring and address.");
    return;
  }
  uint8_t s = Wire.read();
  Serial.print("# magnet detected: "); Serial.println((s & 0x20) ? "yes" : "NO");
  if (s & 0x10) Serial.println("# WARNING: magnet too weak (too far away)");
  if (s & 0x08) Serial.println("# WARNING: magnet too strong (too close)");
}

// ---- Main -------------------------------------------------------------------
uint32_t nextSample;

void setup() {
  Serial.begin(921600);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // fast-mode I2C
  delay(100);
  Serial.println("# AS5600 logger: micros,raw");
  printMagnetStatus();
  nextSample = micros();
}

void loop() {
  // Fixed-rate timing: schedule from the previous target, not from "now",
  // so small delays don't accumulate into drift.
  if ((int32_t)(micros() - nextSample) >= 0) {
    nextSample += PERIOD_US;
    uint32_t t = micros();
    int raw = readRawAngle();
    if (raw < 0) {
      Serial.println("# i2c error");
      return;
    }
    Serial.print(t);
    Serial.print(',');
    Serial.println(raw);
  }
}
