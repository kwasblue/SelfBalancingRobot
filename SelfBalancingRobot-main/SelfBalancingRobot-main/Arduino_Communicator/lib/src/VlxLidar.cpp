// Lidar.cpp
#include "VlxLidar.h"

// Constructor: Initialize with XSHUT and IRQ pins
Lidar::Lidar(uint8_t xshut, uint8_t irq) : xshut_pin(xshut), irq_pin(irq), sensor(Adafruit_VL53L1X(xshut_pin, irq_pin)) {}

// Initialize the LIDAR sensor
bool Lidar::begin() {
  if (!sensor.begin(0x29, &Wire)) {
    Serial.print(F("Error initializing VL53L1X sensor: "));
    Serial.println(sensor.vl_status);
    return false;
  }
  Serial.println(F("VL53L1X sensor initialized successfully!"));
  return true;
}

// Start ranging
bool Lidar::startRanging() {
  if (!sensor.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(sensor.vl_status);
    return false;
  }
  Serial.println(F("Ranging started"));
  return true;
}

// Set timing budget for ranging
void Lidar::setTimingBudget(uint16_t timingBudget) {
  sensor.setTimingBudget(timingBudget);
  Serial.print(F("Timing budget set to: "));
  Serial.println(sensor.getTimingBudget());
}

// Get the distance measurement
int16_t Lidar::getDistance() {
  if (sensor.dataReady()) {
    int16_t distance = sensor.distance();
    if (distance == -1) {
      Serial.print(F("Couldn't get distance: "));
      Serial.println(sensor.vl_status);
      return -1;
    }
    sensor.clearInterrupt();  // Clear the interrupt for new readings
    return distance;
  }
  return -1;  // Data not ready
}
