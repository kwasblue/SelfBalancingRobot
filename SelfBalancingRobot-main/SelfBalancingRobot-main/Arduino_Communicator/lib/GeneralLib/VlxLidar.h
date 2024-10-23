// Lidar.h
#ifndef VLXLIDAR_H
#define VLXLIDAR_H

#include "Adafruit_VL53L1X.h"
#include <Wire.h>

class Lidar {
  private:
    Adafruit_VL53L1X sensor;  // Sensor object
    uint8_t xshut_pin;        // XSHUT pin for the LIDAR
    uint8_t irq_pin;          // IRQ pin for the LIDAR

  public:
    // Constructor
    Lidar(uint8_t xshut, uint8_t irq);

    // Initialize the LIDAR sensor
    bool begin();

    // Start ranging
    bool startRanging();

    // Set timing budget
    void setTimingBudget(uint16_t timingBudget);

    // Get distance
    int16_t getDistance();
};

#endif  // LIDAR_H
