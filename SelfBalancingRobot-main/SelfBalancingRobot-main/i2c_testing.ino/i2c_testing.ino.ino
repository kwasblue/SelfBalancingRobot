#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup()
{
  // Start serial communication
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin(21, 22);  // SDA, SCL
  Wire.setClock(200); // Set I2C to 400 kHz for VL53L1X

  // Initialize the sensor
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1); // Stop if sensor is not detected
  }

  // Set sensor to long distance mode with a 50ms timing budget
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings with a 50ms interval
  sensor.startContinuous(50);
}

void loop()
{
  // Read sensor data
  sensor.read();

  // Print the distance and status
  Serial.print("Range (mm): ");
  Serial.print(sensor.ranging_data.range_mm);
  Serial.print("\tStatus: ");
  Serial.print(sensor.rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print("\tPeak signal (MCPS): ");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tAmbient (MCPS): ");
  Serial.println(sensor.ranging_data.ambient_count_rate_MCPS);

  delay(100); // Delay before the next reading
}