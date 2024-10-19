#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h> // Include the MQTT library

// I2C and MPU6050 variables
const int MPU = 0x68;              // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float gyroAngleX, gyroAngleY, yaw;
float elapsedTime, currentTime, previousTime;
float mqttGyroX, mqttGyroY, mqttGyroZ; // For MQTT task

// Timing and loop frequency variables
unsigned long lastMQTTTransmission = 0;
unsigned long loopStartTime, loopEndTime;
float loopFrequency;

// Wi-Fi and MQTT credentials and settings
const char* ssid = "hoe5andMicha3l";       // Wi-Fi SSID
const char* password = "DashiLover69";     // Wi-Fi Password
const char* mqtt_server = "10.0.0.25";     // MQTT Broker IP Address
const int mqtt_port = 1883;                // MQTT Broker Port (default is 1883)

// Task settings
const int mqttTaskDelayMs = 20;            // MQTT task delay in milliseconds (50Hz)
const int i2cTaskDelayMs = 1;              // I2C task delay in milliseconds

// MQTT client setup
WiFiClient espClient;
PubSubClient client(espClient);

// Task handles
TaskHandle_t MQTTTask;

// Function to set up Wi-Fi connection
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// Function to reconnect to MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Gyro")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

// Task to send data via MQTT
void MQTTTaskCode(void *pvParameters) {
  while (true) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    unsigned long now = millis();
    // Transmit data at 50 Hz (every 20ms)
    if (now - lastMQTTTransmission > mqttTaskDelayMs) {
      lastMQTTTransmission = now;

      // Prepare the data in a format suitable for transmission (CSV format in this case)
      char msg[100];
      snprintf(msg, 100, "GyroX: %.2f, GyroY: %.2f, GyroZ: %.2f", mqttGyroX, mqttGyroY, mqttGyroZ);

      // Publish the message to a topic
      client.publish("gyro/data", msg);
    }
    vTaskDelay(1); // Yield to other tasks
  }
}

// Task for reading I2C data, pinned to core 1
void i2cTask(void *pvParameters) {
  while (true) {
    loopStartTime = micros(); // Capture the start time of the loop

    // Calculate elapsed time for sensor integration
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0;

    // === Read gyroscope data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Apply calibration offsets to gyro data
    GyroX += 0.56;
    GyroY -= 2;
    GyroZ += 0.79;

    // Integrate gyro angles over time
    gyroAngleX += GyroX * elapsedTime;
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;

    // Update the shared gyro values for MQTT transmission
    mqttGyroX = GyroX;
    mqttGyroY = GyroY;
    mqttGyroZ = GyroZ;

    // Calculate and print the loop frequency
    loopEndTime = micros(); // Capture the end time of the loop
    loopFrequency = 1000000.0 / (loopEndTime - loopStartTime); // Calculate loop frequency in Hz
    Serial.print("Loop Frequency: ");
    Serial.println(loopFrequency);

    vTaskDelay(i2cTaskDelayMs); // Yield to other tasks
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400 kHz for faster communication

  // Initialize WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  // Start the MQTT task with lower priority and pin it to core 0
  xTaskCreatePinnedToCore(MQTTTaskCode, "MQTT Task", 10000, NULL, 1, &MQTTTask, 0);

  // Start the I2C communication (main loop) task on core 1
  xTaskCreatePinnedToCore(i2cTask, "I2C Task", 10000, NULL, 2, NULL, 1);

  // Initialize the MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission(true);
}

void loop() {
  // Empty because i2cTask is running on core 1
}