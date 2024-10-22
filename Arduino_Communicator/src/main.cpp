#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>  // Include the MQTT library

// I2C and MPU6050 variables
const int MPU = 0x68;              // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float gyroAngleX, gyroAngleY, yaw;
float elapsedTime, currentTime, previousTime;
float mqttGyroX, mqttGyroY, mqttGyroZ; // For MQTT task

// Motor control variables
int in1 = 19;
int in2 = 18;
int in3 = 4;
int in4 = 2;
int duty_cycle = 0;
float error = 0;
float Kp = 20;
float Kd = 10;
float anglex = 0;
float roll = 0;  // Rotation about x-axis

// Timing variables
unsigned long lastMQTTTransmission = 0;

// Wi-Fi and MQTT credentials and settings
const char* ssid = "Tufts_Robot";        // Wi-Fi SSID
const char* password = "";               // Wi-Fi Password
const char* mqtt_server = "10.243.82.33"; // MQTT Broker IP Address
const int mqtt_port = 1883;               // MQTT Broker Port

// Task settings
const int mqttTaskDelayMs = 20;           // MQTT task delay in milliseconds (50Hz)
const int i2cTaskDelayMs = 1;             // I2C task delay in milliseconds
const int mqttPublishIntervalMs = 10;   // MQTT data publishing interval (1 second)

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

      // Subscribe to the topic for receiving gains updates
      client.subscribe("ESP32/gains");
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

// Motor control functions (declared before they are used)
void motor1(int duty_cycle) {
  if (duty_cycle < 0) {
    analogWrite(in1, abs(duty_cycle));
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, abs(duty_cycle));
  }
}

void motor2(int duty_cycle) {
  if (duty_cycle < 0) {
    analogWrite(in3, abs(duty_cycle));
    analogWrite(in4, 0);
  } else {
    analogWrite(in3, 0);
    analogWrite(in4, abs(duty_cycle));
  }
}

// MQTT callback function to process incoming messages without JSON library
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Ensure the topic is "ESP32/gains"
  if (strcmp(topic, "ESP32/gains") == 0) {
    // Convert the payload to a string
    char payloadString[100];
    strncpy(payloadString, (char*)payload, length);
    payloadString[length] = '\0';  // Null-terminate the string

    // Look for the "balancing" key and extract the Kp and Kd values
    char* start = strstr(payloadString, "[");
    char* end = strstr(payloadString, "]");

    if (start != NULL && end != NULL) {
      // Move start pointer after the '[' character
      start++;

      // Extract the values of Kp and Kd
      float newKp, newKd;
      if (sscanf(start, "%f,%f", &newKp, &newKd) == 2) {
        // Update Kp and Kd
        Kp = newKp;
        Kd = newKd;

        // Print the new values of Kp and Kd
        Serial.print("Updated Kp: ");
        Serial.print(Kp);
        Serial.print(" | Kd: ");
        Serial.println(Kd);
      } else {
        Serial.println("Failed to parse Kp and Kd from payload.");
      }
    } else {
      Serial.println("Invalid JSON format.");
    }
  }
}

// Task to send data via MQTT and process incoming messages
void MQTTTaskCode(void *pvParameters) {
  unsigned long lastPublishTime = 0;  // Time of last data publish

  while (true) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();  // Handle incoming MQTT messages

    unsigned long now = millis();

    // Publish data periodically
    if (now - lastPublishTime > mqttPublishIntervalMs) {
      lastPublishTime = now;

      // Create a JSON payload with angle estimate, gyro data, and PWM
      char msg[200];
      snprintf(msg, 200,
        "{\"anglex\": %.2f, \"gyroX\": %.2f, \"gyroY\": %.2f, \"gyroZ\": %.2f, \"PWM\": %d}",
        anglex, mqttGyroX, mqttGyroY, mqttGyroZ, duty_cycle);

      // Publish the message to the "ESP32/data" topic
      client.publish("ESP32/data", msg);
    }

    vTaskDelay(1);  // Yield to other tasks
  }
}

// Task for reading IMU data and controlling motors, pinned to core 1 (unchanged)
void i2cTask(void *pvParameters) {
  while (true) {
    // Calculate elapsed time for sensor integration
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0;

    // === Read gyroscope data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Apply calibration offsets to gyro data
    GyroX += 0.56;
    GyroY -= 2;
    GyroZ += 0.79;

    // Calculate roll (angle about x-axis) from accelerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Accelerometer data first register address 0x3B
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    roll = atan2(AccY, AccZ) * (180 / PI);

    // Angle estimation using complementary filter
    anglex = 0.02 * (anglex - GyroX * elapsedTime) + 0.98 * roll;

    // Control logic
    error = 180 - anglex;
    duty_cycle = 50 + (Kp * error) + (Kd * (-GyroX));

    // // Safety cutoff for duty cycle
    // if (anglex > 270 || anglex < 90) {
    //   duty_cycle = 0;
    // }

    // Motor control
    motor1(duty_cycle);
    motor2(duty_cycle);

    // Update shared values for MQTT
    mqttGyroX = GyroX;
    mqttGyroY = GyroY;
    mqttGyroZ = GyroZ;

    vTaskDelay(i2cTaskDelayMs);  // Yield to other tasks
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400 kHz for faster communication

  // Initialize motor control pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);  // Set MQTT callback

  // Start the MQTT task with lower priority and pin it to core 0
  xTaskCreatePinnedToCore(MQTTTaskCode, "MQTT Task", 10000, NULL, 1, &MQTTTask, 0);

  // Start the I2C communication (main loop) task on core 1
  xTaskCreatePinnedToCore(i2cTask, "I2C Task", 10000, NULL, 2, NULL, 1);

  // Initialize the MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission(true);
}

void loop() {
  // Empty because i2cTask is running on core 1
}