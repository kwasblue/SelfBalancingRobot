#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>  // Include OLED library
#include <VlxLidar.h>


// I2C and MPU6050 variables
const int MPU = 0x68;              // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float gyroAngleX, gyroAngleY, gyroanglex, GyroX_last, yaw;
float dt, tt, tt_last;
float mqttGyroX, mqttGyroY, mqttGyroZ; // For MQTT task
float loopFrequency = 0;
// Motor control variables
int in1 = 19;
int in2 = 18;
int in3 = 4;
int in4 = 2;
int duty_cycle = 0;
float error = 0;
float max_PWM_angle = 45;
float min_PWM = 50;
float Kp = 1.0;    // Initial value for Kp
float Kp0 = (255-min_PWM)/max_PWM_angle; // normalization value for max PWM at 45 degrees
float Kd = 0;     // Initial value for Kd
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
const int i2cTaskDelayMs = 0.1;           // I2C task delay in milliseconds
const int mqttPublishIntervalMs = 10;     // MQTT data publishing interval (1 second)

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// MQTT client setup
WiFiClient espClient;
PubSubClient client(espClient);

// Task handles
TaskHandle_t MQTTTask;

// Structure to store gains and a flag for new updates
struct BalancingGains {
  float Kp;
  float Kd;
  bool newGainsAvailable;
};

BalancingGains gains = {20.0, 0.0, false};  // Initialize gains with Kp = 20 and Kd = 0
SemaphoreHandle_t gainsMutex;  // Mutex to protect shared data

// Gyro range options (in degrees per second)
#define MPU6050_RANGE_250_DEG  0b00  // ±250 dps
#define MPU6050_RANGE_500_DEG  0b01  // ±500 dps
#define MPU6050_RANGE_1000_DEG 0b10  // ±1000 dps
#define MPU6050_RANGE_2000_DEG 0b11  // ±2000 dps

// Accelerometer range options (in g)
#define MPU6050_RANGE_2_G  0b00  // ±2g
#define MPU6050_RANGE_4_G  0b01  // ±4g
#define MPU6050_RANGE_8_G  0b10  // ±8g
#define MPU6050_RANGE_16_G 0b11  // ±16g

// DLPF bandwidth options (in Hz)
#define MPU6050_BAND_260_HZ 0b000  // 260 Hz
#define MPU6050_BAND_184_HZ 0b001  // 184 Hz
#define MPU6050_BAND_94_HZ  0b010  // 94 Hz
#define MPU6050_BAND_44_HZ  0b011  // 44 Hz
#define MPU6050_BAND_21_HZ  0b100  // 21 Hz
#define MPU6050_BAND_10_HZ  0b101  // 10 Hz
#define MPU6050_BAND_5_HZ   0b110  // 5 Hz

// --------- Kwasi did this stuff -----------------//

#define XSHUT_PIN 3  // set pins for the lidar xshut here
#define IRQ_PIN 1 // irq pin here 

Lidar lidarsensor(XSHUT_PIN, IRQ_PIN)  // initialize the lidar class

//--------------------------------------------------------------//

// Function prototypes
void setGyroRange(uint8_t range);
void setAccelRange(uint8_t range);
void setDLPF(uint8_t bandwidth);
void setup_wifi();
void reconnect();
void MQTTTaskCode(void *pvParameters);
void i2cTask(void *pvParameters);
void motor1(int duty_cycle);
void motor2(int duty_cycle);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void updateOLEDDisplay();

// Function to set the gyroscope range
void setGyroRange(uint8_t range) {
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(range << 3);  // Shift the range value into the correct position
  Wire.endTransmission(true);
}

// Function to set the accelerometer range
void setAccelRange(uint8_t range) {
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(range << 3);  // Shift the range value into the correct position
  Wire.endTransmission(true);
}

// Function to set the digital low-pass filter bandwidth
void setDLPF(uint8_t bandwidth) {
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(bandwidth);  // Set the DLPF bandwidth
  Wire.endTransmission(true);
}

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

      // Show the current gains after connecting to MQTT
      updateOLEDDisplay();  // Ensure OLED is updated with gains
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

// Motor control functions
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

// Function to update OLED display with gains
void updateOLEDDisplay() {
  if (xSemaphoreTake(gainsMutex, portMAX_DELAY)) {
    // Print to Serial for debugging
    Serial.print("Updating OLED Display - Kp: ");
    Serial.print(gains.Kp, 2);
    Serial.print(", Kd: ");
    Serial.println(gains.Kd, 2);

    display.clearDisplay();  // Clear the OLED screen
    display.display();
    
    // Display the current gains (Kp and Kd)
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("GAINS"); // Set cursor to start position for Kp
    display.print("Kp: ");
    display.println(gains.Kp, 2);  // Display Kp with 2 decimal places

       // Move to the next line for Kd
    display.print("Kd: ");
    
    display.print(gains.Kd, 2);  // Display Kd with 2 decimal places
    
    display.display();  // Push everything to the OLED screen
    
    xSemaphoreGive(gainsMutex);
  } else {
    Serial.println("Failed to take mutex for OLED update.");
  }
}

// MQTT callback function to process incoming messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, "ESP32/gains") == 0) {
    char payloadString[100];
    strncpy(payloadString, (char*)payload, length);
    payloadString[length] = '\0';  // Null-terminate the string

    char* start = strstr(payloadString, "[");
    char* end = strstr(payloadString, "]");

    if (start != NULL && end != NULL) {
      start++;
      float newKp, newKd;
      if (sscanf(start, "%f,%f", &newKp, &newKd) == 2) {
        // Update Kp and Kd using a mutex to prevent race conditions
        if (xSemaphoreTake(gainsMutex, portMAX_DELAY)) {
          gains.Kp = newKp;
          gains.Kd = newKd;
          gains.newGainsAvailable = true;  // Indicate new gains are ready to be displayed
          xSemaphoreGive(gainsMutex);
        }

        // Update OLED display when new gains are received
        updateOLEDDisplay();

        Serial.print("Updated Kp: ");
        Serial.print(newKp);
        Serial.print(" | Kd: ");
        Serial.println(newKd);
      }
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

// Task for reading IMU data and controlling motors, pinned to core 1
// Global variable to store frequency
void i2cTask(void *pvParameters) {
  while (true) {
    tt = millis();
    dt = tt - tt_last;

    // === Read gyroscope data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawGyroX = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawGyroY = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawGyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

    GyroX = rawGyroX / 131.0;
    GyroY = rawGyroY / 131.0;
    GyroZ = rawGyroZ / 131.0;

    // Calculate roll (angle about x-axis) from accelerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Accelerometer data first register address 0x3B
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawAccX = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawAccY = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawAccZ = (int16_t)(Wire.read() << 8 | Wire.read());

    float AccX = rawAccX / 16384.0;
    float AccY = rawAccY / 16384.0;
    float AccZ = rawAccZ / 16384.0;

    gyroanglex = ((GyroX - GyroX_last) * (float)dt / 1000) * (180 / PI);
    roll = atan2(AccY, AccZ) + PI;
    roll = roll * (180 / PI);

    // Angle estimation using complementary filter
    anglex = 0.02 * (anglex - gyroanglex) + 0.98 * roll;

    // Control logic
    error = 180 - anglex;
    duty_cycle = 40 + (Kp*Kp0 * error) + (Kd * (-GyroX));

    // Motor control
    motor1(duty_cycle);
    motor2(duty_cycle);

    // Update shared values for MQTT
    mqttGyroX = GyroX;
    mqttGyroY = GyroY;
    mqttGyroZ = GyroZ;

    // Calculate and print loop frequency
    
    loopFrequency = 1000.0 / dt;  // Convert period (ms) to frequency (Hz)
    Serial.print("Loop frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");
    

    tt_last = tt;
    GyroX_last = GyroX;
    // ------- Kwasi did this------------------------//
    // This is the part of the loop for the lidat should output 
    int16_t distance = lidarSensor.getDistance();
    if (distance != -1) {
      Serial.print(F("Distance: "));
      Serial.print(distance);
      Serial.println(F(" mm"));
    // ------------Kwasi------------------------------------------//
    //vTaskDelay(i2cTaskDelayMs);  // Yield to other tasks
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

  // More kwasi things that might be usless hopfully it doesnt break if it does comment out

   // Initialize the LIDAR sensor Kwasi ---------------------------//
  if (!lidarSensor.begin()) {
    //Serial.println(F("Failed to initialize sensor!"));
    while (1) delay(10);

  // Start ranging
  if (!lidarSensor.startRanging()) {
    //Serial.println(F("Failed to start ranging!"));
    while (1) delay(10);
   
   lidarSensor.setTimingBudget(50);  // Set timing budget to 50ms
  // Kwasi------------------------------------------------------------//

  // Initialize WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);  // Set MQTT callback

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);  // Loop forever if the display fails to initialize
  }
  display.clearDisplay();  // Clear the display buffer
  display.display();
  display.setTextColor(WHITE);
  // Initialize the mutex
  gainsMutex = xSemaphoreCreateMutex();

  // Display the initial gains on the OLED at startup
  updateOLEDDisplay();

  // Start the MQTT task with lower priority and pin it to core 0
  xTaskCreatePinnedToCore(MQTTTaskCode, "MQTT Task", 10000, NULL, 1, &MQTTTask, 0);

  // Start the I2C communication (main loop) task on core 1
  xTaskCreatePinnedToCore(i2cTask, "I2C Task", 10000, NULL, 2, NULL, 1);

  // Initialize the MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission(true);

  // Configure Gyro Range
  setGyroRange(MPU6050_RANGE_1000_DEG);  // Set to ±1000 dps (default)

  // Configure Accelerometer Range
  setAccelRange(MPU6050_RANGE_2_G);  // Set to ±2g (default)

  // Configure Digital Low-Pass Filter
  setDLPF(MPU6050_BAND_94_HZ);  // Set to 94 Hz bandwidth (default)

  Serial.println("MPU6050 configured.");
}

void loop() {
  // Empty because i2cTask is running on core 1
}
