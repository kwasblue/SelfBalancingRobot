#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>

// Wi-Fi and MQTT settings
const char* ssid = "Tufts_Robot";  // Replace with your Wi-Fi SSID
const char* password = "";  // Replace with your Wi-Fi password
const char* mqtt_server = "broker.hivemq.com";  // Public broker

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

WiFiClient espClient;
PubSubClient client(espClient);

String mqtt_message = "";  // Stores the last received MQTT message
String current_gains = "gains:";  // Initialize current gains

// Wi-Fi symbol bitmap
#define WIFI_SYMBOL_WIDTH  16
#define WIFI_SYMBOL_HEIGHT 16

static const unsigned char PROGMEM wifi2_icon16x16[] = {
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000111, 0b11100000, //      ######     
  0b00001111, 0b11110000, //     ########    
  0b00011000, 0b00011000, //    ##      ##   
  0b00000011, 0b11000000, //       ####      
  0b00000111, 0b11100000, //      ######     
  0b00000100, 0b00100000, //      #    #     
  0b00000001, 0b10000000, //        ##       
  0b00000001, 0b10000000, //        ##       
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
};

// Function to update the OLED display
void updateDisplay(bool wifi_connected) {
  display.clearDisplay();

  // Display Wi-Fi status
  if (wifi_connected) {
    display.drawBitmap(SCREEN_WIDTH - WIFI_SYMBOL_WIDTH, 0, wifi2_icon16x16, WIFI_SYMBOL_WIDTH, WIFI_SYMBOL_HEIGHT, WHITE);
    display.setCursor(0, 50);
    display.print("WiFi Connected");
  } else {
    display.setCursor(0, 50);
    display.print("WiFi Disconnected");
  }

  // Display the current gains value
  display.setCursor(0, 20);
  display.print(current_gains);

  display.display();
}

void setup_wifi() {
  delay(10);
  updateDisplay(false);  // Show "Connecting to Wi-Fi..."

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    updateDisplay(false);  // Show "Connecting..."
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");

  // Show Wi-Fi connected and update the display
  updateDisplay(true);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  mqtt_message = "";  // Clear previous message
  for (int i = 0; i < length; i++) {
    mqtt_message += (char)payload[i];  // Append each byte to string
  }

  Serial.println(mqtt_message);

  // Update the current gains variable with the new message
  current_gains = "gains: " + mqtt_message;

  // Update the display to show the new gains and Wi-Fi status
  updateDisplay(WiFi.status() == WL_CONNECTED);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    updateDisplay(WiFi.status() == WL_CONNECTED);  // Keep showing Wi-Fi status
    
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      updateDisplay(WiFi.status() == WL_CONNECTED);

      // Once connected, publish an announcement...
      client.publish("esp32/test", "hello world");
      // ... and resubscribe to the topic
      client.subscribe("esp32/gains/follow");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      updateDisplay(WiFi.status() == WL_CONNECTED);  // Show Wi-Fi status during retry
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Setup OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  // Connect to Wi-Fi
  setup_wifi();

  // Initialize MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  // Check Wi-Fi connection status and update display
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();  // Attempt to reconnect Wi-Fi
  }

  // MQTT handling
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Continuously display the Wi-Fi status and current gains
  updateDisplay(WiFi.status() == WL_CONNECTED);
  delay(100)
}