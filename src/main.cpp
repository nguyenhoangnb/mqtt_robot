#include <WiFi.h>
#include <PubSubClient.h>

// Replace these with your network credentials
const char* ssid = "NQT";
const char* password = "11345678";

// MQTT broker information
const char* mqtt_server = "192.168.1.114"; // Broker IP

WiFiClient espClient;
PubSubClient client(espClient);

#define home_fan 25
#define home_light 33

// Timer variables
bool lightOn = false;
bool fanOn = false;
bool stop = false;
unsigned long previousLightMillis = 0;
unsigned long previousFanMillis = 0;
const long lightInterval = 10000; // Light ON duration in milliseconds
const long fanInterval = 15000;   // Fan ON duration in milliseconds

unsigned long statusPublishInterval = 5000; // Interval to publish status updates
unsigned long previousStatusPublishMillis = 0;

// Function prototypes
void setup_wifi();
void reconnect();
void callback(char* topic, byte* message, unsigned int length);
void publishStatus();

void setup() {
  Serial.begin(115200);
  setup_wifi();
  pinMode(home_fan, OUTPUT);
  pinMode(home_light, OUTPUT);
  client.setServer(mqtt_server, 1883);    // Set the MQTT server and port
  client.setCallback(callback);           // Set the callback function
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(messageTemp);

  // Control light based on MQTT message
  // if (String(topic)=="home/stop"){
  //   stop = true;
  //   digitalWrite(home_fan, LOW);
  //   digitalWrite(home_light, LOW);
  //   lightOn = false;
  //   fanOn = false;
  // }
  // while 
  if (String(topic) == "home/light") {
    if (messageTemp == "ON") {
      Serial.println("Turning the light ON");
      digitalWrite(home_light, HIGH);
      lightOn = true;
      previousLightMillis = millis(); // Start timing for the light
    } else if (messageTemp == "OFF") {
      Serial.println("Turning the light OFF");
      digitalWrite(home_light, LOW);
      lightOn = false; // Stop the light timer
    }
  }
  // Control fan based on MQTT message
  else if (String(topic) == "home/fan") {
    if (messageTemp == "ON") {
      Serial.println("Turning the fan ON");
      digitalWrite(home_fan, HIGH);
      fanOn = true;
      previousFanMillis = millis(); // Start timing for the fan
    } else if (messageTemp == "OFF") {
      Serial.println("Turning the fan OFF");
      digitalWrite(home_fan, LOW);
      fanOn = false; // Stop the fan timer
    }
  }

  // Publish updated status after a change
  publishStatus();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("home/light"); // Subscribe to light control topic
      client.subscribe("home/fan");   // Subscribe to fan control topic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishStatus() {
  if (client.connected()) {
    client.publish("home/light/status", lightOn ? "ON" : "OFF");
    client.publish("home/fan/status", fanOn ? "ON" : "OFF");
    Serial.println("Published current status of light and fan");
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  // Check if light has been on for the specified interval
  if (lightOn && (currentMillis - previousLightMillis >= lightInterval)) {
    Serial.println("Turning the light OFF after 10 seconds");
    digitalWrite(home_light, LOW);
    lightOn = false;
    publishStatus(); // Publish updated status
  }

  // Check if fan has been on for the specified interval
  if (fanOn && (currentMillis - previousFanMillis >= fanInterval)) {
    Serial.println("Turning the fan OFF after 15 seconds");
    digitalWrite(home_fan, LOW);
    fanOn = false;
    publishStatus(); // Publish updated status
  }

  // Periodically publish the status
  if (currentMillis - previousStatusPublishMillis >= statusPublishInterval) {
    publishStatus();
    previousStatusPublishMillis = currentMillis;
  }
}
