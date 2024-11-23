// #include <WiFi.h>
// #include <PubSubClient.h>

// // Replace these with your network credentials
// const char* ssid = "NQT";
// const char* password = "11345678";

// // MQTT broker information
// const char* mqtt_server = "192.168.1.114"; // Broker IP

// WiFiClient espClient;
// PubSubClient client(espClient);

// #define home_fan 25
// #define home_light 33
// #define MAX_COMMAND 100
// struct command_stack{
//     int head;
//     int tail;
//     int pos;
//     char commands[MAX_COMMAND];
//     unsigned long times[MAX_COMMAND];
//     command_stack():head(0), tail(0){};
//     void push(char command, unsigned int time){
//         if (isFull()){
//             Serial.println("Stack is full. Cannot push command.");
//             return ;
//         }
//         commands[tail] = command;
//         times[tail] = time;
//         tail = (tail+1)%MAX_COMMAND;
//     }
//     void pop(){
//         if (isEmpty()){
//             Serial.println("Stack is empty. Cannot pop command");
//             return; 
//         }
//         head = (head + 1) % MAX_COMMAND;
//     }
//     bool isEmpty(){
//         return head == tail;
//     };
//     bool isFull(){
//         return (tail + 1) % MAX_COMMAND == head;
//     }
// };

// // Timer variables
// bool lightOn = false;
// bool fanOn = false;
// bool stop = false;
// bool topic_status = false;
// unsigned long previousLightMillis = 0;
// unsigned long previousFanMillis = 0;
// const long lightInterval = 10000; // Light ON duration in milliseconds
// const long fanInterval = 15000;   // Fan ON duration in milliseconds



// unsigned long statusPublishInterval = 5000; // Interval to publish status updates
// unsigned long previousStatusPublishMillis = 0;

// // Function prototypes
// void setup_wifi();
// void reconnect();
// void callback(char* topic, byte* message, unsigned int length);
// void publishStatus();

// void setup() {
//   Serial.begin(115200);
//   setup_wifi();
//   pinMode(home_fan, OUTPUT);
//   pinMode(home_light, OUTPUT);
//   client.setServer(mqtt_server, 1883);    // Set the MQTT server and port
//   client.setCallback(callback);           // Set the callback function
// }

// void setup_wifi() {
//   delay(10);
//   Serial.println();
//   Serial.print("Connecting to ");
//   Serial.println(ssid);

//   WiFi.begin(ssid, password);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }

//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
// }

// void callback(char* topic, byte* message, unsigned int length) {
//   String messageTemp;
  
//   for (int i = 0; i < length; i++) {
//     messageTemp += (char)message[i];
//   }

//   Serial.print("Message arrived on topic: ");
//   Serial.print(topic);
//   Serial.print(". Message: ");
//   Serial.println(messageTemp);

//   // Control stop and resume based on MQTT message
//   // if (String(topic) == "home/stop") {
//   //   stop = true;
//   //   Serial.println("System stopped: All devices turned off.");
//   //   digitalWrite(home_fan, LOW);
//   //   digitalWrite(home_light, LOW);
//   //   lightOn = false;
//   //   fanOn = false;
//   //   publishStatus();
//   // } else if (String(topic) == "home/resume") {
//   //   stop = false;
//   //   Serial.println("System resumed: Devices can now be controlled individually.");
//   //   publishStatus();
//   // }

//   // Control light based on MQTT message
//   if (String(topic)){
//     topic_status = true;
//   } else topic_status = false;
//   if (String(topic) == "home/light") {
//     if (messageTemp == "ON") {
//       Serial.println("Turning the light ON");
//       digitalWrite(home_light, HIGH);
//       lightOn = true;
//       previousLightMillis = millis(); // Start timing for the light
//     } else if (messageTemp == "OFF") {
//       Serial.println("Turning the light OFF");
//       digitalWrite(home_light, LOW);
//       lightOn = false; // Stop the light timer
//     }
//   }

//   // Control fan based on MQTT message
//   if ( String(topic) == "home/fan") {
//     if (messageTemp == "ON") {
//       Serial.println("Turning the fan ON");
//       digitalWrite(home_fan, HIGH);
//       fanOn = true;
//       previousFanMillis = millis(); // Start timing for the fan
//     } else if (messageTemp == "OFF") {
//       Serial.println("Turning the fan OFF");
//       digitalWrite(home_fan, LOW);
//       fanOn = false; // Stop the fan timer
//     }
//   }

//   // Publish updated status after a change
//   publishStatus();
// }

// void reconnect() {
//   while (!client.connected()) {
//     Serial.print("Attempting MQTT connection...");
//     if (client.connect("ESP32Client")) {
//       Serial.println("connected");
//       client.subscribe("home/light"); // Subscribe to light control topic
//       client.subscribe("home/fan");   // Subscribe to fan control topic
//       client.subscribe("home/stop");  // Subscribe to stop control topic
//       client.subscribe("home/resume");// Subscribe to resume control topic
//     } else {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       delay(5000);
//     }
//   }
// }

// void publishStatus() {
//   if (client.connected()) {
//     client.publish("home/light/status", lightOn ? "ON" : "OFF");
//     client.publish("home/fan/status", fanOn ? "ON" : "OFF");
//     client.publish("home/system/status", stop ? "STOPPED" : "RESUMED");
//     Serial.println("Published current status of light, fan, and system mode");
//   }
// }

// void loop() {
//   if (!client.connected()) {
//     reconnect();
//   }
//   client.loop();

//   unsigned long currentMillis = millis();

//   // Check if light has been on for the specified interval
//   if (lightOn && (currentMillis - previousLightMillis >= lightInterval) && !topic_status) {
//     Serial.println("Turning the light OFF after 10 seconds");
//     digitalWrite(home_light, LOW);
//     lightOn = false;
   
//   }
//   // Check if fan has been on for the specified interval
//   if (fanOn && (currentMillis - previousFanMillis >= fanInterval) && !topic_status) {
//     Serial.println("Turning the fan OFF after 15 seconds");
//     digitalWrite(home_fan, LOW);
//     fanOn = false;
//   }

//   // Periodically publish the status
//   if (currentMillis - previousStatusPublishMillis >= statusPublishInterval) {
//     publishStatus();
//     previousStatusPublishMillis = currentMillis;
//   }
// }
