// #include <WiFi.h>
// #include <PubSubClient.h>

// #define MAX_COMMAND 100
// const char* ssid = "NQT";
// const char* password = "11345678";
// #define led1 4 // GPIO pin for LED
// #define led2 16
// #define led3 17

// // MQTT broker information
// const char* mqtt_server = "192.168.1.114"; // Broker IP
// const int mqttPort = 1883;
// const int interval = interval;
// const char* mqttTopic = "commands"; // Topic for receiving commands
// bool topic_status = false;
// WiFiClient espClient;
// PubSubClient client(espClient);

// // Command stack structure
// struct command_stack {
//     int head;
//     int tail;
//     char commands[MAX_COMMAND];
//     unsigned long times[MAX_COMMAND];

//     command_stack() : head(0), tail(0) {}

//     void push(char command, unsigned long time) {
//         if (isFull()) {
//             Serial.println("Stack is full. Cannot push command.");
//             return;
//         }
//         commands[tail] = command;
//         times[tail] = time;
//         tail = (tail + 1) % MAX_COMMAND;
//     }

//     void pop() {
//         if (isEmpty()) {
//             Serial.println("Stack is empty. Cannot pop command.");
//             return;
//         }
//         head = (head + 1) % MAX_COMMAND;
//     }

//     bool isEmpty() {
//         return head == tail;
//     }

//     bool isFull() {
//         return (tail + 1) % MAX_COMMAND == head;
//     }

//     void display() {
//         if (isEmpty()) {
//             Serial.println("Stack is empty.");
//             return;
//         }
//         Serial.println("Stack contents:");
//         int index = head;
//         while (index != tail) {
//             Serial.print("Command: ");
//             Serial.print(commands[index]);
//             Serial.print(", Time: ");
//             Serial.println(times[index]);
//             index = (index + 1) % MAX_COMMAND;
//         }
//     }
// };

// command_stack command;

// // MQTT message callback function
// void callback(char* topic, byte* payload, unsigned int length) {
//     Serial.print("Message arrived [");
//     Serial.print(topic);
//     Serial.print("]: ");

//     for (int i = 0; i < length; i++) {
//         Serial.print((char)payload[i]);
//     }
//     Serial.println();

//     if (length > 0 && payload != nullptr) {
//         command.push((char)payload[0], millis());
//     }

//     topic_status = true;
// }

// // Command execution
// void executeCommand(char command) {
//     unsigned long startMillis;
//     switch (command) {
//         case 'A':
//             Serial.println("Executing Command A: Turning LED ON for 10 seconds.");
//             digitalWrite(led1, LOW);
//             startMillis = millis();
//             while (millis() - startMillis < interval) {
//                 client.loop(); // Maintain MQTT connection
//             }
//             digitalWrite(led1, LOW);
//             break;
//         case 'B':
//             Serial.println("Executing Command B: Turning LED ON permanently.");
//             digitalWrite(led1, HIGH);
//             break;
//         case 'C':
//             Serial.println("Executing Command C: Blinking LED 5 times.");
//             for (int i = 0; i < 5; i++) {
//                 digitalWrite(led1, HIGH);
//                 delay(500);
//                 digitalWrite(led1, LOW);
//                 delay(500);
//             }
//             break;
//         case 'D':
//             Serial.println("Executing Command D: Turning LED OFF.");
//             digitalWrite(led1, LOW);
//             break;
//         default:
//             Serial.println("Unknown Command.");
//             break;
//     }
// }

// // Reconnect to MQTT broker
// void reconnect() {
//     while (!client.connected()) {
//         Serial.print("Attempting MQTT connection...");
//         if (client.connect("ArduinoClient")) {
//             Serial.println("connected");
//             client.subscribe(mqttTopic);
//         } else {
//             Serial.print("failed, rc=");
//             Serial.print(client.state());
//             Serial.println(" retrying in 10 seconds");
//             delay(10000);
//         }
//     }
// }

// // Automatic LED toggling
// void automatic() {
//     bool check = false;

//     while (!check) {
//         // Turn LED ON
//        // Serial.println("Automatic mode: LED ON.");
//         digitalWrite(led1, HIGH);
//         unsigned long startMillis = millis();

//         // Wait for 10 seconds while checking for MQTT messages
//         while (millis() - startMillis < interval) {
//             client.loop(); // Maintain MQTT connection
//             Serial.print("ABC");
//            // delay(100);
//             if (topic_status) {
//                 unsigned long remainingTime = interval - (millis() - startMillis);
//                 command.push('A', remainingTime);
//                 Serial.println("PUSH: LED ON state saved to stack.");
//                 check = true;
//                 break;
//             }
//         }

//         if (check) break;

//         // Turn LED OFF
//         //Serial.println("Automatic mode: LED OFF.");
//         digitalWrite(led1, LOW);
//         startMillis = millis();

//         // Wait for 10 seconds while checking for MQTT messages
//         while (millis() - startMillis < interval) {
//             client.loop(); // Maintain MQTT connection
//             Serial.print("ABC");
//             //delay(100);
//             if (topic_status) {
//                 unsigned long remainingTime = interval - (millis() - startMillis);
//                 command.push('D', remainingTime);
//                 Serial.println("PUSH: LED OFF state saved to stack.");
//                 check = true;
//                 break;
//             }
//         }
//     }

//     Serial.println("Exiting automatic mode due to MQTT command.");
// }



// void setup() {
//     Serial.begin(9600);
//     pinMode(led1, OUTPUT);
//     pinMode(led2, OUTPUT);
//     pinMode(led3, OUTPUT);
//     digitalWrite(led1, LOW);
//     digitalWrite(led2, LOW);
//     digitalWrite(led3, LOW);

//     Serial.print("Connecting to WiFi");
//     WiFi.begin(ssid, password);
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println("\nWiFi connected");

//     client.setServer(mqtt_server, mqttPort);
//     client.setCallback(callback);

//     reconnect();
// }

// void loop() {
//     if (!client.connected()) {
//         reconnect();
//     }
//     client.loop();

//     if (!topic_status) {
//         automatic();
//     }

//     if (!command.isEmpty()) {
//         char cmd = command.commands[command.head];
//         command.pop();
//         executeCommand(cmd);
//     } else {
//         topic_status = false;
//     }
// }
