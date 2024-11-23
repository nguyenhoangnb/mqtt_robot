#include <WiFi.h>
#include <PubSubClient.h>

// Cấu hình WiFi
const char* ssid = "NQT";           // WiFi SSID
const char* password = "11345678";  // WiFi Password

// Cấu hình MQTT
const char* mqtt_server = "192.168.1.114"; // MQTT Broker IP
const int mqttPort = 1883;
const char* mqttTopic = "commands";       // Topic nhận lệnh

// GPIO cho đèn LED
#define led1 4
#define led2 16
#define led3 17

// Thời gian mặc định
const int interval = 10000; // Thời gian bật/tắt LED trong chế độ tự động (10 giây)

// MQTT và WiFi clients
WiFiClient espClient;
PubSubClient client(espClient);

// Biến trạng thái
bool topic_status = false; // MQTT topic status flag
struct timer_control{
    char command_control;
    unsigned int time_control;
};
int size_timer_control = 4;
unsigned int getValueByKey(timer_control command[], char key) {
    for (int i = 0; i < size_timer_control; i++) {
        if (command[i].command_control == key) {
            return command[i].time_control;  // Return the corresponding value
        }
    }
    return 0;  // Return 0 if key not found
}
timer_control Timer_control[] = {
    {'A', 10000},
    {'B', 5000},
    {'C', 6000},
    {'D', 8000}
};
// Định nghĩa stack lưu lệnh
#define MAX_COMMAND 1000
class CommandStack {
private:
    char commands[MAX_COMMAND];
    unsigned long times[MAX_COMMAND];
    int top;
    char control[MAX_COMMAND];

public:
    CommandStack() : top(-1) {}

    // Thêm lệnh vào stack
    bool push(char command, unsigned long time, char c) {
        if (isFull()) {
            Serial.println("Stack đầy, không thể thêm lệnh.");
            return false;
        }
        top++;
        commands[top] = command;
        times[top] = time;
        control[top] = c;
        return true;
    }

    // Xoá lệnh khỏi stack
    bool pop() {
        if (isEmpty()) {
            Serial.println("Stack rỗng, không thể xoá lệnh.");
            return false;
        }
        if (control[top] == 'C' && top > 1){
            top--;
        }
        if (control[top] == 'A'){
            top--;
        }
        return true;
    }

    // Lấy lệnh trên cùng (không xoá)
    char peekCommand() {
        return isEmpty() ? '\0' : commands[top];
    }

    // Kiểm tra stack rỗng
    bool isEmpty() {
        return top == -1;
    }
    void freeStack(){
        top = -1;
    }
    // Kiểm tra stack đầy
    bool isFull() {
        return top >= MAX_COMMAND - 1;
    }
    unsigned int get_time(){
        return times[top];
    }
    // Hiển thị nội dung stack
    void display() {
        if (isEmpty()) {
            Serial.println("Stack rỗng.");
            return;
        }
        Serial.println("Nội dung Stack:");
        for (int i = 0; i <= top; i++) {
            Serial.print("Lệnh: ");
            Serial.print(commands[i]);
            Serial.print(", Thời gian: ");
            Serial.println(times[i]);
        }
    }
};

CommandStack commandStack;

// Callback nhận lệnh từ MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Nhận lệnh từ [");
    Serial.print(topic);
    Serial.print("]: ");

    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    if (length > 0 && payload != nullptr) {
        char receivedCommand = (char)payload[0];
        topic_status = true;
        if (receivedCommand >= 'A' && receivedCommand <= 'D') { // Kiểm tra lệnh hợp lệ
            commandStack.push(receivedCommand, getValueByKey(Timer_control,receivedCommand), 'C');
        } else {
            Serial.println("Lệnh không hợp lệ.");
        }
    }
}

// Thực thi lệnh
void executeCommand(char cmd, unsigned int time ,char control) {
    switch (cmd) {
        case 'A': {
            Serial.println("Lệnh A: Bật LED1 trong 10 giây.");
            topic_status = false;
            digitalWrite(led1, HIGH);
            unsigned long startMillis = millis();
            while (millis() - startMillis < time && !topic_status) {
                client.loop(); // Duy trì kết nối MQTT
            }
            if (topic_status && control == 'A'){
                unsigned int time_break = millis() - startMillis;
                commandStack.push(cmd, time_break, control);
            }
            digitalWrite(led1, LOW);
            break;
        }
        case 'B': {
            Serial.println("Lệnh B: Bật LED2 vĩnh viễn cho đến khi có lệnh khác.");
            topic_status = false;
            digitalWrite(led2, HIGH);
            unsigned long startMillis = millis();
            while (millis() - startMillis < time && !topic_status){
                client.loop(); // Duy trì kết nối MQTT
            }
            if (topic_status && control == 'A'){
                unsigned int time_break = millis() - startMillis;
                commandStack.push(cmd, time_break , control);
            }
            digitalWrite(led2, LOW);
            break;
        }
        case 'C': {
            Serial.println("Lệnh C: Nhấp nháy LED1 5 lần.");
            unsigned long startMillis = millis();
            while (millis() - startMillis < time && !topic_status){
                client.loop(); // Duy trì kết nối MQTT
                for (int i = 0; i < 5; i++) {
                    digitalWrite(led1, HIGH);
                    delay(500);
                    digitalWrite(led1, LOW);
                    delay(500);
                }
            }
            if (topic_status && control == 'A'){
                unsigned int time_break = millis() - startMillis;
                commandStack.push(cmd, time_break , control);
            }
            
            break;
        }
        case 'D': {
            Serial.println("Lệnh D: Tắt LED1.");
            unsigned long startMillis = millis();
            digitalWrite(led1, LOW);
            while (millis() - startMillis < time && !topic_status){
                client.loop(); // Duy trì kết nối MQTT
            }
            if (topic_status && control == 'A'){
                unsigned int time_break = millis() - startMillis;
                commandStack.push(cmd, time_break , control);
            }
            break;
        }
        default: {
            Serial.println("Lệnh không xác định.");
            break;
        }
    }
}

// Kết nối lại MQTT Broker
void reconnect() {
    while (!client.connected()) {
        Serial.print("Đang kết nối lại MQTT...");
        if (client.connect("ESP32Client")) {
            Serial.println("Kết nối MQTT thành công.");
            client.subscribe(mqttTopic);
        } else {
            Serial.print("Thất bại, rc=");
            Serial.print(client.state());
            Serial.println(" Thử lại sau 5 giây.");
            delay(5000);
        }
    }
}

// Chế độ tự động
void automatic() {
    Serial.println("Chuyển sang chế độ tự động...");
    while (!topic_status){
        int index = random(0, 4);
        commandStack.push(Timer_control[index].command_control, Timer_control[index].time_control, 'A');
        char cmd = Timer_control[index].command_control;
        client.loop();
        unsigned int t_ = commandStack.get_time();
       // Serial.println(Timer_control[index])
       commandStack.pop();
        executeCommand(cmd,t_ ,'A');
    }
    
    Serial.println("Thoát chế độ tự động do có lệnh MQTT.");
}

void setup() {
    Serial.begin(115200);

    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);

    Serial.print("Kết nối WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nKết nối WiFi thành công!");

    client.setServer(mqtt_server, mqttPort);
    client.setCallback(callback);

    reconnect();
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    if (!topic_status) {
        //commandStack.freeStack();
        automatic();
    }
    else if (topic_status){
        char cmd = commandStack.peekCommand();
        unsigned int t_ = commandStack.get_time();
        commandStack.pop();
        executeCommand(cmd, t_, 'C');
    }
    // if (!commandStack.isEmpty()) {
        
    // }
}
