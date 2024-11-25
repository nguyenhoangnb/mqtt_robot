// Motor pins
const int leftMotorForward = 9;
const int leftMotorBackward = 10;
const int rightMotorForward = 11;
const int rightMotorBackward = 12;

// Ultrasonic sensor pins
const int trigPin = 7;
const int echoPin = 6;

// Constants
const float wheelRadius = 0.4; // meters
const float wheelBase = 0.825; // meters (distance between the wheels)
const float pi = 3.14159265359;

// Variables
float X[3] = {0.0, 0.0, 0.0}; // [x, y, theta] robot state
float Q[2] = {0.0, 0.0}; // [left wheel, right wheel] joint positions

// Control variables
float targetDistance = 30.0; // Distance to detect obstacles (in cm)
float stopDistance = 20.0; // Minimum distance to stop (in cm)
float backDistance = 15.0; // Distance to reverse when too close (in cm)
float moveSpeed = 255; // Speed of moving forward or backward
float turnSpeed = 150; // Speed of turning (adjust for faster/slower turns)

// Function to update the motor speeds
void setMotorSpeeds(float leftSpeed, float rightSpeed) {
    // Convert the speeds to PWM values (simple proportional control)
    int leftPWM = constrain(abs(leftSpeed) * 255, 0, 255);
    int rightPWM = constrain(abs(rightSpeed) * 255, 0, 255);
    
    // Apply direction based on speed sign
    if (leftSpeed >= 0) {
        analogWrite(leftMotorForward, leftPWM);
        analogWrite(leftMotorBackward, 0);
    } else {
        analogWrite(leftMotorForward, 0);
        analogWrite(leftMotorBackward, leftPWM);
    }

    if (rightSpeed >= 0) {
        analogWrite(rightMotorForward, rightPWM);
        analogWrite(rightMotorBackward, 0);
    } else {
        analogWrite(rightMotorForward, 0);
        analogWrite(rightMotorBackward, rightPWM);
    }
}

// Function to measure the distance from the ultrasonic sensor
float getUltrasonicDistance() {
    long duration, distance;
  
    // Send pulse to trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echo pin
    duration = pulseIn(echoPin, HIGH);

    // Calculate the distance (in cm)
    distance = duration * 0.0343 / 2; // Speed of sound = 343 m/s (0.0343 cm/µs)
    
    return distance;
}

// Function to make the robot turn (left or right)
void turnRobot(int direction) {
    // Rotate the robot by setting the left and right motor speeds with opposite directions
    if (direction == 1) { // Turn right
        setMotorSpeeds(turnSpeed, -turnSpeed); // Right motor forward, left motor backward
    } else if (direction == -1) { // Turn left
        setMotorSpeeds(-turnSpeed, turnSpeed); // Left motor forward, right motor backward
    }
    delay(500); // Turn for 0.5 seconds (adjust for the degree of rotation)
    setMotorSpeeds(0, 0); // Stop the robot after turning
}

// Function to move the robot backward
void moveBackward() {
    setMotorSpeeds(-moveSpeed, -moveSpeed); // Move both wheels backward at full speed
    delay(500); // Move backward for 0.5 seconds (adjust for the distance)
    setMotorSpeeds(0, 0); // Stop after reversing
}

// Function to move the robot forward
void moveForward() {
    setMotorSpeeds(moveSpeed, moveSpeed); // Move both wheels forward at full speed
}

// Function to check for obstacles and navigate
void navigate() {
    // Get the distance from the ultrasonic sensor
    float distanceToObject = getUltrasonicDistance();
    Serial.print("Distance: ");
    Serial.println(distanceToObject);

    // If the robot detects an obstacle in front
    if (distanceToObject < stopDistance) {
        // Stop the robot and start obstacle avoidance behavior
        setMotorSpeeds(0, 0); // Stop the robot
        Serial.println("Obstacle detected, stopping!");

        // Reverse a little bit
        moveBackward();

        // Turn to avoid the obstacle (try both directions, left and right)
        int turnDirection = (random(0, 2) == 0) ? 1 : -1; // Randomly choose direction to turn
        turnRobot(turnDirection);

        // Move forward again after turning
        moveForward();
    } else if (distanceToObject < backDistance) {
        // If too close, move backward
        setMotorSpeeds(-moveSpeed, -moveSpeed); // Reverse with full speed
        delay(500); // Reverse for 0.5 seconds (adjust for distance)
        setMotorSpeeds(0, 0); // Stop after reversing
    } else {
        // If no obstacle, continue moving forward
        moveForward();
    }
}

void setup() {
    // Set motor pins as output
    pinMode(leftMotorForward, OUTPUT);
    pinMode(leftMotorBackward, OUTPUT);
    pinMode(rightMotorForward, OUTPUT);
    pinMode(rightMotorBackward, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Start serial communication for debugging
    Serial.begin(9600);
}

void loop() {
    navigate(); // Call the navigation function to check and move the robot
}
