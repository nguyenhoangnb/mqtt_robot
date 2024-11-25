#include <AccelStepper.h>  // Include the AccelStepper library for controlling stepper motors

// Motor pins for ULN2003 (for controlling a 28BYJ-48 stepper motor)
const int leftMotorPin1 = 8;
const int leftMotorPin2 = 9;
const int leftMotorPin3 = 10;
const int leftMotorPin4 = 11;

const int rightMotorPin1 = 4;
const int rightMotorPin2 = 5;
const int rightMotorPin3 = 6;
const int rightMotorPin4 = 7;

// Ultrasonic sensor pins
const int trigPin = 12;
const int echoPin = 13;

// Stepper motor speed and acceleration
const int motorSpeed = 1000;  // Maximum speed for stepper motors (steps per second)
const int motorAccel = 500;   // Acceleration for stepper motors (steps per second squared)

// Distance thresholds
const float stopDistance = 20.0; // Minimum distance to stop (in cm)
const float backDistance = 15.0; // Distance to reverse when too close (in cm)

// Create stepper motor objects for left and right motors
AccelStepper leftMotor(AccelStepper::FULL4WIRE, leftMotorPin1, leftMotorPin2, leftMotorPin3, leftMotorPin4);
AccelStepper rightMotor(AccelStepper::FULL4WIRE, rightMotorPin1, rightMotorPin2, rightMotorPin3, rightMotorPin4);

// Function to set motor speeds (both forward and backward)
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setMaxSpeed(motorSpeed);
    rightMotor.setMaxSpeed(motorSpeed);
    leftMotor.setAcceleration(motorAccel);
    rightMotor.setAcceleration(motorAccel);

    leftMotor.moveTo(leftSpeed);
    rightMotor.moveTo(rightSpeed);
}

// Function to measure the distance from the ultrasonic sensor
float getUltrasonicDistance() {
    long duration;
    long distance;

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
    if (direction == 1) {  // Turn right
        setMotorSpeeds(100, -100);  // Right motor forward, left motor backward
    } else if (direction == -1) {  // Turn left
        setMotorSpeeds(-100, 100);  // Left motor forward, right motor backward
    }
    delay(500);  // Turn for 0.5 seconds (adjust for the degree of rotation)
    setMotorSpeeds(0, 0);  // Stop after turning
}

// Function to move the robot backward
void moveBackward() {
    setMotorSpeeds(-200, -200);  // Move both wheels backward at full speed
    delay(500);  // Move backward for 0.5 seconds (adjust for the distance)
    setMotorSpeeds(0, 0);  // Stop after reversing
}

// Function to move the robot forward
void moveForward() {
    setMotorSpeeds(200, 200);  // Move both wheels forward at full speed
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
        moveBackward();
    } else {
        // If no obstacle, continue moving forward
        moveForward();
    }
}

void setup() {
    // Set motor pins as output
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);
    pinMode(leftMotorPin3, OUTPUT);
    pinMode(leftMotorPin4, OUTPUT);

    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(rightMotorPin3, OUTPUT);
    pinMode(rightMotorPin4, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Start serial communication for debugging
    Serial.begin(9600);
}

void loop() {
    navigate();  // Call the navigation function to check and move the robot
}
