#include <AccelStepper.h>  // Include the AccelStepper library

// Define the control pins for the ULN2003 driver (for a 28BYJ-48 stepper motor)
#define LEFT_PIN_1 8
#define LEFT_PIN_2 9
#define LEFT_PIN_3 10
#define LEFT_PIN_4 11

#define RIGHT_PIN_1 4
#define RIGHT_PIN_2 5
#define RIGHT_PIN_3 6
#define RIGHT_PIN_4 7

// Define ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 13

// PID Constants
const float Kp = 2.4;   // Proportional constant
const float Kd = 0.9;   // Derivative constant
const float Ki = 0.0;   // Integral constant (not used in this simple example)

// Robot's physical constants
const float wheel_base = 0.825;  // distance between left and right wheels in meters
const float wheel_radius = 0.04; // radius of the wheel in meters

// Goal position (x, y)
float goal_x = 1.0;   // Set goal x position
float goal_y = 1.0;   // Set goal y position

// Create instances for left and right stepper motors using the ULN2003 driver
AccelStepper left_motor(AccelStepper::FULL4WIRE, LEFT_PIN_1, LEFT_PIN_2, LEFT_PIN_3, LEFT_PIN_4);
AccelStepper right_motor(AccelStepper::FULL4WIRE, RIGHT_PIN_1, RIGHT_PIN_2, RIGHT_PIN_3, RIGHT_PIN_4);

// Robot's current position and state
float current_x = 0.0;      // Current x position of the robot (in meters)
float current_y = 0.0;      // Current y position of the robot (in meters)
float current_theta = 0.0;  // Current orientation of the robot (in radians)

// Ultrasonic distance variable
long distance = 0;  // Distance measured by the ultrasonic sensor

// Initialize the ultrasonic sensor
void setup() {
  Serial.begin(9600);

  // Set max speed and acceleration for the stepper motors
  left_motor.setMaxSpeed(1000);   // Max speed in steps per second
  left_motor.setAcceleration(500);  // Acceleration in steps per second squared
  right_motor.setMaxSpeed(1000);  // Max speed in steps per second
  right_motor.setAcceleration(500);  // Acceleration in steps per second squared

  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Compute the position and velocity of the robot
  computeRobotPosition();

  // Read ultrasonic distance
  distance = getUltrasonicDistance();

  // Check if the robot is too close to an obstacle (threshold distance)
  if (distance < 20) {  // Obstacle detected within 20 cm
    // Stop or turn to avoid the obstacle
    avoidObstacle();
  } else {
    // Compute PID errors and desired velocities to reach the goal
    float error_x = goal_x - current_x;
    float error_y = goal_y - current_y;

    // Calculate desired theta using the goal position and current position
    float desired_theta = atan2(error_y, error_x);
    float angle_error = desired_theta - current_theta;

    // Angle error wrapping to be between -PI and PI
    if (angle_error > PI) angle_error -= 2 * PI;
    if (angle_error < -PI) angle_error += 2 * PI;

    // Control speed and steering using PID (only proportional and derivative here)
    float linear_velocity = Kp * sqrt(error_x * error_x + error_y * error_y);
    float angular_velocity = Kp * angle_error + Kd * (angle_error - current_theta);

    // Convert desired velocities into stepper steps (scale by the wheel's geometry)
    int left_motor_speed = constrain(linear_velocity - angular_velocity * wheel_base / 2, -255, 255);
    int right_motor_speed = constrain(linear_velocity + angular_velocity * wheel_base / 2, -255, 255);

    // Convert the desired velocities into steps for the stepper motors
    long left_steps = left_motor_speed * 100;  // Adjust scaling factor as per your motor specifications
    long right_steps = right_motor_speed * 100;  // Adjust scaling factor as per your motor specifications

    // Set the stepper motor target positions (steps to move)
    left_motor.moveTo(left_steps);
    right_motor.moveTo(right_steps);

    // Run the motors
    left_motor.run();
    right_motor.run();

    // Print the robot status for debugging
    Serial.print("X: ");
    Serial.print(current_x);
    Serial.print(" Y: ");
    Serial.print(current_y);
    Serial.print(" Theta: ");
    Serial.println(current_theta);
  }

  delay(50);
}

void avoidObstacle() {
  // Stop the motors when obstacle is detected
  left_motor.stop();
  right_motor.stop();
  Serial.println("Obstacle detected! Stopping...");

  // Optionally, make the robot turn (or reverse) to avoid the obstacle
  // For example, turn right for a few seconds
  turnRight();
  delay(1000);
}

void turnRight() {
  // Turn the robot right by rotating the motors in opposite directions
  left_motor.setSpeed(200);  // Adjust speed
  right_motor.setSpeed(-200);  // Reverse direction
  left_motor.runSpeed();
  right_motor.runSpeed();
}

long getUltrasonicDistance() {
  // Send a pulse to trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the pulse on the Echo pin
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (duration in microseconds / 2) * speed of sound (340 m/s)
  long distance = (duration * 0.0344) / 2;

  return distance;  // Return the measured distance in centimeters
}

void computeRobotPosition() {
  // Compute the robot's position based on encoder feedback (same as before)
  // For simplicity, using previous logic
  long left_pos = left_motor.currentPosition();
  long right_pos = right_motor.currentPosition();

  float delta_left = (left_pos - left_motor.currentPosition()) * (2 * PI * wheel_radius) / 360.0;
  float delta_right = (right_pos - right_motor.currentPosition()) * (2 * PI * wheel_radius) / 360.0;

  float delta_dist = (delta_left + delta_right) / 2.0;
  float delta_theta = (delta_right - delta_left) / wheel_base;

  current_theta += delta_theta;
  current_x += delta_dist * cos(current_theta);
  current_y += delta_dist * sin(current_theta);
}
