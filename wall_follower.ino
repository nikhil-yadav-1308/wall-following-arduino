#include "KickSort.h" // For sorting to get median of distance values

// PID Gains
const double Kp = 2.5;  // Proportional gain
const double Ki = 0; // Integral gain
const double Kd = 0; // Derivative gain

// Variables for PID calculations
double error_left = 0;
double integral = 0;
double prevError = 0;

const double desiredDistance_left = 10.0; // in centimeters

// Motor speeds
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Left Motor pins
int en_left = 10;
int in1 = 13;
int in2 = 12;
// Right Motor pins
int en_right = 11;
int in3 = 8;
int in4 = 9;

// Ultrasonic Sensor pins
int trigPinC = 3;
int echoPinC = 2;
int trigPinL = 4;
int echoPinL = 5;
int trigPinR = 6;
int echoPinR = 7;


void setup() {
    // Set all the motor control pins to outputs
    pinMode(en_left, OUTPUT);
    pinMode(en_right, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Set all the sensor pins
    pinMode(trigPinC, OUTPUT);
    pinMode(echoPinC, INPUT);
    pinMode(trigPinL, OUTPUT);
    pinMode(echoPinL, INPUT);
    pinMode(trigPinR, OUTPUT);
    pinMode(echoPinR, INPUT);
  
    // Turn off motors - Initial state
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    Serial.begin(9600);
}


void loop() {
    // Get distances from 3 sides
    int* distances = findObstacle();
    
    // If nothing is there, do nothing
    if (distances[0] >= 200 & distances[1] >= 200 & distances[2] >= 200) {
        return;
    }
    
    // Limit the error to a certain amount; 
    // minimum value = -20 and maximum = desiredDistance_left
    if (distances[1] >= 20) {
        error_left = desiredDistance_left - 20;
    } else {
        error_left = desiredDistance_left - distances[1];
    }
  
  // PID calculations
  integral += error_left;
  double derivative = error_left - prevError;
  double pidOutput = Kp * error_left + Ki * integral + Kd * derivative;
 
  // If left corner, don’t turn immediately as the car may get stuck
  if (prevError >= 0 & error_left < 0) { // & distances[0]<=40) {
        Serial.println("Left Corner!");
        pidOutput = 20;
  }
  prevError = error_left;

  leftMotorSpeed = 100 + pidOutput;
  rightMotorSpeed = 100 - pidOutput;
  analogWrite(en_left, leftMotorSpeed);
  analogWrite(en_right, rightMotorSpeed);

  // Move
    if (distances[1] <= 25 && distances[0] <= 10) {
        turnRight(); // Turn right by 90 degrees
  } else {
      goForward();
  }
} // End of loop()


// Function to find distances from obstacles on 3 sides
// Output is a vector of size 3: [Front, Left, Right]
int* findObstacle() {
    static int distances[3];
    distances[0] = getDistance(trigPinC, echoPinC); // Front
    distances[1] = getDistance(trigPinL, echoPinL); // Left
    distances[2] = getDistance(trigPinR, echoPinR); // Right
    return distances;
}


// Function to get distance from a sensor
int getDistance(int tPin, int ePin) {
    // Take distances 7 times then take median as the estimate
    long duration;
    int distances[7];
    for (int i=0; i<7; i++) {
        digitalWrite(tPin, LOW);
        delayMicroseconds(2);
        digitalWrite(tPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(tPin, LOW);
    
        duration = pulseIn(ePin, HIGH);
        distances[i] = duration*0.034/2;
    }
  // Sort the distances vector to get median
  KickSort<int>::bubbleSort(distances, 7, KickSort_Dir::DESCENDING);
    return distances[3];
}


void goForward() {
    // Turn on motors - move forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  delay(150);
  
    // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


void turnRight() {
  analogWrite(en_left, 120);
  analogWrite(en_right, 120);
  // Turn on motor - move right
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(400);
  
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
