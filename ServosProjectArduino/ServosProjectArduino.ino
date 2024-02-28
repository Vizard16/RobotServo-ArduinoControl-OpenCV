#include <VarSpeedServo.h>  // Include the VarSpeedServo library

// Pins for Servo motors
const int servoXPin = 3;   // Replace with the pin connected to the servo motor for X direction
const int servoY1Pin = 6;  // Replace with the pin connected to the first servo motor for Y direction
const int servoY2Pin = 5;  // Replace with the pin connected to the second servo motor for Y direction
const int servoMPin = 9;
const int flexPin = A0;        // Analog input pin for the flex sensor
const int servoPin = 11;       // Digital pin for the servo control
const int xPin = A1;           // Connect X-axis output of ADXL335 to analog pin A1

// Servo objects
VarSpeedServo servoX;      // Use VarSpeedServo instead of Servo for X direction
VarSpeedServo servoY1;     // Use VarSpeedServo for the first Y direction servo
VarSpeedServo servoY2;     // Use VarSpeedServo for the second Y direction servo
VarSpeedServo servoM;      // Use VarSpeedServo for the motor servo
VarSpeedServo servoFlex;   // Use VarSpeedServo for the flex sensor servo
VarSpeedServo servoAccel;  // Use VarSpeedServo for the accelerometer servo

// Flex sensor parameters
const int flexThreshold = 100; // Adjust this value based on your sensor's calibration

// Variables for storing servo positions
int servoXPosition = 90;  // Initial position for servoX (0 to 180 degrees)
int servoYPosition = 90;  // Initial position for servoY (0 to 180 degrees)
int servoMPosition = 90;  // Initial position for servoM (0 to 180 degrees)
int servoFlexPosition = 90; // Initial position for servoFlex (0 to 180 degrees)
int servoAccelPosition = 90; // Initial position for servoAccel (0 to 180 degrees)

void setup() {
  Serial.begin(9600);
  servoX.attach(servoXPin);
  servoY1.attach(servoY1Pin);
  servoY2.attach(servoY2Pin);
  servoM.attach(servoMPin);
  servoFlex.attach(servoPin);
  servoAccel.attach(10); // Assuming you are using pin 10 for the accelerometer servo
  
  servoFlex.write(90); // Set the initial position of the flex sensor servo to 90 degrees
  servoAccel.write(90); // Set the initial position of the accelerometer servo to 90 degrees
}

void loop() {
  if (Serial.available() > 0) {
    // Read data from Serial input
    String data = Serial.readStringUntil('\n');
    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
    String x_str = data.substring(0, commaIndex1);
    String y_str = data.substring(commaIndex1 + 1, commaIndex2);
    String z_str = data.substring(commaIndex2 + 1);

    // Convert data to float values
    float x = x_str.toFloat();
    float y = y_str.toFloat();
    float z = z_str.toFloat();

    // Map the x and y values to the servo angles (0 to 180 degrees)
    int servoXAngle = map(x, -10, 10, 60, 175);
    int servoYAngle = map(y, -5, 5, 55, 175);

    // Move the servos for X and Y directions to the target angles with variable speed (adjust the speed values as needed)
    servoX.slowmove(servoXAngle, 20);   // 10 is the speed (lower value means slower movement)
    servoY1.slowmove(servoYAngle, 20);  // 10 is the speed (lower value means slower movement)
    servoY2.slowmove(servoYAngle, 20);  // 10 is the speed (lower value means slower movement)

    // Map the flex sensor values to the servo range (0-180 degrees)
    int flexValue = analogRead(flexPin);
    int servoFlexAngle = map(flexValue, flexThreshold, 1023, 0, 180);
    servoFlex.slowmove(servoFlexAngle, 20); // Adjust the speed for the flex sensor servo (higher value means faster movement)
  
    // Read accelerometer value
    int xValue = analogRead(xPin);

    // Map the accelerometer value to servo position (adjust the mapping according to your needs)
    // Here, we are using a different mapping to accommodate the orientation and sensitivity of the accelerometer
    int servoAccelAngle = map(xValue, 265, 402, 0, 180);

    // Move the accelerometer servo to the new position with variable speed
    servoAccel.slowmove(servoAccelAngle, 100); // The second argument is the speed (0 to 255)
  }
}

