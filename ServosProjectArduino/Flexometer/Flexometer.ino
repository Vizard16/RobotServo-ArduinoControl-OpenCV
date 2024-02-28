#include <VarSpeedServo.h>

const int flexPin = A0;        // Analog input pin for the flex sensor
const int servoPin = 11;       // Digital pin for the servo control
const int flexThreshold = 100; // Adjust this value based on your sensor's calibration

VarSpeedServo servo;

void setup() {
  servo.attach(servoPin); // Attaches the servo on pin 11 to the servo object
  servo.write(90);        // Set the initial position of the servo to 90 degrees
  Serial.begin(9600);     // Initialize serial communication for debugging
}

void loop() {
  int flexValue = analogRead(flexPin); // Read the value from the flex sensor
  
  // Map the flex sensor values to the servo range (0-180 degrees)
  int servoAngle = map(flexValue, flexThreshold, 1023, 0, 180);
  
  // Limit the servo angle to prevent mechanical issues
  servoAngle = constrain(servoAngle, 0, 180); 
  
  // Set the speed of the servo based on the flex sensor input
  // Higher flex sensor values will lead to faster servo movement
  int servoSpeed = map(flexValue, flexThreshold, 1023, 1, 50);
  
  // Move the servo to the desired position with speed control
  servo.slowmove(servoAngle, servoSpeed);
  
  // Check if the flex sensor value has changed significantly
  static int previousFlexValue = flexValue;
  int flexChangeThreshold = 10; // Adjust this value based on sensitivity
  
  if (abs(flexValue - previousFlexValue) > flexChangeThreshold) {
    // Flex sensor has been bent significantly
    Serial.println("Flex sensor bent!");
    // You can add additional actions or messages here if needed
  }
  
  previousFlexValue = flexValue; // Update previousFlexValue for the next iteration

  // Add a delay to make the movement and message visible; adjust as needed
  delay(50);
}
