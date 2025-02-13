#include <AFMotor.h>
#include <SoftwareSerial.h>

// Initialize SoftwareSerial on pins 10 and 11
SoftwareSerial BTSerial(10, 9); // 10 to hc-05 TX pin, 9 to hc-05 RX

// Create AF_DCMotor objects for motor control
AF_DCMotor motor1(1); // Left Wheel Front
AF_DCMotor motor2(2); // Right Wheel Front
AF_DCMotor motor3(3); // Left Wheel Back
AF_DCMotor motor4(4); // Right Wheel Back

void setup() {
  // Start serial communication with the Bluetooth module
  BTSerial.begin(9600);
  Serial.begin(9600);

  // Set all motors to initial speed of 0
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  Serial.println("4-Wheel Chassis Control Ready");
}

void loop() {
  // Check if data is available from the Bluetooth module
  if (BTSerial.available()) {
    char command = BTSerial.read();
    
    // Print the received command
    Serial.print("Received: ");
    Serial.println(command);

    // Control motors based on received command
    switch (command) {
      case 'F': // Move Forward
        moveForward();
        break;
      case 'B': // Move Backward
        moveBackward();
        break;
      case 'L': // Turn Left
        turnLeft();
        break;
      case 'R': // Turn Right
        turnRight();
        break;
      case 'S': // Stop
        stopAllMotors();
        break;
      default:
        Serial.println("Unknown Command");
        break;
    }
  }
}

void moveForward() {
  motor1.setSpeed(255); // Set speed to maximum
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
  Serial.println("Moving Forward");
}

void moveBackward() {
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  
  Serial.println("Moving Backward");
}

void turnLeft() {
  motor1.setSpeed(0); // Stop left motors
  motor2.setSpeed(255); // Set right motors to max speed
  motor3.setSpeed(0); // Stop left back motor
  motor4.setSpeed(255); // Set right back motor to max speed
  
  motor1.run(RELEASE);
  motor2.run(FORWARD);
  motor3.run(RELEASE);
  motor4.run(BACKWARD);
  
  Serial.println("Turning Left");
}

void turnRight() {
  motor1.setSpeed(255); // Set left motors to max speed
  motor2.setSpeed(0); // Stop right motors
  motor3.setSpeed(255); // Set left back motor to max speed
  motor4.setSpeed(0); // Stop right back motor
  
  motor1.run(FORWARD);
  motor2.run(RELEASE);
  motor3.run(BACKWARD);
  motor4.run(RELEASE);
  
  Serial.println("Turning Right");
}

void stopAllMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  
  Serial.println("All Motors Stopped");
}