#include "BluetoothSerial.h"
const char *pin = "1234"; 
String device_name = "CYBERPLANT BOT-14";

#define motor1t1 34
#define motor1t2 35
#define motor2t1 32
#define motor2t2 33

#define speed1 2
#define speed2 15

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
char value;

void setup() {
  Serial.begin(115200);
  pinMode(motor1t1,OUTPUT);
  pinMode(motor1t2,OUTPUT);
  pinMode(motor2t1,OUTPUT);
  pinMode(motor2t2,OUTPUT);
  pinMode(speed1,OUTPUT);
  pinMode(speed2,OUTPUT);
  
  analogWrite(speed1,80);
  analogWrite(speed2,80);
  
  digitalWrite(motor1t1,HIGH);
  digitalWrite(motor2t1,HIGH);
  delay(500);
  digitalWrite(motor1t1,LOW);
  digitalWrite(motor1t1,LOW);
  
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
    value = Serial.read();
  }
  if (SerialBT.available()) {
      //Serial.println(SerialBT.read());
      value = SerialBT.read();
      Serial.println(value);
  }

  if (!SerialBT.available()) {
   digitalWrite(motor1t1,LOW);
   digitalWrite(motor1t2,LOW);
   digitalWrite(motor2t1,LOW);
   digitalWrite(motor2t2,LOW);
  }

  if (value == 'F') {
    
      motor1forward();
      motor2forward();
  }
  else if (value == 'B') {
    motor1reverse();
    motor2reverse();
  
 
  }
  else if (value == 'R') {
     motor1forward();
     motor2reverse();

  }
  else if (value == 'L') {
    motor1reverse();
    motor2forward();
    
  }
  else if (value == '1') {
    analogWrite(speed1,60);
    analogWrite(speed2,60);
  }
  else if (value == '2') {
    analogWrite(speed1,80);
    analogWrite(speed2,80);
  }
   else if (value == '3') {
    analogWrite(speed1,100);
    analogWrite(speed2,100);
  }
   else if (value == '4') {
    analogWrite(speed1,120);
    analogWrite(speed2,120);
  }
   else if (value == '5') {
    analogWrite(speed1,140);
    analogWrite(speed2,140);
  }
   else if (value == '6') {
    analogWrite(speed1,160);
    analogWrite(speed2,160);
  }
   else if (value == '7') {
    analogWrite(speed1,180);
    analogWrite(speed2,180);
  }
   else if (value == '8') {
    analogWrite(speed1,200);
    analogWrite(speed2,200);
  }
   else if (value == '9') {
    analogWrite(speed1,220);
    analogWrite(speed2,220);
  }
  else{
    digitalWrite(motor1t1, LOW);
    digitalWrite(motor1t2, LOW);
    digitalWrite(motor2t1, LOW);
    digitalWrite(motor2t2, LOW);
  }

  
}

void motoralloff() {
  digitalWrite(motor1t1, LOW);
  digitalWrite(motor1t2, LOW);
  digitalWrite(motor2t1, LOW);
  digitalWrite(motor2t2, LOW);
}

void motor1forward() {
  digitalWrite(motor1t1, HIGH);
  digitalWrite(motor1t2, LOW);
}

void motor2forward() {
  digitalWrite(motor2t1, HIGH);
  digitalWrite(motor2t2, LOW);
}


void motor1reverse() {
  digitalWrite(motor1t1, LOW);
  digitalWrite(motor1t2, HIGH);
}

void motor2reverse() {
  digitalWrite(motor2t1, LOW);
  digitalWrite(motor2t2, HIGH);
}