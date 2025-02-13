#include "BluetoothSerial.h"
const char *pin = "1234"; 
String device_name = "harshan"; // Fill device name like "STUDENTNAME_GRADE_SCHOOL NAME IN SHORTFORM".
int kf = 0;
int kb = 0;
int ba;
#define motor1t1 19
#define motor1t2 18
#define motor2t1 5
#define motor2t2 17
#define speed 15
#define touch 26
const int back_switch = 16;
#define BLE_conn 2

BluetoothSerial SerialBT;
char value;

bool debounceSwitch(int pin) {
  static bool lastState = HIGH;
  static unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 50;  // 50ms debounce delay

  bool currentState = digitalRead(pin);
  if (currentState != lastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentState == LOW) {
      lastState = currentState;
      return true;
    }
  }
  lastState = currentState;
  return false;
}

int speedValue = 0; 
void setup() {
  Serial.begin(115200);
  pinMode(back_switch, INPUT_PULLUP);
  pinMode(motor1t1, OUTPUT);
  pinMode(motor1t2, OUTPUT);
  pinMode(motor2t1, OUTPUT);
  pinMode(motor2t2, OUTPUT);
  pinMode(speed, OUTPUT);
  pinMode(touch, OUTPUT);
  pinMode(BLE_conn, OUTPUT);

  analogWrite(speed, 100);
  SerialBT.begin(device_name); 
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

  for (speedValue = 0; speedValue <= 80; speedValue++) {
    digitalWrite(motor1t1,HIGH);
    digitalWrite(motor2t1,HIGH);
    analogWrite(speed, speedValue);
    delay(50);  
  }

}

void loop() {
  d(); 
  if (SerialBT.hasClient()) {
    digitalWrite(BLE_conn, HIGH);
  } else {
    digitalWrite(BLE_conn, LOW);
    motoralloff();
  }
   
  if (Serial.available()) {
    SerialBT.write(Serial.read());
    value = Serial.read();
  }
  if (SerialBT.available()) {
    value = SerialBT.read();
    Serial.println(value);
  }

  // Motor control based on Bluetooth command
  if (value == 'F') {
    motor1forward();
    motor2forward();
  } else if (value == 'B') {
    motor1reverse();
    motor2reverse();
  } else if (value == 'R') {
    motor1forward();
    motor2reverse();
  } else if (value == 'L') {
    motor1reverse();
    motor2forward();
  } else if (value >= '1' && value <= '9') {
    int speedValue = (value - '0') * 20 + 40;
    analogWrite(speed, speedValue);
  } else {
    motoralloff();
  }
}

void d() {
  ba = debounceSwitch(back_switch) ? LOW : HIGH; // Debounce the switch

  if (ba == LOW && kf == 1 && kb == 0) {
    kf = 0;
    kb = 1;
    delay(10); // Consider removing or reducing the delay
  } else if (ba == LOW && kf == 0 && kb == 0) {
    kf = 1;
    kb = 1;
    delay(10); // Consider removing or reducing the delay
  }

  if (ba == HIGH) {
    kb = 0;
  }

  digitalWrite(touch, kf); // Control the LED or other indicator
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
