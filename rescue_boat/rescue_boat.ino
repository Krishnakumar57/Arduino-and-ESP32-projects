#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h> // Use ESP32Servo instead of Servo

// DHT Sensor
#define DHTPIN 14
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// MPU6050
Adafruit_MPU6050 mpu;

// Ultrasonic Sensor
#define TRIG_PIN 12
#define ECHO_PIN 13

// Servo Motor
#define SERVO_PIN 15
Servo servo;

// GPS Module
HardwareSerial gpsSerial(2); // Use HardwareSerial for ESP32 (UART2)

// WiFi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

void setup() {
  Serial.begin(115200);

  // Initialize DHT
  dht.begin();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // Specify RX (16) and TX (17)

  // Initialize Ultrasonic Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize Servo
  servo.attach(SERVO_PIN);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // Initialize Camera
  // ESP32Cam.begin(); // Uncomment if using ESP32-CAM-specific functions
}

void loop() {
  // Radar Scanning with Servo
  for (int angle = 0; angle <= 180; angle += 10) {
    servo.write(angle);
    delay(500);  // Allow time for servo to reach position
    long distance = measureDistance();
    Serial.printf("Angle: %d, Distance: %ld cm\n", angle, distance);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo.write(angle);
    delay(500);
    long distance = measureDistance();
    Serial.printf("Angle: %d, Distance: %ld cm\n", angle, distance);
  }

  // Read DHT sensor
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  Serial.printf("Temperature: %.2f C, Humidity: %.2f %%\n", temp, humidity);

  // Read MPU6050
  sensors_event_t accel, gyro, tempMpu;
  mpu.getEvent(&accel, &gyro, &tempMpu);
  Serial.printf("Acceleration: X=%.2f, Y=%.2f, Z=%.2f\n",
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

  // Read GPS Data
  String gpsData = "";
  while (gpsSerial.available()) {
    gpsData += (char)gpsSerial.read();
  }
  Serial.printf("GPS Data: %s\n", gpsData.c_str());

  delay(2000);
}

// Function to measure distance using ultrasonic sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;  // Convert to cm
  return distance;
}
