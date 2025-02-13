#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>

// WiFi credentials
const char* ssid = "Lucifer";
const char* password = "12345678";

// Web server
WebServer server(80);

// Sensor Pin Definitions
const int ldrPin = 16;
const int moisturePin = 12;
const int ultrasonicTrig = 15;
const int ultrasonicEcho = 14;
const int soundPin = 4;
const int vibrationPin = 4; 
const int servoPin = 12;
const int mq7Pin = 13;
const int dhtPin = 2;

#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);

// Create instances of sensors
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Servo roverServo;

// Variables for sensor data
long distance;
int ldrValue, moistureValue, soundValue, vibrationValue, mq7Value;
float temperature, humidity, pressure;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

void setup() {
  Serial.begin(115200);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");
  Serial.println("IP address: " + WiFi.localIP().toString());

  // Initialize sensors
  roverServo.attach(servoPin);
  roverServo.write(0);

  dht.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor");
    while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  pinMode(ultrasonicTrig, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);
  pinMode(ldrPin, INPUT);
  pinMode(moisturePin, INPUT);
  pinMode(soundPin, INPUT);
  pinMode(vibrationPin, INPUT);
  pinMode(mq7Pin, INPUT);

  // Define web server routes
  server.on("/", handleRoot);
  server.on("/ldr", handleLdr);
  server.on("/moisture", handleMoisture);
  server.on("/ultrasonic", handleUltrasonic);
  server.on("/sound", handleSound);
  server.on("/vibration", handleVibration);
  server.on("/mq7", handleMq7);
  server.on("/bmp280", handleBmp280);
  server.on("/dht11", handleDht11);
  server.on("/mpu6050", handleMpu6050);
  server.on("/servo", handleServo);

  // Start the server
  server.begin();
}

void loop() {
  ldrValue = analogRead(ldrPin);
  moistureValue = analogRead(moisturePin);
  soundValue = digitalRead(soundPin);
  vibrationValue = digitalRead(vibrationPin);
  mq7Value = analogRead(mq7Pin);
  distance = getUltrasonicDistance();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  pressure = bmp.readPressure() / 100.0F;

  server.handleClient();
}

void handleRoot() {
  String html = "<html><body><h1>Rover Sensor Data</h1>";
  html += "<p><a href='/ldr'>LDR Value</a></p>";
  html += "<p><a href='/moisture'>Soil Moisture</a></p>";
  html += "<p><a href='/ultrasonic'>Ultrasonic Distance</a></p>";
  html += "<p><a href='/sound'>Sound Sensor</a></p>";
  html += "<p><a href='/vibration'>Vibration Sensor</a></p>";
  html += "<p><a href='/mq7'>MQ7 Gas Sensor</a></p>";
  html += "<p><a href='/bmp280'>BMP280 Data</a></p>";
  html += "<p><a href='/dht11'>DHT11 Data</a></p>";
  html += "<p><a href='/mpu6050'>MPU6050 Data</a></p>";
  html += "<p><a href='/servo'>Control Servo</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleLdr() {
  server.send(200, "text/plain", "LDR Value: " + String(ldrValue));
}

void handleMoisture() {
  server.send(200, "text/plain", "Soil Moisture Value: " + String(moistureValue));
}

void handleUltrasonic() {
  server.send(200, "text/plain", "Distance: " + String(distance) + " cm");
}

void handleSound() {
  server.send(200, "text/plain", "Sound Detected: " + String(soundValue == HIGH ? "Yes" : "No"));
}

void handleVibration() {
  server.send(200, "text/plain", "Vibration Detected: " + String(vibrationValue == HIGH ? "Yes" : "No"));
}

void handleMq7() {
  server.send(200, "text/plain", "MQ7 Gas Sensor Value: " + String(mq7Value));
}

void handleBmp280() {
  server.send(200, "text/plain", "Temperature: " + String(temperature) + " °C, Pressure: " + String(pressure) + " hPa");
}

void handleDht11() {
  server.send(200, "text/plain", "Temperature: " + String(temperature) + " °C, Humidity: " + String(humidity) + " %");
}

void handleMpu6050() {
  String response = "Accel X: " + String(accelX) + ", Accel Y: " + String(accelY) + ", Accel Z: " + String(accelZ);
  response += "\nGyro X: " + String(gyroX) + ", Gyro Y: " + String(gyroY) + ", Gyro Z: " + String(gyroZ);
  server.send(200, "text/plain", response);
}

void handleServo() {
  int angle = server.arg("angle").toInt();
  roverServo.write(angle);
  server.send(200, "text/plain", "Servo Position: " + String(angle));
}

long getUltrasonicDistance() {
  digitalWrite(ultrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrig, LOW);
  long duration = pulseIn(ultrasonicEcho, HIGH);
  return (duration / 2) / 29.1;
}
