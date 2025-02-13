#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DHT.h>

// Define Pins
#define DHTPIN 15
#define DHTTYPE DHT11
#define TRIGPIN 13
#define ECHOPIN 12
#define SOILPIN 34
#define IN1 2
#define IN2 4
#define IN3 16
#define IN4 17

// WiFi Credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Initialize objects
DHT dht(DHTPIN, DHTTYPE);
AsyncWebServer server(80);

// Variables
float temperature, humidity, distance, soilMoisture;

void setup() {
  // Serial Communication
  Serial.begin(115200);
  
  // Initialize sensors
  dht.begin();
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(SOILPIN, INPUT);
  
  // Initialize motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<h1>Agriculture Rover</h1>";
    html += "<p>Temperature: " + String(temperature) + " °C</p>";
    html += "<p>Humidity: " + String(humidity) + " %</p>";
    html += "<p>Soil Moisture: " + String(soilMoisture) + " %</p>";
    html += "<p>Obstacle Distance: " + String(distance) + " cm</p>";
    request->send(200, "text/html", html);
  });

  server.begin();
}

void loop() {
  // Read DHT11 Sensor
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Read Soil Moisture
  soilMoisture = analogRead(SOILPIN);
  soilMoisture = map(soilMoisture, 0, 4095, 0, 100); // Convert to percentage

  // Read Ultrasonic Sensor
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  distance = pulseIn(ECHOPIN, HIGH) * 0.034 / 2;

  // Obstacle Avoidance Logic
  if (distance < 10) {
    // Stop motors
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } else {
    // Move forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  delay(1000);
}