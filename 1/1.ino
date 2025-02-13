#include <BluetoothSerial.h>
#include <DHT.h>

// Define Pins (adjust these to your ESP32-WROOM setup)
#define DHTPIN 15       // GPIO15 (update to the pin connected to the DHT sensor)
#define TRIGPIN 4       // GPIO4 (for the ultrasonic sensor trigger)
#define ECHOPIN 16      // GPIO16 (for the ultrasonic sensor echo)
#define SOILPIN 34      // GPIO34 (ADC1 channel, for the soil moisture sensor)
#define IN1 26          // GPIO26 (motor driver pin)
#define IN2 25          // GPIO25 (motor driver pin)
#define IN3 33          // GPIO33 (motor driver pin)
#define IN4 32          // GPIO32 (motor driver pin)

// Define DHT sensor type
#define DHTTYPE DHT11

// Initialize objects
DHT dht(DHTPIN, DHTTYPE);
BluetoothSerial SerialBT; // Bluetooth Serial

// Variables
float temperature, humidity, distance, soilMoisture;

void setup() {
  // Serial Communication
  Serial.begin(115200);

  // Initialize Bluetooth
  SerialBT.begin("AgricultureRover"); // Bluetooth name
  Serial.println("Bluetooth started! Connect to 'AgricultureRover'.");

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

  // Send data via Bluetooth
  String output = "Temperature: " + String(temperature) + " °C\n";
  output += "Humidity: " + String(humidity) + " %\n";
  output += "Soil Moisture: " + String(soilMoisture) + " %\n";
  output += "Obstacle Distance: " + String(distance) + " cm\n";
  SerialBT.println(output);

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
