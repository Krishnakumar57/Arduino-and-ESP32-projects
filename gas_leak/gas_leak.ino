#include <BluetoothSerial.h>

// Create a Bluetooth Serial object
BluetoothSerial SerialBT;

// Pin definitions for MQ sensors
const int mq2Pin = 34; // Analog pin for MQ2
const int mq3Pin = 35; // Analog pin for MQ3
const int mq6Pin = 32; // Analog pin for MQ6
const int mq7Pin = 33; // Analog pin for MQ7

// Threshold values for gas and alcohol detection
const int gasThreshold = 300;  // Adjust based on MQ2 sensor calibration
const int alcoholThreshold = 300; // Adjust based on MQ3 sensor calibration

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Start Bluetooth communication
  SerialBT.begin("ESP32_Sensor_Monitor"); // Name of the Bluetooth device
  Serial.println("Bluetooth initialized. Pair your device with 'ESP32_Sensor_Monitor'");

  // Inform that the setup is complete
  Serial.println("Setup complete!");
}

void loop() {
  // Read analog values from MQ sensors
  int mq2Value = analogRead(mq2Pin);
  int mq3Value = analogRead(mq3Pin);
  int mq6Value = analogRead(mq6Pin);
  int mq7Value = analogRead(mq7Pin);

  // Print sensor values to the Bluetooth Serial
  String data = "MQ2 Gas Level: " + String(mq2Value) + "\n";
  data += "MQ3 Alcohol Level: " + String(mq3Value) + "\n";
  data += "MQ6 Gas Level: " + String(mq6Value) + "\n";
  data += "MQ7 Alcohol Level: " + String(mq7Value) + "\n";
  
  // Send the data over Bluetooth
  SerialBT.println(data);

  // Check if gas or alcohol is detected
  if (mq2Value > gasThreshold) {
    SerialBT.println("Gas detected by MQ2!");
  }

  if (mq3Value > alcoholThreshold) {
    SerialBT.println("Alcohol detected by MQ3!");
  }

  if (mq6Value > gasThreshold) {
    SerialBT.println("Gas detected by MQ6!");
  }

  if (mq7Value > alcoholThreshold) {
    SerialBT.println("Alcohol detected by MQ7!");
  }

  // Delay for stability
  delay(1000);
}
