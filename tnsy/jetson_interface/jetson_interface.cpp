#include <SPI.h>

const int csPin = 10; // Chip Select pin

void setup() {
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH); // Set CS high
  SPI.begin(); // Initialize SPI
  Serial.begin(9600); // Initialize serial for debugging
}

void loop() {
  digitalWrite(csPin, LOW); // Select the slave device
  SPI.transfer("Hello from Teensyduino");
  digitalWrite(csPin, HIGH); // Deselect the slave device

  delay(1000); // Send data every second

  // Reading data from Jetson Orin Nano
  digitalWrite(csPin, LOW); // Select the slave device
  char receivedData = SPI.transfer(0x00); // Dummy byte to read data
  digitalWrite(csPin, HIGH); // Deselect the slave device

  Serial.print("Received: ");
  Serial.println(receivedData);

  delay(1000); // Wait for a second
}