#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>

int led = 13;

void getSensorData() {
  // 
}

void stabilize() {
  // 
}

void writeTelemetry() {
  //
}

void setup() {
  pinMode(led, OUTPUT);
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  digitalWrite(led, HIGH);
  delay(250);
  digitalWrite(led, LOW);
  delay(250);
  Serial1.println("Hello World! from the loop!");
  Serial.println("Hello World! from the loop!");
}