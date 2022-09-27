#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>

int led = 13;
Adafruit_BNO08x imu = Adafruit_BNO08x();
sh2_SensorValue_t imuData;

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
  imu.begin_I2C();
}

void loop() {
  digitalWrite(led, HIGH);
  delay(50);
  digitalWrite(led, LOW);
  delay(50);
  imu.getSensorEvent(&imuData);
  Serial.print(imuData.un.accelerometer.x);
  Serial.print(" ");
  Serial.print(imuData.un.accelerometer.y);
  Serial.print(" ");
  Serial.println(imuData.un.accelerometer.z);
}