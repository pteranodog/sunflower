// -------- HEADERS ---------
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
// --------------------------
 

// ------ FLIGHT STATE ------
// Flight state definitions and initialization
enum FlightState {LAUNCH, ASCENT, STABILIZATION, DESCENT, LANDING, LANDED};
FlightState currentState = LAUNCH;
// --------------------------


// ---- PIN NUMBER SETUP ----
// Serial pin
const int SERIAL_TX = 1;
// Photoresistor selection pins
const int PHOTORESISTOR_1 = 2;
const int PHOTORESISTOR_2 = 3;
const int PHOTORESISTOR_3 = 4;
const int PHOTORESISTOR_4 = 5;
const int PHOTORESISTOR_5 = 6;
const int PHOTORESISTOR_6 = 7;
const int PHOTORESISTOR_7 = 8;
// Solenoid control pins
const int SOLENOID_CW = 9;
const int SOLENOID_CCW = 10;
// Camera pin
const int CAMERA = 11;
// LED pins
const int BOX_LED = 12;
const int BOARD_LED = 13;
// Eye readout pins
const int EYE_FRONT = 14;
const int EYE_LEFT = 15;
const int EYE_RIGHT = 16;
const int EYE_BACK = 17;
// I2C pins
const int I2C_SDA = 18;
const int I2C_SCL = 19;
// --------------------------

unsigned int packets = 0;
unsigned int millisAtStart = 0;

struct {
  bool accel;
  bool gyro;
  bool mag;
  bool linAccel;
  bool rotVec;
} updated;
struct {
  float x;
  float y;
  float z;
} accel;
struct {
  float x;
  float y;
  float z;
} gyro;
struct {
  float x;
  float y;
  float z;
} mag;
struct {
  float x;
  float y;
  float z;
} linAccel;
struct {
  float i;
  float j;
  float k;
  float real;
} rotVec;
double temp = 0;
double pressure = 0;
double altitude = 0;


// ------ SENSOR SETUP ------
// IMU sensor setup
Adafruit_BNO08x imu = Adafruit_BNO08x();
sh2_SensorValue_t imuData;
// Barometer sensor setup
// Adafruit_BMP3XX barometer = Adafruit_BMP3XX();
// --------------------------


// -- FUNCTION DECLARATION --
void getSensorData();
FlightState updateState(FlightState state);
void stabilize();
void blinkLED(int LEDPin);
void writeTelemetry();
void parseIMUData();
// --------------------------


// -- MAIN SETUP FUNCTION --
void setup() {
  // Pin mode setup!
  pinMode(BOARD_LED, OUTPUT);

  // Start I2C
  Wire.begin();
  imu.begin_I2C();
  // barometer.begin_I2C();

  // Enable IMU Reports
  imu.enableReport(SH2_ACCELEROMETER, 50000);
  imu.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000);
  imu.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000);
  imu.enableReport(SH2_LINEAR_ACCELERATION, 50000);
  imu.enableReport(SH2_ROTATION_VECTOR, 50000);


  // Start serial output
  Serial.begin(115200);

  // I don't know what this does but it was in the example
  // barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  // barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  // barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  // barometer.setOutputDataRate(BMP3_ODR_50_HZ);
}
// --------------------------


// --- MAIN LOOP FUNCTION ---
void loop() {
  millisAtStart = millis();
  getSensorData();
  blinkLED(BOARD_LED);
  writeTelemetry();
  // delay(50 - (millis() - millisAtStart));
}
// --------------------------


// -- FUNCTION DEFINITIONS --
void getSensorData() {
  // Get sensor data
  parseIMUData();
  // barometer.performReading();
  // Put sensor data into variables
}

void parseIMUData() {
  updated.accel = false;
  updated.gyro = false;
  updated.mag = false;
  updated.linAccel = false;
  updated.rotVec = false;
  while ((!updated.accel) || (!updated.gyro) || (!updated.mag) || (!updated.linAccel) || (!updated.rotVec)) {
    imu.getSensorEvent(&imuData);
    switch (imuData.sensorId) {
      case SH2_ACCELEROMETER:
        accel.x = imuData.un.accelerometer.x;
        accel.y = imuData.un.accelerometer.y;
        accel.z = imuData.un.accelerometer.z;
        updated.accel = true;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gyro.x = imuData.un.gyroscope.x;
        gyro.y = imuData.un.gyroscope.y;
        gyro.z = imuData.un.gyroscope.z;
        updated.gyro = true;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mag.x = imuData.un.magneticField.x;
        mag.y = imuData.un.magneticField.y;
        mag.z = imuData.un.magneticField.z;
        updated.mag = true;
        break;
      case SH2_LINEAR_ACCELERATION:
        linAccel.x = imuData.un.linearAcceleration.x;
        linAccel.y = imuData.un.linearAcceleration.y;
        linAccel.z = imuData.un.linearAcceleration.z;
        updated.linAccel = true;
        break;
      case SH2_ROTATION_VECTOR:
        rotVec.i = imuData.un.rotationVector.i;
        rotVec.j = imuData.un.rotationVector.j;
        rotVec.k = imuData.un.rotationVector.k;
        rotVec.real = imuData.un.rotationVector.real;
        updated.rotVec = true;
        break;
    }
  }
}

FlightState updateState() {
  FlightState state = currentState;
  // Update the flight state
  return state;
}

void stabilize() {
  // Stabilize
}

void blinkLED(int LEDPin) {
  static int cyclesSinceLastBlink = 0;
  if (cyclesSinceLastBlink >= 20) {
    digitalWrite(LEDPin, HIGH);
    cyclesSinceLastBlink = 0;
  }
  else {
    if (cyclesSinceLastBlink == 0) {
      digitalWrite(LEDPin, LOW);
    }
    cyclesSinceLastBlink++;
  }
}

void writeTelemetry() {
  Serial.print("RCS1,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(packets++);
  Serial.print(",");
  Serial.print(currentState);
  Serial.print(",");
  // cam state?
  Serial.print(",");
  // Serial.print(barometer.readAltitude(1013.25));
  Serial.print(",");
  // Serial.print(barometer.temperature);
  Serial.print(",");
  Serial.print(accel.x);
  Serial.print(",");
  Serial.print(accel.y);
  Serial.print(",");
  Serial.print(accel.z);
  Serial.print(",");
  Serial.print(gyro.x);
  Serial.print(",");
  Serial.print(gyro.y);
  Serial.print(",");
  Serial.print(gyro.z);
  Serial.print(",");
  Serial.print(mag.x);
  Serial.print(",");
  Serial.print(mag.y);
  Serial.print(",");
  Serial.print(mag.z);
  Serial.print(",");
  Serial.print(linAccel.x);
  Serial.print(",");
  Serial.print(linAccel.y);
  Serial.print(",");
  Serial.print(linAccel.z);
  Serial.print(",");
  Serial.print(rotVec.i);
  Serial.print(",");
  Serial.print(rotVec.j);
  Serial.print(",");
  Serial.print(rotVec.k);
  Serial.print(",");
  Serial.print(rotVec.real);
  Serial.print(",");
  // SPS row
  Serial.print(",");
  // SPS a
  Serial.print(",");
  // SPS b
  Serial.print(",");
  // SPS c
  Serial.print(",");
  // SPS d
  Serial.print(",");
  // Sun angle
  Serial.print(",");
  // Serial.print(barometer.pressure);
  Serial.print(",");
  // board temp
  Serial.print(",");
  // PID out
  Serial.print(",");
  // deadzone
  Serial.print(",");
  // deadspeed
  Serial.print(",");
  // control out cw
  Serial.print(",");
  // control out ccw
  Serial.print(",");
  // calculated thrust
  Serial.print(",");
  // solenoid on time
  Serial.println("");  
}
// --------------------------