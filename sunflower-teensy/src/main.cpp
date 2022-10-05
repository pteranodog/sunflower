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
// Serial1 pin
const int Serial1_TX = 1;
// Photoresistor selection pins
const int PHOTORESISTOR_0 = 2;
const int PHOTORESISTOR_1 = 3;
const int PHOTORESISTOR_2 = 4;
const int PHOTORESISTOR_3 = 5;
const int PHOTORESISTOR_4 = 6;
const int PHOTORESISTOR_5 = 7;
const int PHOTORESISTOR_6 = 8;
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
// I2C pins
const int I2C_SDA = 18;
const int I2C_SCL = 19;
// --------------------------

unsigned int packets = 0;
unsigned int millisAtStart = 0;
unsigned int loopTime = 0;

struct {
  bool accel;
  bool gyro;
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
  float i;
  float j;
  float k;
  float real;
} rotVec;
double temp = 0;
double pressure = 0;
double altitude = 0;
struct Eye {
  int array[7];
};
struct {
  Eye front;
  Eye left;
  Eye right;
} SPS;
unsigned int SPSRow = 0;
float sunAngle = 0;

// ------ SENSOR SETUP ------
// IMU sensor setup
Adafruit_BNO08x imu = Adafruit_BNO08x();
sh2_SensorValue_t imuData;
// Barometer sensor setup
Adafruit_BMP3XX barometer = Adafruit_BMP3XX();
// --------------------------


// -- FUNCTION DECLARATION --
void getSensorData();
FlightState updateState(FlightState state);
void stabilize();
void blinkLED(int LEDPin);
void writeTelemetry();
void parseIMUData();
void updateSPS();
float calculateSunAngle();
// --------------------------


// -- MAIN SETUP FUNCTION --
void setup() {
  // Pin mode setup!
  pinMode(BOARD_LED, OUTPUT);
  pinMode(BOX_LED, OUTPUT);
  pinMode(CAMERA, OUTPUT);
  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);
  pinMode(PHOTORESISTOR_0, OUTPUT);
  pinMode(PHOTORESISTOR_1, OUTPUT);
  pinMode(PHOTORESISTOR_2, OUTPUT);
  pinMode(PHOTORESISTOR_3, OUTPUT);
  pinMode(PHOTORESISTOR_4, OUTPUT);
  pinMode(PHOTORESISTOR_5, OUTPUT);
  pinMode(PHOTORESISTOR_6, OUTPUT);
  pinMode(EYE_FRONT, INPUT);
  pinMode(EYE_LEFT, INPUT);
  pinMode(EYE_RIGHT, INPUT);

  // Start I2C
  Wire.begin();
  imu.begin_I2C();
  barometer.begin_I2C();
  // Enable IMU Reports
  imu.enableReport(SH2_ACCELEROMETER);
  imu.enableReport(SH2_GYROSCOPE_CALIBRATED);
  imu.enableReport(SH2_ROTATION_VECTOR);
  // Start Serial1 output
  Serial1.begin(115200);
  // I don't know what this does but it was in the example
  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);
}
// --------------------------


// --- MAIN LOOP FUNCTION ---
void loop() {
  getSensorData();
  blinkLED(BOARD_LED);
  writeTelemetry();
}
// --------------------------


// -- FUNCTION DEFINITIONS --
void getSensorData() {
  // Get sensor data
  parseIMUData();
  barometer.performReading();
  updateSPS();
  sunAngle = calculateSunAngle();
}

void parseIMUData() {
  updated.accel = false;
  updated.gyro = false;
  updated.rotVec = false;
  while ((!updated.accel) || (!updated.gyro) || (!updated.rotVec)) {
    if (imu.getSensorEvent(&imuData)) {
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
  static int millisAtLastBlink = millis();
  if (millis() - millisAtLastBlink > 1000) {
    digitalWrite(LEDPin, HIGH);
    millisAtLastBlink = millis();
  } else {
    digitalWrite(LEDPin, LOW);
  }
}

void writeTelemetry() {
  Serial1.print("RCS1,");
  Serial1.print(millis());
  Serial1.print(",");
  Serial1.print(packets++);
  Serial1.print(",");
  Serial1.print(currentState);
  Serial1.print(",");
  Serial1.print("OFF"); // cam state?
  Serial1.print(",");
  Serial1.print(barometer.readAltitude(1013.25));
  Serial1.print(",");
  Serial1.print(barometer.temperature);
  Serial1.print(",");
  Serial1.print(accel.x);
  Serial1.print(",");
  Serial1.print(accel.y);
  Serial1.print(",");
  Serial1.print(accel.z);
  Serial1.print(",");
  Serial1.print(gyro.x);
  Serial1.print(",");
  Serial1.print(gyro.y);
  Serial1.print(",");
  Serial1.print(gyro.z);
  Serial1.print(",");
  Serial1.print(rotVec.i);
  Serial1.print(",");
  Serial1.print(rotVec.j);
  Serial1.print(",");
  Serial1.print(rotVec.k);
  Serial1.print(",");
  Serial1.print(rotVec.real);
  Serial1.print(",");
  Serial1.print(SPSRow);
  Serial1.print(",");
  Serial1.print(SPS.front.array[SPSRow]);
  Serial1.print(",");
  Serial1.print(SPS.left.array[SPSRow]);
  Serial1.print(",");
  Serial1.print(SPS.right.array[SPSRow]);
  Serial1.print(",");
  // Sun angle
  Serial1.print(",");
  Serial1.print(barometer.pressure);
  Serial1.print(",");
  // board temp
  Serial1.print(",");
  // PID out
  Serial1.print(",");
  // deadzone
  Serial1.print(",");
  // deadspeed
  Serial1.print(",");
  // control out cw
  Serial1.print(",");
  // control out ccw
  Serial1.print(",");
  // calculated thrust
  Serial1.print(",");
  // solenoid on time
  Serial1.println("");  
}
// --------------------------

void updateSPS() {
  SPS.front.array[SPSRow] = analogRead(EYE_FRONT);
  SPS.left.array[SPSRow] = analogRead(EYE_LEFT);
  SPS.right.array[SPSRow] = analogRead(EYE_RIGHT);
  digitalWrite(PHOTORESISTOR_0 + SPSRow, LOW);
  SPSRow = (SPSRow + 1) % 7;
  digitalWrite(PHOTORESISTOR_0 + SPSRow, HIGH);
}

float calculateSunAngle() {
  // Calculate sun angle
  return 0;
}