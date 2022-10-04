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
// --------------------------


// -- MAIN SETUP FUNCTION --
void setup() {
  // Pin mode setup!
  pinMode(BOARD_LED, OUTPUT);

  // Start I2C
  Wire.begin();
  imu.begin_I2C();
  barometer.begin_I2C();

  // Enable IMU Reports
  imu.enableReport(SH2_ACCELEROMETER);
  imu.enableReport(SH2_GYROSCOPE_CALIBRATED);
  imu.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR);
  imu.enableReport(SH2_LINEAR_ACCELERATION);
  imu.enableReport(SH2_ROTATION_VECTOR);


  // Start serial output
  Serial.begin(115200);

  // I don't know what this does but it was in the example
  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);
}
// --------------------------


// --- MAIN LOOP FUNCTION ---
void loop() {
  millisAtStart = millis();
  getSensorData();
  blinkLED(BOARD_LED);
  writeTelemetry();
  delay(50 - (millis() - millisAtStart));
}
// --------------------------


// -- FUNCTION DEFINITIONS --
void getSensorData() {
  // Get sensor data
  imu.getSensorEvent(&imuData);
  barometer.performReading();
  // Put sensor data into variables
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
  Serial.print(barometer.readAltitude(1013.25));
  Serial.print(",");
  Serial.print(barometer.temperature);
  Serial.print(",");
  Serial.print(imuData.un.accelerometer.x);
  Serial.print(",");
  Serial.print(imuData.un.accelerometer.y);
  Serial.print(",");
  Serial.print(imuData.un.accelerometer.z);
  Serial.print(",");
  Serial.print(imuData.un.gyroscope.x);
  Serial.print(",");
  Serial.print(imuData.un.gyroscope.y);
  Serial.print(",");
  Serial.print(imuData.un.gyroscope.z);
  Serial.print(",");
  Serial.print(imuData.un.magneticField.x);
  Serial.print(",");
  Serial.print(imuData.un.magneticField.y);
  Serial.print(",");
  Serial.print(imuData.un.magneticField.z);
  Serial.print(",");
  Serial.print(imuData.un.linearAcceleration.x);
  Serial.print(",");
  Serial.print(imuData.un.linearAcceleration.y);
  Serial.print(",");
  Serial.print(imuData.un.linearAcceleration.z);
  Serial.print(",");
  Serial.print(imuData.un.geoMagRotationVector.i);
  Serial.print(",");
  Serial.print(imuData.un.geoMagRotationVector.j);
  Serial.print(",");
  Serial.print(imuData.un.geoMagRotationVector.k);
  Serial.print(",");
  Serial.print(imuData.un.geoMagRotationVector.real);
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
  Serial.print(barometer.pressure);
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