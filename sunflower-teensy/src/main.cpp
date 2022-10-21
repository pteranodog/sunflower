// -------- HEADERS ---------
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <InternalTemperature.h>
// --------------------------
 

// ------ FLIGHT STATE ------
// Flight state definitions and initialization
enum FlightState {TEST, LAUNCH, ASCENT, STABILIZATION, DESCENT, LANDING, LANDED};
FlightState currentState = TEST;
// --------------------------

enum CamState {CAM_OFF, CAM_ON, CAM_RISING, CAM_FALLING};
CamState camState = CAM_OFF;

// -- CONTROL CONSTANTS --
const float pidLookup[][3] = {
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {-.9, 0.0, .3},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}
};

const float Kp = 0.6;
const float Ki = 0;
const float Kd = .4;
const float seekDeadzone = 0.4;
const float seekDeadspeed = 0.1;
const float stableDeadzone = 0.6;
const float stableDeadspeed = 1;
float currentDeadzone = stableDeadzone;
float currentDeadspeed = stableDeadspeed;
float controlOut = 0;
float thresholdUpper = 0.1;
float thresholdLower = 0.1;
bool solenoidCW = false;
bool solenoidCCW = false;
int cumulativeSolenoidTime = 0;
int maxSolenoidTime = 30 * 1000;
float currentThrust = 0;
// ------------------------

// ---- PIN NUMBER SETUP ----
// Serial1 pin
const int Serial1_RX = 21;
const int Serial1_TX = 1;
// Photoresistor selection pins
const int PHOTORESISTOR_0 = 3;
const int PHOTORESISTOR_1 = 4;
const int PHOTORESISTOR_2 = 5;
const int PHOTORESISTOR_3 = 6;
const int PHOTORESISTOR_4 = 7;
const int PHOTORESISTOR_5 = 8;
const int PHOTORESISTOR_6 = 9;
// Solenoid control pins
const int SOLENOID_CW = 22;
const int SOLENOID_CCW = 23;
// Camera pin
const int CAMERA = 11;
// LED pins
const int BOX_LED = 0;
const int BOARD_LED = 13;
// Eye readout pins
const int EYE_FRONT = 17;
const int EYE_RIGHT = 16;
const int EYE_LEFT = 15;
// I2C pins
const int I2C_SDA = 18;
const int I2C_SCL = 19;
const int SOLAR_PANEL = 20;
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
  float x;
  float y;
  float z;
  float relativeSunAngle;
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
double groundpressure;
bool ledOn = false;

// ------ SENSOR SETUP ------
// IMU sensor setup
Adafruit_BNO08x imu = Adafruit_BNO08x();
sh2_SensorValue_t imuData;
// // Barometer sensor setup
Adafruit_BMP3XX barometer = Adafruit_BMP3XX();
// --------------------------


// -- FUNCTION DECLARATION --
void checkCamera();
void getSensorData();
FlightState updateState();
void stabilize();
void blinkLED(int LEDPin, unsigned int milliseconds);
void writeTelemetry();
void parseIMUData();
void updateSPS();
float calculateSunAngle();
void quaternionToEuler();
void applyControl(float control);
float calculateThrust();
int largestPhotoresistor(Eye eye);
float smoothSunAngle(float newSunAngle);
// --------------------------


// -- MAIN SETUP FUNCTION --
void setup() {
  // Start Serial1 output
  Serial1.setRX(Serial1_RX);
  Serial1.setTX(Serial1_TX);
  Serial1.begin(115200);

  // Pin mode setup!
  pinMode(BOARD_LED, OUTPUT);
  pinMode(BOX_LED, OUTPUT);
  pinMode(CAMERA, OUTPUT);
  digitalWrite(CAMERA, HIGH);
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
  pinMode(SOLAR_PANEL, INPUT);

  // Start I2C
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  if (!imu.begin_I2C()) {
    while(true) {
      blinkLED(BOX_LED, 500);
      delay(50);
    }
  }
  if (!barometer.begin_I2C()) {
    while(true) {
      blinkLED(BOX_LED, 250);
      delay(50);
    }
  }
  // Enable IMU Reports
  imu.enableReport(SH2_ACCELEROMETER);
  imu.enableReport(SH2_GYROSCOPE_CALIBRATED);
  imu.enableReport(SH2_ROTATION_VECTOR);
  
  // I don't know what this does but it was in the example
  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);

  for (int i = 0; i < 10; i++) {
    barometer.readPressure();
    delay(50);
  }
  for (int i = 0; i < 10; i++) {
    groundpressure += barometer.readPressure();
    delay(50);
  }
  groundpressure /= 10.0;
  groundpressure /= 100.0;

  digitalWrite(SOLENOID_CW, HIGH);
  delay(100);
  digitalWrite(SOLENOID_CW, LOW);
  delay(1000);
  digitalWrite(SOLENOID_CCW, HIGH);
  delay(100);
  digitalWrite(SOLENOID_CCW, LOW);
  delay(1000);
}
// --------------------------


// --- MAIN LOOP FUNCTION ---
void loop() {
  checkCamera();
  getSensorData();
  if (currentState == TEST) {
    blinkLED(BOX_LED, 3000);
  } else {
    blinkLED(BOX_LED, 1000);
  }
  if (currentState == STABILIZATION || currentState == TEST) {
    stabilize();
  }
  writeTelemetry();
}
// --------------------------

// -- FUNCTION DEFINITIONS --

void checkCamera() {
  static unsigned int cameraInterval = 0.1 * 60 * 1000;
  static unsigned int lastChange = 10 * 1000;
  switch (camState) {
    case CAM_ON:
      if (millis() - lastChange > cameraInterval) {
        digitalWrite(CAMERA, LOW);
        camState = CAM_FALLING;
        lastChange = millis();
      }
      break;
    case CAM_FALLING:
      if (millis() - lastChange > 700) {
        digitalWrite(CAMERA, HIGH);
        camState = CAM_OFF;
        lastChange = millis();
      }
      break;
    case CAM_OFF:
      if (currentState == LANDED) {
        return;
      }
      if (millis() - lastChange > 500) {
        digitalWrite(CAMERA, LOW);
        camState = CAM_RISING;
        lastChange = millis();
      }
      break;
    case CAM_RISING:
      if (millis() - lastChange > 700) {
        digitalWrite(CAMERA, HIGH);
        camState = CAM_ON;
        lastChange = millis();
      }
      break;
  }
}

void getSensorData() {
  // Get sensor data
  parseIMUData();
  barometer.performReading();
  temp = barometer.temperature;
  pressure = barometer.pressure;
  altitude = barometer.readAltitude(groundpressure);
  updateSPS();
  sunAngle = smoothSunAngle(calculateSunAngle());
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
          quaternionToEuler();
          updated.rotVec = true;
          break;
      }
    }
  }
}

void quaternionToEuler() {
    float sqr = sq(rotVec.real);
    float sqj = sq(rotVec.j);
    float sqk = sq(rotVec.k);
    rotVec.x = atan2(2.0 * (rotVec.i * rotVec.j + rotVec.k * rotVec.real), 1.0 - 2.0 * (sqj + sqk)); 
    rotVec.y = asin(2.0 * (rotVec.i * rotVec.k - rotVec.j * rotVec.real));
    rotVec.z = atan2(2.0 * (rotVec.i * rotVec.real + rotVec.j * rotVec.k), 1.0 - 2.0 * (sqk + sqr));
    while (rotVec.x < sunAngle - PI) {
      rotVec.x += 2 * PI;
    }
    while (rotVec.x > sunAngle + PI) {
      rotVec.x -= 2 * PI;
    }
}

FlightState updateState() {
  switch (currentState) {
    case LAUNCH:
      if (altitude > 3000) {
        return ASCENT;
      }
    case TEST:
      break;
    case ASCENT:
      if (altitude > 20000) {
        return STABILIZATION;
      }
      break;
    case STABILIZATION:
      if (altitude < 3000) {
        return LANDING;
      }
      if (abs(accel.x) < .03 && abs(accel.y) < .03 && abs(accel.z) < .03) {
        return DESCENT;
      }
    case DESCENT:
      if (altitude < 3000) {
        return LANDING;
      }
      break;
    case LANDING:
      float mag = sqrt(sq(accel.x) + sq(accel.y) + sq(accel.z));
      if (mag > 9.3 && mag < 10.5) {
        return LANDED;
      }
      break;
    case LANDED:
      break;
  }  if (altitude > 20000) {
    return STABILIZATION;
  }
  return currentState;
}

FlightState nextState(FlightState next_state) {
  static FlightState state_to_change = ASCENT;
  unsigned static int stateChangeTime = 0;
  if (currentState == TEST) {
    return TEST;
  }
  if (currentState == LANDED) {
    return LANDED;
  }
  if (next_state == currentState) {
    state_to_change = currentState;
    return currentState;
  }
  if (next_state == state_to_change) {
    if (millis() > stateChangeTime) {
      state_to_change = next_state;
      return next_state;
    } else {
      return currentState;
    }
  } else {
    state_to_change = next_state;
    stateChangeTime = millis() + 5000;
    return currentState;
  }
}

void stabilize() {
  if (cumulativeSolenoidTime > maxSolenoidTime) {
    return;
  }
  if (abs(sunAngle - rotVec.x) < currentDeadzone && abs(gyro.x) < currentDeadspeed) {
    currentDeadzone = stableDeadzone;
    currentDeadspeed = stableDeadspeed;
    applyControl(0);
    controlOut = 0;
    return;
  }
  currentDeadzone = seekDeadzone;
  currentDeadspeed = seekDeadspeed;
  static float integral = 0;
  float proportional = (sunAngle - rotVec.x) * Kp;
  integral += (sunAngle - rotVec.x) * Ki;
  integral *= 0.97;
  float derivative = gyro.z * Kd;
  float control = proportional + integral + derivative;
  applyControl(control);
  controlOut = control;
  currentThrust = calculateThrust();
}

void blinkLED(int LEDPin, unsigned int milliseconds) {
  static unsigned int millisAtFirstBlink = millis();
  if (millis() - millisAtFirstBlink > milliseconds) {
    digitalWrite(LEDPin, HIGH);
    millisAtFirstBlink = millis();
    ledOn = true;
  } else {
    digitalWrite(LEDPin, LOW);
    ledOn = false;
  }
}

void writeTelemetry() {
  Serial1.print("1,");
  Serial1.print(millis());
  Serial1.print(",");
  Serial1.print(packets++);
  Serial1.print(",");
  Serial1.print(currentState);
  Serial1.print(",");
  Serial1.print(camState);
  Serial1.print(",");
  Serial1.print(altitude);
  Serial1.print(",");
  Serial1.print(temp);
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
  Serial1.print(rotVec.x);
  Serial1.print(",");
  Serial1.print(rotVec.y);
  Serial1.print(",");
  Serial1.print(rotVec.z);
  Serial1.print(",");
  if (ledOn) {
    Serial1.print("-1,-1,-1,-1");
  } else {
    Serial1.print(SPSRow);
    Serial1.print(",");
    Serial1.print(SPS.front.array[SPSRow]);
    Serial1.print(",");
    Serial1.print(SPS.left.array[SPSRow]);
    Serial1.print(",");
    Serial1.print(SPS.right.array[SPSRow]);
  }
  Serial1.print(",");
  Serial1.print(sunAngle);
  Serial1.print(",");
  Serial1.print(pressure);
  Serial1.print(",");
  Serial1.print(InternalTemperature.readTemperatureC());
  Serial1.print(",");
  Serial1.print(controlOut);
  Serial1.print(",");
  Serial1.print(currentDeadzone);
  Serial1.print(",");
  Serial1.print(currentDeadspeed);
  Serial1.print(",");
  Serial1.print(solenoidCW);
  Serial1.print(",");
  Serial1.print(solenoidCCW);
  Serial1.print(",");
  Serial1.print(currentThrust);
  Serial1.print(",");
  Serial1.print(cumulativeSolenoidTime);
  Serial1.print(",");
  Serial1.print(analogRead(SOLAR_PANEL));
  Serial1.println("");
}
// --------------------------

void updateSPS() {
  if (ledOn) {
    return;
  }
  SPS.front.array[SPSRow] = analogRead(EYE_FRONT);
  SPS.left.array[SPSRow] = analogRead(EYE_LEFT);
  SPS.right.array[SPSRow] = analogRead(EYE_RIGHT);
  digitalWrite(PHOTORESISTOR_0 + SPSRow, LOW);
  SPSRow = (SPSRow + 1) % 7;
  digitalWrite(PHOTORESISTOR_0 + SPSRow, HIGH);
}

float calculateSunAngle() {
  float newSunAngle;
  int frontLargest = largestPhotoresistor(SPS.front);
  int leftLargest = largestPhotoresistor(SPS.left);
  int rightLargest = largestPhotoresistor(SPS.right);
  if (leftLargest == 6 && rightLargest == 0) {
    return rotVec.x;
  }
  if (SPS.front.array[frontLargest] > SPS.left.array[leftLargest] && SPS.front.array[frontLargest] > SPS.right.array[rightLargest]) {
    newSunAngle = .3285 * frontLargest + 2.2997 + rotVec.x;
  } else if (SPS.left.array[leftLargest] > SPS.front.array[frontLargest] && SPS.left.array[leftLargest] > SPS.right.array[rightLargest]) {
    newSunAngle = .3285 * leftLargest - 2.2997 + rotVec.x;
  } else if (SPS.right.array[rightLargest] > SPS.front.array[frontLargest] && SPS.right.array[rightLargest] > SPS.left.array[leftLargest]) {
    newSunAngle = .3285 * rightLargest + rotVec.x;
  } else {
    newSunAngle = sunAngle;
  }
  while (abs(newSunAngle) > PI) {
    if (newSunAngle > PI) {
      newSunAngle -= 2 * PI;
    } else if (newSunAngle < -PI) {
      newSunAngle += 2 * PI;
    }
  }
  return newSunAngle;
}

float smoothSunAngle(float newSunAngle) {
  float smoothedSunAngle = newSunAngle;
  if (abs(newSunAngle - sunAngle) > 0.1 && abs(newSunAngle - sunAngle) < PI) {
    smoothedSunAngle = sunAngle + 0.1 * (newSunAngle - sunAngle);
  }
  return smoothedSunAngle;
}

int largestPhotoresistor(Eye eye) {
  float largest = 0;
  int indexOfLargest = 0;
  for (int i = 0; i < 7; i++) {
    if (eye.array[i] > largest) {
      largest = eye.array[i];
      indexOfLargest = i;
    }
  }
  return indexOfLargest;
}

void applyControl(float control) {
  static int solenoidStartTime = 0;
  static unsigned int solenoidCooldown = 0;
  if (millis() < solenoidCooldown) {
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, LOW);
    solenoidCW = false;
    solenoidCCW = false;
    return;
  }
  if (abs(control) > thresholdUpper && !solenoidCW && !solenoidCCW) {
    if (control > 0) {
      digitalWrite(SOLENOID_CW, HIGH);
      solenoidCW = true;
      solenoidStartTime = millis();
    } else {
      digitalWrite(SOLENOID_CCW, HIGH);
      solenoidCCW = true;
      solenoidStartTime = millis();
    }
  }
  if (solenoidCW && control < thresholdLower) {
    digitalWrite(SOLENOID_CW, LOW);
    solenoidCW = false;
    cumulativeSolenoidTime += millis() - solenoidStartTime;
  }
  if (solenoidCCW && control > -thresholdLower) {
    digitalWrite(SOLENOID_CCW, LOW);
    solenoidCCW = false;
    cumulativeSolenoidTime += millis() - solenoidStartTime;
  }
  if ((solenoidCCW || solenoidCW) && millis() - solenoidStartTime > 100) {
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, LOW);
    solenoidCW = false;
    solenoidCCW = false;
    cumulativeSolenoidTime += millis() - solenoidStartTime;
    solenoidCooldown = 100 + millis();
  } 
}

float calculateThrust() {
  static float previousVel = 0;
  static float previousTime = millis();
  static float solenoidWasOn = false;
  
  if (solenoidWasOn) {
    // Return average of current thrust and new calculated thrust (for smoothness)
    return (currentThrust + (abs(gyro.z - previousVel) / (millis() - previousTime))) / 2;
  }

  previousTime = millis();
  previousVel = gyro.z;
  solenoidWasOn = solenoidCCW || solenoidCW;
  return 0;
}
