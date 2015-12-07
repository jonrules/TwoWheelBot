/* Include A-Star libraries */
#include <AStar32U4.h>

/* Include sensor libraries */
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

/* Include motor driver libraries */
#include "DualVNH5019MotorShield.h"

/* Include utils libraries */
#include <GyroAngleFilter.h>
#include <PidController.h>
#include <math.h>


/* Data structures */
struct AxisValues {
  int x;
  int y;
  int z;
};

struct LongAxisValues {
  long x;
  long y;
  long z;
};

/* PID Controller */
long targetValue = 0l;
unsigned char terms = PidController<long>::TERM_PROPORTIONAL | PidController<long>::TERM_INTEGRAL | PidController<long>::TERM_DERIVATIVE;
PidController<long>  pidController(targetValue, 10, 0x07, 0);

/* lsm */
LSM303 lsm;
AxisValues accelerometerNeutral;
AxisValues accelerometerPhase = { 0, 0, 0 };
LongAxisValues accelerometerAngle;

/* Gyro */
L3G gyro;
AxisValues gyroNeutral;
LongAxisValues gyroVelocity;
// Sensitivity in units of mdps/LSB
float gyroSensitivity = 8.75;
// Gyro angle filter
GyroAngleFilter<long> gyroAngleFilter;

/* Motor driver */
DualVNH5019MotorShield motorDriver(A1, A2, A0, PB7, A4, A5, A3, PD6);
int motorDriverMin = -400;
int motorDriverMax = 400;
int motorDriverNeutral = 0;
int motorDirection = -1;

void stopIfMotorFault()
{
  if (motorDriver.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (motorDriver.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  motorDriver.setM1Speed(rightSpeed*motorDirection);
  motorDriver.setM2Speed(leftSpeed*motorDirection);
  stopIfMotorFault();
}

long calculateGyroVelocity(int value, int neutralValue) {
  long velocity = (long)(value - neutralValue)*gyroSensitivity/1000;
  return velocity;
}

long calculateAccelerometerAngleOffset(int value, int neutralValue) {
  long offset = (long)(value - neutralValue)*90;
  return offset;
}

void defaultCalibration() {
  gyroNeutral.x = -904;
  gyroNeutral.y = 471;
  gyroNeutral.z = -216;
  
  lsm.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  lsm.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  accelerometerNeutral.x = -40;
  accelerometerNeutral.y = 0;
  accelerometerNeutral.z = -950;

  ledRed(1);
  delay(1000);
  ledRed(0);
  delay(1000);
  ledRed(1);
  delay(1000);
  ledRed(0);
}

void calibrate() {
  /* Calibrate gyro */
  gyro.read();
  gyroNeutral.x = (int)gyro.g.x;
  gyroNeutral.y = (int)gyro.g.y;
  gyroNeutral.z = (int)gyro.g.z;
  
  ledRed(1);
  delay(1000);
  ledYellow(1);
  delay(1000);
  ledRed(0);
  ledYellow(0);
  delay(2000);
  ledYellow(1);
  ledRed(1);
  delay(2000);

  /* Calibarate lsm/accelerometer */
  lsm.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  lsm.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  lsm.read();
  accelerometerNeutral.x = lsm.a.x >> 4;
  accelerometerNeutral.y = lsm.a.y >> 4;
  accelerometerNeutral.z = lsm.a.z >> 4;
  Serial.print("Accelerometer Neutral: (");
  Serial.print(accelerometerNeutral.x);
  Serial.print(", ");
  Serial.print(accelerometerNeutral.y);
  Serial.print(", ");
  Serial.print(accelerometerNeutral.z);
  Serial.println(")");

  ledGreen(1);
  delay(1000);
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  delay(1000);
  ledRed(1);
  ledYellow(1);
  ledGreen(1);
  delay(1000);
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  lsm.init();
  lsm.enableDefault();

  if (!gyro.init(L3G::deviceType::device_D20H, L3G::sa0State::sa0_high))
  {
    delay(2000);
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();

  motorDriver.init();

  defaultCalibration();
  //calibrate();

  // Gyro angle filter
  gyroAngleFilter.setCorrectionFactor(0.1);
  gyroAngleFilter.setSampleTime(10);
  gyroAngleFilter.init(0);

  // PID Controller
  pidController.setProportionalGain(0.001);
  pidController.setIntegralGain(0.0003);
  pidController.setDerivativeGain(0.15);
}

void loop() {
  gyro.read();
  lsm.read();

  int leftSpeed = motorDriverNeutral;
  int rightSpeed = motorDriverNeutral;

  gyroVelocity.x = calculateGyroVelocity((int)gyro.g.x, gyroNeutral.x);
  gyroVelocity.y = calculateGyroVelocity((int)gyro.g.y, gyroNeutral.y);
  gyroVelocity.z = calculateGyroVelocity((int)gyro.g.z, gyroNeutral.z);

  accelerometerAngle.x = calculateAccelerometerAngleOffset((int)lsm.a.x >> 4, accelerometerNeutral.x);
  accelerometerAngle.y = calculateAccelerometerAngleOffset((int)lsm.a.y >> 4, accelerometerNeutral.y);
  accelerometerAngle.z = calculateAccelerometerAngleOffset((int)lsm.a.z >> 4, accelerometerNeutral.z);

  long result = pidController.calculate(gyroAngleFilter.calculate(gyroVelocity.z, -accelerometerAngle.y));
  //long result = gyroAngleFilter.calculate(gyroVelocity.z, -accelerometerAngle.y);
  int motorOffset = (int)result;
  leftSpeed += motorOffset;
  rightSpeed += motorOffset;
  setMotorSpeeds(leftSpeed, rightSpeed);

  Serial.print("X:");
  Serial.print(accelerometerAngle.x/1000);
  Serial.print(", Y:");
  Serial.print(accelerometerAngle.y/1000);
  Serial.print(", Z:");
  Serial.print(accelerometerAngle.z/1000);
  Serial.print(", Result:");
  Serial.print(result);
  Serial.print(", Left:");
  Serial.print(leftSpeed);
  Serial.print(", Right:");
  Serial.println(rightSpeed);
}

