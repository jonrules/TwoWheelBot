/* Include A-Star libraries */
#include <AStar32U4.h>

/* Include sensor libraries */
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

/* Include motor driver libraries */
#include "DualVNH5019MotorShield.h"

/* Include PID Controller */
#include <PidController.h>

/* Include utils libraries */
#include <math.h>


/* Data structures */
struct AxisValues {
  int x;
  int y;
  int z;
};

struct LongAxisValues {
  long x;
  long int y;
  long z;
};

/* PID Controller */
long targetValue = 0l;
unsigned char terms = PidController<long>::TERM_PROPORTIONAL | PidController<long>::TERM_INTEGRAL | PidController<long>::TERM_DERIVATIVE;
PidController<long>  pidController(targetValue, 1, 0x01, 10);

/* lsm */
LSM303 lsm;
AxisValues accelerometerNeutral;
AxisValues accelerometerPhase = { 0, 0, 0 };
LongAxisValues offsetAngle;

/* Motor driver */
DualVNH5019MotorShield motorDriver(A1, A2, A0, PB7, A4, A5, A3, PD6);
int motorDriverMin = -400;
int motorDriverMax = 400;
int motorDriverNeutral = 0;

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
  motorDriver.setM1Speed(rightSpeed);
  stopIfMotorFault();
  motorDriver.setM2Speed(leftSpeed);
  stopIfMotorFault();
}

long calculateAccelerometerAngleOffset(int value, int neutralValue) {
  long offset = (long)(value - neutralValue)*90;
  return offset;
}

void defaultCalibration() {
  lsm.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  lsm.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  accelerometerNeutral.x = -40;
  accelerometerNeutral.y = 10;
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

  motorDriver.init();

  defaultCalibration();
  //calibrate();

  // PID Controller
  pidController.setProportionalGain(3.5);
  pidController.setIntegralGain(0.08);
  pidController.setDerivativeGain(25.0);
}

void loop() {
  lsm.read();

  int leftSpeed = motorDriverNeutral;
  int rightSpeed = motorDriverNeutral;

  offsetAngle.x = calculateAccelerometerAngleOffset((int)lsm.a.x >> 4, accelerometerNeutral.x);
  offsetAngle.y = calculateAccelerometerAngleOffset((int)lsm.a.y >> 4, accelerometerNeutral.y);
  offsetAngle.z = calculateAccelerometerAngleOffset((int)lsm.a.z >> 4, accelerometerNeutral.z);

  long result = pidController.calculate(offsetAngle.y/10);
  int servoOffset = (int)result;
  leftSpeed -= servoOffset;
  rightSpeed += servoOffset;
  setMotorSpeeds(leftSpeed, rightSpeed);

  Serial.print("X:");
  Serial.print(offsetAngle.x/1000);
  Serial.print(", Y:");
  Serial.print(offsetAngle.y/1000);
  Serial.print(", Z: ");
  Serial.print(offsetAngle.z/1000);
  Serial.print(", Result:");
  Serial.print(result);
  Serial.print(", Left:");
  Serial.print(leftSpeed);
  Serial.print(", Right:");
  Serial.println(rightSpeed);
}

