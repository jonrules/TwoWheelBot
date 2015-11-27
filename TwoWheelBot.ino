/* Include A-Star libraries */
#include <AStar32U4.h>

/* Include sensor libraries */
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

/* Include pololu maestro libraries */
#include <PololuMaestro.h>
/* On boards with a hardware serial port available for use, use
  that port to communicate with the Maestro. For other boards,
  create a SoftwareSerial object using pin 10 to receive (RX) and
  pin 11 to transmit (TX). */
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 11);
#endif

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
PidController<long>  pidController(targetValue, 10, 0x07);

/* lsm */
LSM303 lsm;
AxisValues accelerometerNeutral;
AxisValues accelerometerPhase = { 0, 0, 0 };
LongAxisValues offsetAngle;

/* Maestro servo controller */
MiniMaestro maestro(maestroSerial);
int leftServo = 0;
int rightServo = 1;
int leftServoTrim = 0;
int rightServoTrim = 300;
int servoMin = 4000;
int servoMax = 8000;
int servoNeutral = 6000;
int servoOffsetHigh = 2000;
int servoOffsetLow = 1500;
int servoAngleTolerance = 1;


long calculateAccelerometerAngleOffset(int value, int neutralValue) {
  long offset = (long)(value - neutralValue)*90;
  return offset;
}

int calculateServoOffset(long angle) {
  int offset = angle/10;
  return offset;
}

int getSafeServoPosition(int servoPosition) {
  if (servoPosition > servoMax) {
    return servoMax;
  }
  if (servoPosition < servoMin) {
    return servoMin;
  }
  return servoPosition;
}

void defaultCalibration() {
  maestro.setTarget(leftServo, servoNeutral);
  maestro.setTarget(rightServo, servoNeutral);

  lsm.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  lsm.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  accelerometerNeutral.x = -40;
  accelerometerNeutral.y = -80;
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
  maestro.setTarget(leftServo, servoNeutral);
  maestro.setTarget(rightServo, servoNeutral);
  
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

  maestroSerial.begin(9600);

  defaultCalibration();
  //calibrate();

  // PID Controller
  pidController.setProportionalGain(0.5);
  pidController.setIntegralGain(0.01);
  pidController.setDerivativeGain(50.0);
}

void loop() {
  lsm.read();

  int leftServoPosition = servoNeutral;
  int rightServoPosition = servoNeutral;

  offsetAngle.x = calculateAccelerometerAngleOffset((int)lsm.a.x >> 4, accelerometerNeutral.x);
  offsetAngle.y = calculateAccelerometerAngleOffset((int)lsm.a.y >> 4, accelerometerNeutral.y);
  offsetAngle.z = calculateAccelerometerAngleOffset((int)lsm.a.z >> 4, accelerometerNeutral.z);

  long result = pidController.calculate(offsetAngle.y/10);
  int servoOffset = (int)result;
  leftServoPosition -= servoOffset;
  rightServoPosition += servoOffset;
  maestro.setTarget(leftServo, getSafeServoPosition(leftServoPosition + leftServoTrim));
  maestro.setTarget(rightServo, getSafeServoPosition(rightServoPosition + rightServoTrim));

  Serial.print("X:");
  Serial.print(offsetAngle.x/1000);
  Serial.print(", Y:");
  Serial.print(offsetAngle.y/1000);
  Serial.print(", Z: ");
  Serial.print(offsetAngle.z/1000);
  Serial.print(", Result:");
  Serial.print(result);
  Serial.print(", Left:");
  Serial.print(leftServoPosition);
  Serial.print(", Right:");
  Serial.println(rightServoPosition);
}

