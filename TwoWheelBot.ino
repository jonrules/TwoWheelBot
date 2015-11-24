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

/* Timers */
unsigned long initialTime;

/* lsm */
LSM303 lsm;
AxisValues accelerometerNeutral;
AxisValues accelerometerPhase = { 0, 0, 0 };
int accelerometerSensitivity = 1;
float accelerometerAngleCorrectionFactor = 0.1;

/* Gyro */
L3G gyro;
// Sensitivity in units of mdps/LSB
float gyroSensitivity = 8.75;
// Neutral values in units of LSB (raw reading)
AxisValues gyroNeutral;
// Angle from initial position in units of md/s
LongAxisValues gyroAngle;
int gyroAngleDivisor = 2;

/* Maestro servo controller */
MiniMaestro maestro(maestroSerial);
int leftServo = 0;
int rightServo = 1;
int leftServoTrim = 0;
int rightServoTrim = 100;
int servoMin = 4000;
int servoMax = 8000;
int servoNeutral = 6000;
int servoOffsetHigh = 2000;
int servoOffsetLow = 1000;
int servoAngleTolerance = 2000;


long calculateAngleOffset(int value, int neutralValue, unsigned long dt) {
  long offset = (long)(value - neutralValue)*gyroSensitivity/1000*dt;
  return offset;
}

long calculateAccelerometerAngleOffset(int value, int neutralValue, long angle) {
  long offset = ((long)(value - neutralValue)*90 - angle)*accelerometerAngleCorrectionFactor;
  return offset;
}

int calculateServoOffset(long angle) {
  long absAngle = abs(angle);
  if (absAngle > servoAngleTolerance)
  {
    int offset = (int)(absAngle/angle)*servoOffsetLow;
    return offset;
  }
  return 0;
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

  gyroNeutral.x = -904;
  gyroNeutral.y = 471;
  gyroNeutral.z = -216;

  lsm.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  lsm.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  accelerometerNeutral.x = 1000;
  accelerometerNeutral.y = -50;
  accelerometerNeutral.z = 50;

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
  ledRed(0);
  delay(1000);
  ledRed(1);
  delay(1000);

  /* Calibrate gyro */
  gyro.read();
  gyroNeutral.x = (int)gyro.g.x;
  gyroNeutral.y = (int)gyro.g.y;
  gyroNeutral.z = (int)gyro.g.z;
  Serial.print("Gyro Neutral: (");
  Serial.print(gyroNeutral.x);
  Serial.print(", ");
  Serial.print(gyroNeutral.y);
  Serial.print(", ");
  Serial.print(gyroNeutral.z);
  Serial.println(")");

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

  if (!gyro.init(L3G::deviceType::device_D20H, L3G::sa0State::sa0_high))
  {
    delay(2000);
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();

  lsm.init();
  lsm.enableDefault();

  maestroSerial.begin(9600);

  defaultCalibration();
  //calibrate();

  // Initial values
  initialTime = millis();
}

void loop() {
  gyro.read();
  lsm.read();

  unsigned long currentTime = millis();
  unsigned long dt = currentTime - initialTime;

  int leftServoPosition = servoNeutral;
  int rightServoPosition = servoNeutral;

  //gyroAngle.x += calculateAngleOffset((int)gyro.g.x, gyroNeutral.x, dt);
  gyroAngle.y += calculateAngleOffset((int)gyro.g.y, gyroNeutral.y, dt);
  gyroAngle.y += calculateAccelerometerAngleOffset((int)lsm.a.z >> 4, accelerometerNeutral.z, gyroAngle.y);
  //gyroAngle.z += calculateAngleOffset((int)gyro.g.z, gyroNeutral.z, dt);

  int servoOffset = calculateServoOffset(gyroAngle.y);
  leftServoPosition += servoOffset;
  rightServoPosition -= servoOffset;

  Serial.print("X:");
  Serial.print(gyroAngle.x/1000);
  Serial.print(", Y:");
  Serial.print(gyroAngle.y/1000);
  Serial.print(", Z: ");
  Serial.print(gyroAngle.z/1000);
  Serial.print(", Acc.z:");
  Serial.print((int)lsm.a.z);
  Serial.print(", Left:");
  Serial.print(leftServoPosition);
  Serial.print(", Right:");
  Serial.println(rightServoPosition);

  maestro.setTarget(leftServo, getSafeServoPosition(leftServoPosition + leftServoTrim));
  maestro.setTarget(rightServo, getSafeServoPosition(rightServoPosition + rightServoTrim));

  initialTime = currentTime;
}

