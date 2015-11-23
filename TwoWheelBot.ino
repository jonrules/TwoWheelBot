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

/* Timers */
unsigned long time;
int dt = 100;

/* Compass */
LSM303 compass;
float accelerometerScale = 0.4;
AxisValues accelerometerSensitivity = { 50, 50, 700 };
AxisValues accelerometerDirection = { -1, -1, 1 };
AxisValues accelerometerNeutral;

/* Gyro */
L3G gyro;
int gyroSensitivity = 1;
float gyroScale = 0.3;
int gyroAccelerometerCutoff = 2000;
AxisValues gyroDirection = { 1, 1, -1 };
AxisValues gyroNeutral;
AxisValues gyroValues;

/* Maestro servo controller */
MiniMaestro maestro(maestroSerial);
int leftServo = 0;
int rightServo = 1;
int servoMin = 4000;
int servoMax = 8000;
int servoNeutral = 6000;


int calculateAccelerometerOffset(int value, int neutralValue) {
  int offset = (int)(accelerometerScale*(value - neutralValue));
  return offset;
}

int calculateRotationOffset(int value, int neutralValue) {
  int offset = (int)(gyroScale*(value - neutralValue));
  Serial.println(offset);
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

int calibrate() {
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
  gyroNeutral.x = gyro.g.x;
  gyroNeutral.y = gyro.g.y;
  gyroNeutral.z = gyro.g.z;
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

  /* Calibarate compass/accelerometer */
  compass.read();
  accelerometerNeutral.x = compass.a.x;
  accelerometerNeutral.y = compass.a.y;
  accelerometerNeutral.z = compass.a.z;
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

  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  maestroSerial.begin(9600);

  calibrate();

  // Initial values
  time = millis();
  gyro.read();
  gyroValues.x = (int)gyro.g.x;
  gyroValues.y = (int)gyro.g.y;
  gyroValues.z = (int)gyro.g.z;
}

void loop() {
  compass.read();
  gyro.read();

  int leftServoPosition = servoNeutral;
  int rightServoPosition = servoNeutral;

  int zAccelerometerOffset;
  int zAccelerometerDiff = abs(compass.a.z - accelerometerNeutral.z);
  if (zAccelerometerDiff > accelerometerSensitivity.z)
  {
    zAccelerometerOffset = calculateAccelerometerOffset((int)compass.a.z, accelerometerNeutral.z);
    leftServoPosition += accelerometerDirection.z * zAccelerometerOffset;
    rightServoPosition -= accelerometerDirection.z * zAccelerometerOffset;
  }

  int yRotationOffset;
  if (abs(gyro.g.y - gyroNeutral.y) > gyroSensitivity && zAccelerometerDiff < gyroAccelerometerCutoff)
  {
    yRotationOffset = calculateRotationOffset((int)gyro.g.y, gyroNeutral.y);
    leftServoPosition += gyroDirection.y * yRotationOffset;
    rightServoPosition -= gyroDirection.y * yRotationOffset;
  }

  Serial.print("Compass.a.z:");
  Serial.print((int)compass.a.z);
  Serial.print(", Gyro.g.y:");
  Serial.print((int)gyro.g.y);
  Serial.print(", Left:");
  Serial.print(leftServoPosition);
  Serial.print(",Right:");
  Serial.println(rightServoPosition);

  maestro.setTarget(leftServo, getSafeServoPosition(leftServoPosition));
  maestro.setTarget(rightServo, getSafeServoPosition(rightServoPosition));
}

