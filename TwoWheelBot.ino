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


/* Gyro */
L3G gyro;
int maxmDps = 500;
int gyroSensitivity = 500;
float gyroScale = 8.75;
float gyroMax = (float)maxmDps / gyroScale;
int gyroXDirection = 1;
int gyroYDirection = -1;
int gyroZDirection = -1;

/* Compass */
LSM303 compass;
int accelerometerXSensitivity = 50;
int accelerometerXDirection = -1;
int accelerometerXNeutral;
int accelerometerYSensitivity = 50;
int accelerometerYDirection = -1;
int accelerometerYNeutral;
int accelerometerZSensitivity = 25;
int accelerometerZDirection = 1;
int accelerometerZNeutral;

/* Maestro servo controller */
MiniMaestro maestro(maestroSerial);
int leftServo = 0;
int rightServo = 1;
int servoMin = 4000;
int servoMax = 8000;
int servoNeutral = 6000;


int calculateRotationOffset(int gyroValue) {
  int servoOffset = gyroValue * (servoMax - servoMin) / gyroMax;
  return servoOffset;
}

int calculateAccelerometerOffset(int value, int neutralValue) {
  return (value - neutralValue);
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
  ledRed(1);
  
  maestro.setTarget(leftServo, servoNeutral);
  maestro.setTarget(rightServo, servoNeutral);
  
  delay(3000);
  gyro.read();
  compass.read();

  /* Calibarate accelerometer */
  accelerometerXNeutral = compass.a.x;
  accelerometerYNeutral = compass.a.y;
  accelerometerZNeutral = compass.a.z;
  Serial.print("Accelerometer Neutral: (");
  Serial.print(accelerometerXNeutral);
  Serial.print(", ");
  Serial.print(accelerometerYNeutral);
  Serial.print(", ");
  Serial.print(accelerometerZNeutral);
  Serial.println(")");

  ledRed(0);
  ledYellow(1);
  ledGreen(1);
  delay(1000);
  ledYellow(0);
  ledGreen(0);
  delay(1000);
  ledYellow(1);
  ledGreen(1);
  delay(1000);
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
  else
  {
    delay(2000);
    Serial.println("Gyro initialized!");
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
}

void loop() {
  gyro.read();
  compass.read();

  int leftServoPosition = servoNeutral;
  int rightServoPosition = servoNeutral;

  if (abs(gyro.g.y) > gyroSensitivity)
  {
    //int yRotationOffset = calculateRotationOffset((int)gyro.g.y);
    //leftServoPosition += gyroYDirection * yRotationOffset;
    //rightServoPosition -= gyroYDirection * yRotationOffset;
  }

  if (abs(compass.a.z) > accelerometerZSensitivity)
  {
    int zAccelerometerOffset;
    zAccelerometerOffset = calculateAccelerometerOffset((int)compass.a.z, accelerometerZNeutral);
    leftServoPosition += accelerometerZDirection * zAccelerometerOffset;
    rightServoPosition -= accelerometerZDirection * zAccelerometerOffset;
  }

  Serial.print("Gyro.g.y:");
  Serial.print((int)gyro.g.y);
  Serial.print(", Compass.a.z:");
  Serial.print((int)compass.a.z);
  Serial.print(", Left:");
  Serial.print(leftServoPosition);
  Serial.print(",Right:");
  Serial.println(rightServoPosition);

  maestro.setTarget(leftServo, getSafeServoPosition(leftServoPosition));
  maestro.setTarget(rightServo, getSafeServoPosition(rightServoPosition));
}

