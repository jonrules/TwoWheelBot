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
}

/* Timers */
unsigned long time;
int dt = 100;

/* Compass */
LSM303 compass;
float accelerometerScale = 0.5;
int accelerometerXSensitivity = 50;
int accelerometerXDirection = -1;
int accelerometerXNeutral;
int accelerometerYSensitivity = 50;
int accelerometerYDirection = -1;
int accelerometerYNeutral;
int accelerometerZSensitivity = 100;
int accelerometerZDirection = 1;
int accelerometerZNeutral;

/* Gyro */
L3G gyro;
int gyroSensitivity = 700;
float gyroScale = 1;
int gyroMax = 20000;
int gyroXDirection = 1;
int gyroYDirection = 1;
int gyroZDirection = -1;
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

int calculateRotationOffset(int value) {
  //int offset = (int)(gyroScale*value);
  int offset = gyroMax*exp((float)value/2);
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

  ledYellow(1);
  ledGreen(1);
  delay(1000);
  ledRed(0);
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
  if (abs(compass.a.z) > accelerometerZSensitivity)
  {
    zAccelerometerOffset = calculateAccelerometerOffset((int)compass.a.z, accelerometerZNeutral);
    leftServoPosition += accelerometerZDirection * zAccelerometerOffset;
    rightServoPosition -= accelerometerZDirection * zAccelerometerOffset;
  }

  int yRotationOffset;
  if (abs(gyro.g.y) > gyroSensitivity)
  {
    yRotationOffset = calculateRotationOffset((int)gyro.g.y);
    leftServoPosition += gyroYDirection * yRotationOffset;
    rightServoPosition -= gyroYDirection * yRotationOffset;
  }

  Serial.print(", Compass.a.z:");
  Serial.print((int)compass.a.z);
  Serial.print("Gyro.g.y:");
  Serial.print((int)gyro.g.y);
  Serial.print(", Left:");
  Serial.print(leftServoPosition);
  Serial.print(",Right:");
  Serial.println(rightServoPosition);

  maestro.setTarget(leftServo, getSafeServoPosition(leftServoPosition));
  maestro.setTarget(rightServo, getSafeServoPosition(rightServoPosition));
}

