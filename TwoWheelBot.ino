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
float gyroMax = (float)maxmDps/gyroScale;
int gyroXDirection = 1;
int gyroYDirection = -1;
int gyroZDirection = -1;

/* Compass */
LSM303 compass;
int accelerometerYSensitivity = 100;
int accelerometerYDirection = -1;

/* Maestro servo controller */
MiniMaestro maestro(maestroSerial);
int leftServo = 0;
int rightServo = 1;
int servoMin = 4000;
int servoMax = 8000;
int servoZero = (int)((servoMax + servoMin)/2);


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
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  maestroSerial.begin(9600);
}

int calculateRotationOffset(int gyroValue) {
  int servoOffset = gyroValue*(servoMax - servoMin)/gyroMax;
  return servoOffset;
}

int calculateAccelerometerOffset(int accelerometerValue) {
  return accelerometerValue;
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

void loop() {
  gyro.read();
  compass.read();
  
  int leftServoPosition = servoZero;
  int rightServoPosition = servoZero;

  if (abs(gyro.g.y) > gyroSensitivity) 
  {
    int yRotationOffset = calculateRotationOffset((int)gyro.g.y);
    leftServoPosition += gyroYDirection*yRotationOffset;
    rightServoPosition -= gyroYDirection*yRotationOffset;
  }

  if (abs(compass.a.y) > accelerometerYSensitivity)
  {
    int yAccelerometerOffset = calculateAccelerometerOffset((int)compass.a.y);
    leftServoPosition += accelerometerYDirection*yAccelerometerOffset;
    rightServoPosition -= accelerometerYDirection*yAccelerometerOffset;
  }

  Serial.print("Gyro.g.y:");
  Serial.print((int)gyro.g.y);
  Serial.print(", Compass.a.y:");
  Serial.print((int)compass.a.y);
  Serial.print(", Left:");
  Serial.print(leftServoPosition);
  Serial.print(",Right:");
  Serial.println(rightServoPosition);

  maestro.setTarget(leftServo, getSafeServoPosition(leftServoPosition));
  maestro.setTarget(rightServo, getSafeServoPosition(rightServoPosition));
}

