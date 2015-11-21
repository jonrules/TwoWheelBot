/* Include gyro libraries */
#include <Wire.h>
#include <L3G.h>

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

/* Include math libraries */
#include <math.h>


/* Gyro */
L3G gyro;
int maxmDps = 100;
int gyroSensitivity = 500;
float gyroScale = 8.75;
float gyroMax = (float)maxmDps/gyroScale;
int gyroXDirection = 1;
int gyroYDirection = -1;
int gyroZDirection = -1;

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
  maestroSerial.begin(9600);

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
}

int calculateRotationOffset(int gyroValue) {
  int servoOffset = gyroValue*(servoMax - servoMin)/gyroMax;
  return servoOffset;
}

void loop() {
  gyro.read();
  

  int leftServoPosition = servoZero;
  int rightServoPosition = servoZero;

  if (gyro.g.y > gyroSensitivity) 
  {
    int yRotationOffset = calculateRotationOffset((int)gyro.g.y);
    leftServoPosition += gyroYDirection*yRotationOffset;
    rightServoPosition -= gyroYDirection*yRotationOffset;
    Serial.print("Gyro.y:");
    Serial.print((int)gyro.g.y);
    Serial.print(", Left:");
    Serial.print(leftServoPosition);
    Serial.print(",Right:");
    Serial.println(rightServoPosition);
  }

  maestro.setTarget(leftServo, leftServoPosition);
  maestro.setTarget(rightServo, rightServoPosition);
}

