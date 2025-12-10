// Teensy / Arduino
#define TEENSY 13

// I2C Communication
#include <Wire.h>
int8_t slaveAddr = 5;

// BTS Setup
#include <BTS7960.h>
#define maxPWM 50
// Add proper pins for PWM and enable according to board of choice
// Directly add the PWM and enable pins when declaring the object
BTS7960 FW(6, 7);
BTS7960 LW(0, 1);
BTS7960 RW(4, 5);

// Global Velocity Variables
#define sqrt3by2 0.8660254038
#define minus1by2 -0.5000
#define constVector 1
#define arraySize 4  // LX  LY  L2  R2
int8_t buffer = 10;
int8_t receivedData[arraySize] = { 0 };

// Navigation Variables
int16_t wFW = 0, wLW = 0, wRW = 0;
int16_t Vx = 0, Vy = 0;
int16_t VxG = 0, VyG = 0;
int16_t omega = 0;

// //PID
double currentTime = 0;
double previousTime = 0;
double error = 0;
double previousError = 0;
double derivative = 0;
double kp = 8.0;
double kd = 68;
double PID = 0;

// BNO
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define PI 3.1415962

double targetAngle = 0;
double currentAngle = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  pinMode(TEENSY, OUTPUT);
  digitalWrite(TEENSY, HIGH);

  Serial.begin(115200);
  Serial.print("Ganpati Bappa Morya!");

  // Setting-up I communication between ESP32 and Arduino
  Wire2.begin();
  Serial.println("I2C Master Ready!");

  // Setting the enable as HIGH for each BTS
  FW.setEnable(true);
  LW.setEnable(true);
  RW.setEnable(true);

  // Initiating BNO and setting extCrystal as true
  if (!bno.begin()) {
    // Serial.print("No BNO055 detected");
    bno.setExtCrystalUse(true);
    while (1)
      ;
  }
  delay(1000);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentAngle = 0;
  wFW = 0;
  wLW = 0;
  wRW = 0;
  Vx = 0, Vy = 0;
  VxG = 0, VyG = 0;
  omega = 0;

  currentAngle = euler.x();
  // Serial.println(currentAngle);
  float theta = currentAngle * PI / 180.0;

  requestPS4();

  Vy = receivedData[0];  //Y-Component of the Joystick is the X component of the Chassis
  Vx = receivedData[1];
  omega = receivedData[2] - receivedData[3];
  VxG = Vx * cos(-theta) - Vy * sin(-theta);  // Local X
  VyG = Vx * sin(-theta) + Vy * cos(-theta);  // Local Y


  // if (abs(omega) < 10) {
  //   error = currentAngle - targetAngle;
  //   if (error > 180) error -= 360;
  //   if (error < -180) error += 360;

  //   omega = PIDControl(error);

  //   previousError = error;
  //   previousTime = currentTime;
  // } 
  // else {
  //   targetAngle = currentAngle;
  // }

  // Front wheel (120d)
  wFW = constrain(constVector * (VxG*(minus1by2) + VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  // Left Wheel (240d)
  wLW = constrain(constVector * (VxG*(minus1by2) - VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  // Right Wheel (0d)
  wRW = constrain(constVector * (VxG - VyG*(0) + omega), -maxPWM, maxPWM);

  // Sending equation's values to BTS
  FW.rotate(wFW);
  RW.rotate(wRW);
  LW.rotate(wLW);

  // targetAngle = currentAngle;
  // printEq();
  printPS();
  delay(10);
}

void requestPS4() {
  Wire2.requestFrom(slaveAddr, sizeof(receivedData));

  int i = 0;
  while (Wire2.available()) {
    uint8_t raw = Wire2.read();
    if (i == 0) receivedData[0] = map(raw, 0, 255, -127, 127);       // LX
    else if (i == 1) receivedData[1] = map(raw, 0, 255, -127, 127);  // LY
    else if (i == 2) receivedData[2] = map(raw, 0, 255, 0, 127);     // L2
    else if (i == 3) receivedData[3] = map(raw, 0, 255, 0, -127);    // R2

    i++;
  }

  // if (abs(receivedData[0]) < buffer) receivedData[0] = 0;
  // if (abs(receivedData[1]) < buffer) receivedData[1] = 0;
  // if (abs(receivedData[2]) < buffer) receivedData[2] = 0;
  // if (abs(receivedData[3]) < buffer) receivedData[3] = 0;
}


// float PIDControl(int error) {
//   currentTime = millis();
//   int deltaT = (currentTime - previousTime);
//   if (deltaT <= 0) {
//     deltaT = 1;
//   }
//   derivative = (error - previousError) / (deltaT);
//   PID = kp * error + kd * derivative;
//   previousError = error;
//   previousTime = currentTime;
//   PID = constrain(PID, -maxPWM, maxPWM);
//   if (abs(PID) <= 1) {
//     PID = 0;
//   }
//   return PID;
// }

void printPS() {
  Serial.print("LX : ");
  Serial.print(receivedData[0]);
  Serial.print("   ||   LY : ");
  Serial.print(receivedData[1]);
  Serial.print("   ||   L2 : ");
  Serial.print(receivedData[2]);
  Serial.print("   ||   R2 : ");
  Serial.println(receivedData[3]);
}

void printEq() {
  // Serial.print("ANGLE : ");
  // Serial.print(currentAngle);
  Serial.print("   ||   wLW : ");
  Serial.print(wLW);
  Serial.print("   ||   wFW : ");
  Serial.print(wFW);
  Serial.print("   ||   wRW : ");
  Serial.println(wLW);
}