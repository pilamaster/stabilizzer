#include "CurieIMU.h"
#include "Kalman.h"
#include <Servo.h>

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;
//raw data from IMU
int ax, ay, az;
int gx, gy, gz;
//angles from accelerometer
double accXangle;
double accYangle;
double accZangle;
//rate from gyros
double gyroXrate;
double gyroYrate;
double gyroZrate;
//angles from gyros
double gyroXangle = 90;
double gyroYangle = 90;
double gyroZangle = 90;
//angles from complementary filter
double compAngleX = 90;
double compAngleY = 90;
double compAngleZ = 90;
//angles from Kalman filter
double kalAngleX;
double kalAngleY;
double kalAngleZ;
//timer
uint32_t timer;
//Servo initialisation
int portServoUp = 3;
int portServoDown = 4;
Servo servoUp;
Servo servoDown;


int calibrateOffsets = 1;

void setup()
{
  servoUp.attach(portServoUp);
  servoDown.attach(portServoDown);
  //Initial servo position, correct if neccessary
  servoUp.write(90 + 0);
  servoDown.write(90 + 0);

  Serial.begin(9600);
  while (!Serial);    // wait for the serial port to open
  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();
  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.begin())
  {
    Serial.println("CurieIMU connection successful");
  }
  else
  {
    Serial.println("CurieIMU connection failed");
  }
  autoCalibrationIMU();
}

void loop()
{
  //reading raw data from IMU
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
  //calculating angles from accelerometer
  accXangle = (atan2(ay,az)+PI)*RAD_TO_DEG;
  accYangle = (atan2(ax,az)+PI)*RAD_TO_DEG;
  accZangle = (atan2(ay,ax)+PI)*RAD_TO_DEG;
  //rate calculation from gyros
  gyroXrate = (double)gx/131.0;
  gyroYrate = -((double)gy/131.0);
  gyroZrate = ((double)gz/131.0);

  //angle calculation from gyros
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000);
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  gyroZangle += gyroZrate*((double)(micros()-timer)/1000000);

  //complementary filter
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle);
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
  compAngleZ = (0.93*(compAngleZ+(gyroZrate*(double)(micros()-timer)/1000000)))+(0.07*accZangle);

  //Kalman filter
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
  kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (double)(micros()-timer)/1000000);
  //writing angles to servos
  servoUp.write(kalAngleX);
  servoDown.write(kalAngleY);
  //time calculation
  timer = micros();



}

void autoCalibrationIMU()
{
  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,495.3);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-15.6);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,491.4);
    //CurieIMU.setGyroOffset(X_AXIS,7.869);
    //CurieIMU.setGyroOffset(Y_AXIS,-0.061);
    //CurieIMU.setGyroOffset(Z_AXIS,15.494);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }

}
