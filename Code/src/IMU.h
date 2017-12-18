#ifndef IMU_H_
#define IMU_H_

#include <Wire.h>
#include <SPI.h>
#include "lib/SparkFunLSM9DS1.h"

class IMU {
  const float DECLINATION = -8.58;
  const char LSM9DS1_M =	0x1E; // Would be 0x1C if SDO_M is LOW
  const char LSM9DS1_AG =	0x6B; // Would be 0x6A if SDO_AG is LOW

  LSM9DS1 imu;
  boolean initialized = false;
private:


public:
  bool initialize() {
      imu.settings.device.commInterface = IMU_MODE_I2C;
      imu.settings.device.mAddress = LSM9DS1_M;
      imu.settings.device.agAddress = LSM9DS1_AG;
      if(!imu.begin()){
        Serial.println("Failed to communicate with LSM9DS1.");
        Serial.println("Double-check wiring.");
        Serial.println("Default settings in this sketch will " \
                      "work for an out of the box LSM9DS1 " \
                      "Breakout, but may need to be modified " \
                      "if the board jumpers are.");
        initialized = false;
      } else {
          // TODO: anything else we need to do to initialize the imu.
          initialized = true;
      }
      return initialized;
  }

  void readAccel(){
    imu.readAccel();
  }
  /**
   *  Prints the current Gyro state.
   */
  void printGyro() {
    if(!imu.gyroAvailable())
      return;
    // Serial.printf("G: %f (%f), %f (%f), %f (%f) deg/s",
    //           imu.calcGyro(imu.gx), imu.gx,
    //           imu.calcGyro(imu.gy), imu.gx,
    //           imu.calcGyro(imu.gz), imu.gz);
  }

/**
 *  Prints the current Accelleration state.
 */
  void printAccel() {
    if(!imu.accelAvailable())
      return;
Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
//#elif defined PRINT_RAW
//  Serial.print(imu.ax);
//  Serial.print(", ");
//  Serial.print(imu.ay);
//  Serial.print(", ");
//  Serial.println(imu.az);
  }

  /**
   *  Prints the current Magnetic state.
   */
  void printMag() {
    if(!imu.magAvailable())
      return;
    // Serial.printf("M: %f (%f), %f (%f), %f (%f) gause\n",
    //         imu.calcMag(imu.mx), imu.mx,
    //         imu.calcMag(imu.my), imu.my,
    //         imu.calcMag(imu.mz), imu.mz));
  }

  float getTiltXZ(){
    return atan2(imu.ax, imu.az);
  }

  float getTiltYZ() {
    return getRoll();
  }

  float getRoll(){
    return atan2(imu.ay, imu.az);
  }

  float getPitch(){
    return atan2(-imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az));
  }

  float getHeading() {
    //TODO: I think that this could be simplified..
    const float TAU = 2* PI;
    float heading;
    if (imu.my == 0)
      heading = (imu.mx < 0) ? PI : 0;
    else
      heading = atan2(imu.mx, imu.my);

    heading -= DECLINATION * PI / 180;

    if (heading > PI) heading -= TAU;
    else if (heading < -PI) heading += TAU;
    else if (heading < 0) heading += TAU;
    return heading;
  }

  float toDegrees(float rads) {
    return rads *180/PI;
  }

  void printAttitude() {
    float roll = getRoll();
    float pitch = getPitch();
    float heading = getHeading();


    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

    Serial.print("Pitch: ");
    Serial.print(pitch, 2);
    Serial.print("\nRoll:  ");
    Serial.println(roll, 2);
    Serial.print("Heading: "); Serial.println(heading, 2);
  }
};

#endif
