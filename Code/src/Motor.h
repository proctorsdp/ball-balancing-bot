#ifndef _MOTOR_H
#define _MOTOR_H

#include <wire.h>
#include "lib/Adafruit_MotorShield.h"
#include "lib/Adafruit_MS_PWMServoDriver.h"

class Motor {
private:
  Adafruit_MotorShield* driver;
  Adafruit_DCMotor *motorx1, *motorx2, *motory1, *motory2;

public:
  Motor() {
    driver = new Adafruit_MotorShield();
    driver->begin();
    motorx1 = driver->getMotor(1);
    motorx2 = driver->getMotor(2);
    motory1 = driver->getMotor(3);
    motory2 = driver->getMotor(4);
  }

  // Set the speed of the x motors 0 (stopped) to 255 (full speed)
  // Direction is FORWARD, BACKWARD, BRAKE, RELEASE
  void spinx(int duty, uint8_t direction){
    motorx1->setSpeed(duty);
    motorx2->setSpeed(duty);

    motorx1->run(direction);
    motorx2->run(direction);
  }

  // Set the speed of the y motors 0 (stopped) to 255 (full speed)
  // Direction is FORWARD, BACKWARD, BRAKE, RELEASE
  void spiny(int duty, uint8_t direction){
    motory1->setSpeed(duty);
    motory2->setSpeed(duty);

    motory1->run(direction);
    motory2->run(direction);
  }

  void turnLeft(int duty) {
    motorx1->setSpeed(duty);
    motorx2->setSpeed(duty);
    motory1->setSpeed(duty);
    motory2->setSpeed(duty);

    motorx1->run(FORWARD);
    motorx2->run(FORWARD);
    motory1->run(FORWARD);
    motory2->run(FORWARD);
  }

  void turnRight(int duty) {
    motorx1->setSpeed(duty);
    motorx2->setSpeed(duty);
    motory1->setSpeed(duty);
    motory2->setSpeed(duty);

    motorx1->run(BACKWARD);
    motorx2->run(BACKWARD);
    motory1->run(BACKWARD);
    motory2->run(BACKWARD);
  }

  void brakeAll(){
    motorx1->run(BRAKE);
    motorx2->run(BRAKE);
    motory1->run(BRAKE);
    motory2->run(BRAKE);
  }

  void releaseAll(){
    motorx1->run(RELEASE);
    motorx2->run(RELEASE);
    motory1->run(RELEASE);
    motory2->run(RELEASE);
  }
};

#endif
