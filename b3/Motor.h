#include <Adafruit_MotorShield.h>

#ifndef _MOTOR_H
#define _MOTOR_H

#include <wire.h>

class Motor {
private:
  Adafruit_MotorShield* driver;
  Adafruit_DCMotor *motora, *motorb, *motorc;

public:
  Motor() {
    driver = new Adafruit_MotorShield();
    driver->begin();
    motorxa = driver->getMotor(1);
    motorxb = driver->getMotor(2);
    motoryc = driver->getMotor(3);
  }

  // Set the speed of the x motors 0 (stopped) to 255 (full speed)
  // Direction is FORWARD, BACKWARD, BRAKE, RELEASE
  void spinx(int duty, uint8_t direction){
    motorx1->setSpeed(duty);
    motorx2->setSpeed(duty);

    motorx1->run(direction);
    motorx2->run(direction);
  }

  void turnLeft(int duty) {
    motora->setSpeed(duty);
    motorb->setSpeed(duty);
    motorc->setSpeed(duty);

    motora->run(FORWARD);
    motorb->run(FORWARD);
    motorc->run(FORWARD);
  }

  void turnRight(int duty) {
    motora->setSpeed(duty);
    motorb->setSpeed(duty);
    motorc->setSpeed(duty);

    motora->run(BACKWARD);
    motorb->run(BACKWARD);
    motorc->run(BACKWARD);
  }

  void brakeAll(){
    motora->run(BRAKE);
    motorb->run(BRAKE);
    motorc->run(BRAKE);
  }

  void releaseAll(){
    motora->run(RELEASE);
    motorb->run(RELEASE);
    motorc->run(RELEASE);
  }
};

#endif
