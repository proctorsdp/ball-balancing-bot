#ifndef _Wait_Until_H
#define _Wait_Until_H

#include <Arduino.h>

/**
 *  This will provide an easy way to wait a certain time step before performing the next iteration.
 *
 *  Ideally, this should be done with a timer interrupt instead of using the arduino micros()
 *  function. This will only allow us to about 4 microseconds.
 */
class Wait {
private:
  unsigned long Ts;
  unsigned long lastT;
public:

  /**
   *  Sets the default timestep
   */
  Wait(unsigned long Ts) {
    this->Ts = Ts;
  }

/**
 *  initialize the start time for this time step.
 */
  void start() {
    lastT = micros();
  }

  /**
   *  This will block until the current time step is over.
   *  Note that we may have issues as micros() will overflow
   *  within 70 minutes on a 16Mhz processor
   */
  void end() {
    while((micros()-lastT) < Ts){;}
    start();
  }
};

#endif
