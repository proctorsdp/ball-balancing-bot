#include "IMU.h"
#include "WaitUntil.h"
#include "Motor.h"
//#include "Wiichuck.h"


  // Normalize 44-255 to -1-1
    float normalize(float x){
      x -= 171;
      if(x>0)
        x = x*127/84;
      return x/127;
    }

void setup() {
  IMU imu;
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  analogWrite(9, 100);
  bool init = imu.initialize();

  bool ledOn = false;
  if (!init) {
    while (1) {
      delay(1000);
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
        delay(1000);        // wait for a second
    }
  }

//  Wiichuck chuck;
//  chuck.init();
//  chuck.calibrate(); 

  Motor *motor = new Motor();

  // In microseconds.
  unsigned long ts = 100;
  Wait wait(ts);
  wait.start();

//  motor->spinx(255, FORWARD);

  int speedx = 0, speedy = 0;
//  float xz = 0, yz = 0;
//
//  
//      every(1000) {
//        ledOn = !ledOn;
//        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
//      }                     // wait for a second
//      
  // Start main loop
  for(;;) {
      wait.end();
      // Do calculations!!
      float xz = imu.getTiltXZ();
      float yz = imu.getTiltYZ();
//      if (chuck.poll()){
//        xz = normalize(chuck.joyX());
//        yz = normalize(chuck.joyY());
//      }
        
      speedx = (1-abs(xz)/3.14)*255;
      speedy = (1-abs(yz)/3.14)*255;
      motor->spinx(speedx, (xz>0)?FORWARD: BACKWARD);
      motor->spiny(speedy, (yz>0)?FORWARD: BACKWARD);
//      
      Serial.print("Speedx: ");
      Serial.print(speedx);
      Serial.print("");
      Serial.print(" Speedy: ");
      Serial.print(speedy);
      Serial.print("\n");
      imu.readAccel();
//      imu.printAccel();
  }
}

// I don't want to do this in a separate function.
void loop() {
}
