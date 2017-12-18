#include <Arduino.h>
#include "IMU.h"
#include "WaitUntil.h"
#include "Motor.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

void setup() {
    Serial.begin(115200);
    IMU imu;
    pinMode(LED_BUILTIN, OUTPUT);

    analogWrite(9, 100);
    bool init = imu.initialize();

    if (!init) {
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(1000);                       // wait for a second
            digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
            delay(1000);                       // wait for a second
        }
    }

    Motor *motor = new Motor();

    // In microseconds.
    unsigned long ts = 100;
    Wait wait(ts);
    wait.start();

    motor->spinx(255, FORWARD);

    int speedx = 0, speedy = 0;
    // Start main loop
    for (;;) {
        wait.end();
        // Do calculations!!
        float xz = imu.getTiltXZ();
        float yz = imu.getTiltYZ();
        speedx = (1 - abs(xz) / 3.14) * 255;
        motor->spinx(speedx, (xz > 0) ? FORWARD : BACKWARD);


        Serial.print("Tilt XZ: ");
        Serial.print(xz);
        Serial.print("\n");
        Serial.print("Tilt YZ: ");
        Serial.print(yz);
        Serial.print("\n");
        imu.readAccel();
//      imu.printAccel();
    }
}

// I don't want to do this in a separate function.
void loop() {
}

#pragma clang diagnostic pop