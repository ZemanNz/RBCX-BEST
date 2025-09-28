#include <Arduino.h>
#include "robotka.h"


void setup() {
    rkConfig cfg;
    rkSetup(cfg);

    printf("Robotka started!\n");

    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED
    rkLedYellow(true); // Turn on yellow LED
    delay(1000);
    rkLedRed(false); // Turn off red LED

    // forward(1000, 100);
    //rkMotorsSetPower(100, 100);

    
    forward(2000, 50);
    delay(500);

}//230- dole 0 -nahore

void loop() {

}