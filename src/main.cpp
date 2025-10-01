#include <Arduino.h>
#include "robotka.h"


void setup() {
    rkConfig cfg;
    rkSetup(cfg);

    printf("Robotka started!\n");

    rkLedRed(true); // Turn on red LED
    rkLedBlue(false); // Turn on blue LED
    rkLedYellow(true); // Turn on yellow LED
    delay(1000);
    rkLedRed(false); // Turn off red LED

    // forward(1000, 100);
    //rkMotorsSetPower(100, 100);
while (true){
    if (rkButtonIsPressed(BTN_LEFT)) {
        printf("Button ON is pressed!\n");
        rkLedBlue(true); // Turn off blue LED
        delay(1000);
        radius_left(180,360, 60);
        rkLedBlue(false); // Turn off blue LED
    }
    if(rkButtonRight()){
        printf("Button OFF is pressed!\n");
        rkLedGreen(true); // Turn off blue LED
        delay(1000);
        radius_left(120, 90, 40);
        rkLedGreen(false); // Turn off blue LED
    }
    //forward(2000, 60);
    delay(50);
    }
}//230- dole 0 -nahore

void loop() {

}