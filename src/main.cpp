#include <Arduino.h>
#include "robotka.h"

void setup() {
    rkConfig cfg;
    rkSetup(cfg);
    printf("Robotka started!\n");
    
    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED


    rkSmartServoInit(0, 0, 240);
    rkSmartServoInit(1, 0, 240);
    wifi_control_wasd();
    
}
void loop() {
    if (rkButtonIsPressed(BTN_UP)) {
        rkSmartServoMove(0,60);
        rkSmartServoMove(1,200);
    }
    if (rkButtonIsPressed(BTN_DOWN)) {
        rkSmartServoMove(0,160);
        rkSmartServoMove(1,120);
    } 
}