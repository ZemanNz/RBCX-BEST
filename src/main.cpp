#include <Arduino.h>
#include "robotka.h"

void setup() {
    rkConfig cfg;
    //cfg.motor_max_power_pct = 50;
    rkSetup(cfg);
    printf("Robotka started!\n");
    
    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED
    //std::cout << "-------------------------" << std::endl;
    //std::cout << "Max speed je : " << max_rychlost() << " ticks/sec" << std::endl;

}

void loop() {

    // printf("Zadní Ultrasonic: %d mm\n", rkUltraMeasure(1));
    // delay(200);
    // printf("Přední Ultrasonic: %d mm\n", rkUltraMeasure(2));
    // printf_wifi("Zadní Ultrasonic: %d mm", rkUltraMeasure(1));
    // printf_wifi("Přední Ultrasonic: %d mm", rkUltraMeasure(2));
    delay(50);
    if(rkButtonIsPressed(BTN_UP)){
        rkLedGreen(true); // Turn on green LED
        delay(1500);
        radius_left(200,360,40);
        rkLedGreen(false); // Turn off green LED
    }
    if(rkButtonIsPressed(BTN_DOWN)){
        rkLedGreen(true); // Turn on green LED
        delay(1500);
        radius_right(200,360,40);

        rkLedGreen(false); // Turn off green LED
    }
    if(rkButtonIsPressed(BTN_LEFT)){
        rkLedGreen(true); // Turn on green LED
        delay(2000);
        radius_left(100,180,60);
        rkLedGreen(false); // Turn off green LED
    }
    if(rkButtonIsPressed(BTN_RIGHT)){
        rkLedGreen(true); // Turn on green LED
        delay(2000);
        radius_right(100,180,60);
        
        rkLedGreen(false); // Turn off green LED
    }
}