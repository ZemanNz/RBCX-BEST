#include <Arduino.h>
#include "robotka.h"

void setup() {
    rkConfig cfg;
    rkSetup(cfg);
    printf("Robotka started!\n");
    
    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED


}
void loop() {

    printf("Zadní Ultrasonic: %d mm\n", rkUltraMeasure(1));
    printf("Přední Ultrasonic: %d mm\n", rkUltraMeasure(2));
    printf_wifi("Zadní Ultrasonic: %d mm", rkUltraMeasure(1));
    printf_wifi("Přední Ultrasonic: %d mm", rkUltraMeasure(2));
    delay(300);
    if(rkButtonIsPressed(BTN_ON)){
        rkLedGreen(true); // Turn on green LED
        delay(1500);
         wall_following(1300 ,30.0f, 100.0f, true,
                   []() -> uint32_t { return rkUltraMeasure(2); },
                   []() -> uint32_t { return rkUltraMeasure(1); }); 
        rkLedGreen(false); // Turn off green LED
    }
    handleWebClients();
}