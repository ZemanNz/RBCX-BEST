#include <Arduino.h>
#include "robotka.h"

void setup() {
    rkConfig cfg;
    //cfg.motor_max_power_pct = 50;
    rkSetup(cfg);
    printf("Robotka started!\n");

    pinMode(cfg.Button1,INPUT_PULLUP);
    pinMode(cfg.Button2,INPUT_PULLUP);
    delay(100);
    for(int i= 0; i < 50; i++){
        std::cout<<"Tlacitko 1 : " << digitalRead(cfg.Button1) << std::endl;
        std::cout<<"Tlacitko 2 : " << digitalRead(cfg.Button2) << std::endl;
        delay(50);
    }
    
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
        forward_acc(1500,80);
        rkLedGreen(false); // Turn off green LED
    }
    if(rkButtonIsPressed(BTN_DOWN)){
        rkLedGreen(true); // Turn on green LED
        delay(1500);
        back_buttons(60);

        rkLedGreen(false); // Turn off green LED
    }
    if(rkButtonIsPressed(BTN_LEFT)){
        rkLedGreen(true); // Turn on green LED
        delay(2000);
        backward_acc(1500,50);
        rkLedGreen(false); // Turn off green LED
    }
    if(rkButtonIsPressed(BTN_RIGHT)){
        rkLedGreen(true); // Turn on green LED
        delay(2000);
        backward_acc(1500,80);          
        rkLedGreen(false); // Turn off green LED
    }
}