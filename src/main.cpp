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
        print_wifi("Button ON is pressed!\n");
        rkLedBlue(true); // Turn off blue LED
        delay(1000);
        forward(1500,40);
        rkLedBlue(false); // Turn off blue LED
    }
    if(rkButtonRight()){
        printf("Button OFF is pressed!\n");
        print_wifi("Button OFF is pressed!\n");
        rkLedGreen(true); // Turn off blue LED
        delay(1000);
        forward(2000, 75);
        rkLedGreen(false); // Turn off blue LED
        rkLedGreen(false); // Turn off blue LED
    }
    if(rkButtonUp()){
        printf("Button UP is pressed!\n");
        print_wifi("Button UP is pressed!\n");
        rkLedYellow(true); // Turn off blue LED
        delay(1000);
        wifi_control_wasd();
        rkLedYellow(false); // Turn off blue LED
    }
    if(rkButtonDown()){
        printf("Button DOWN is pressed!\n");
        print_wifi("Button DOWN is pressed!\n");
        rkLedRed(true); // Turn off blue LED
        delay(1000);
        wifi_terminal();
        rkLedRed(false); // Turn off blue LED
    }
    //forward(2000, 60);
    handleWebClients();
    delay(50);
    }
}//230- dole 0 -nahore

void loop() {

}