#include <Arduino.h>
#include "robotka.h"

// Nejjednodušší struktura
typedef struct __attribute__((packed)) {
    uint8_t servo_id;
    uint8_t position; // 0-255
} SimpleCommand;

void setup() {
    rkConfig cfg;
    rkSetup(cfg);
    rkUartInit(115200, 16, 17);
    
    printf("Simple UART receiver ready\n");
}

void loop() {
    SimpleCommand cmd;
    
    // Čekáme na příkaz
    if (rkUartReceive_blocking(&cmd, sizeof(cmd), 1000)) {
        // TADY PRACUJEME S PŘIJATÝMI DATY:
        
        // 1. Výpis na serial
        printf("Servo %d -> Position %d\n", cmd.servo_id, cmd.position);
        
        // 2. Ovládání hardware
        int angle = map(cmd.position, 0, 255, 0, 240);
        rkSmartServoMove(cmd.servo_id, angle, 100);
        
        // 3. Odeslání odpovědi
        rkUartSend(&cmd, sizeof(cmd));
    } else {
        printf("Timeout - žádná data\n");
    }
}