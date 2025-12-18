#include "robotka.h"

void setup() {
    Serial.begin(115200);
    rkConfig cfg;
    rkSetup(cfg);

    rkLedAll(true);
    delay(5000);
    rkLedAll(false);
}

void loop() {


}