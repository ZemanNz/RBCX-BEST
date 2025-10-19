#include <stdarg.h>
#include <stdio.h>
#include <Adafruit_TCS34725.h>
#include "_librk_context.h"
#include "_librk_smart_servo.h"
#include "robotka.h"
#ifdef USE_VL53L0X
#include <Adafruit_VL53L0X.h>
#include <rk_laser.cpp>
#endif
#include "RBCX.h"
#include <Adafruit_TCS34725.h>
#define TAG "robotka"

using namespace rb;
using namespace rk;
// Empty loop in case the user won't supply one
void __attribute__((weak)) loop() {
}

void rkSetup(const rkConfig& cfg) {
    gCtx.setup(cfg);
}

void rkControllerSendLog(const char* format, ...) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }
    va_list args;
    va_start(args, format);
    gCtx.prot()->send_log(format, args);
    va_end(args);
}

void rkControllerSendLog(const std::string& text) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }

    gCtx.prot()->send_log(text);
}

void rkControllerSend(const char* cmd, rbjson::Object* data) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }
    gCtx.prot()->send(cmd, data);
}

void rkControllerSendMustArrive(const char* cmd, rbjson::Object* data) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }
    gCtx.prot()->send_mustarrive(cmd, data);
}

uint32_t rkBatteryPercent() {
    return Manager::get().battery().pct();
}

uint32_t rkBatteryVoltageMv() {
    return Manager::get().battery().voltageMv();
}

int16_t rkTemperature() {
    return Manager::get().battery().temperatureC();
}

void rkMotorsSetPower(int8_t left, int8_t right) {
    gCtx.motors().setPower(left, right);
}

void rkMotorsSetPowerLeft(int8_t power) {
    gCtx.motors().setPowerById(gCtx.motors().idLeft(), power);
}

void rkMotorsSetPowerRight(int8_t power) {
    gCtx.motors().setPowerById(gCtx.motors().idRight(), power);
}

void rkMotorsSetPowerById(uint8_t id, int8_t power) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }
    gCtx.motors().setPowerById(rb::MotorId(id), power);
}

void rkMotorsSetSpeed(int8_t left, int8_t right) {
    gCtx.motors().setPower(left, right);
}

void rkMotorsSetSpeedLeft(int8_t speed) {
    gCtx.motors().setSpeedById(gCtx.motors().idLeft(), speed);
}

void rkMotorsSetSpeedRight(int8_t speed) {
    gCtx.motors().setSpeedById(gCtx.motors().idRight(), speed);
}

void rkMotorsSetSpeedById(uint8_t id, int8_t speed) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }
    gCtx.motors().setSpeedById(rb::MotorId(id), speed);
}
void rkMotorsDrive(float mmLeft, float mmRight, float speed_left, float speed_right) {
    SemaphoreHandle_t binary = xSemaphoreCreateBinary();
    gCtx.motors()
        .drive(mmLeft, mmRight, speed_left, speed_right, [=]() {
            xSemaphoreGive(binary);
        });
    xSemaphoreTake(binary, portMAX_DELAY);
    vSemaphoreDelete(binary);
}

void rkMotorsDriveLeft(float mm, uint8_t speed) {
    rkMotorsDriveById(uint8_t(gCtx.motors().idLeft()) + 1, mm, speed);
}

void rkMotorsDriveRight(float mm, uint8_t speed) {
    rkMotorsDriveById(uint8_t(gCtx.motors().idRight()) + 1, mm, speed);
}

void rkMotorsDriveById(uint8_t id, float mm, uint8_t speed) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }

    SemaphoreHandle_t binary = xSemaphoreCreateBinary();
    gCtx.motors()
        .driveById(rb::MotorId(id), mm, speed, [=]() {
            xSemaphoreGive(binary);
        });
    xSemaphoreTake(binary, portMAX_DELAY);
    vSemaphoreDelete(binary);
}

void rkMotorsDriveAsync(float mmLeft, float mmRight, uint8_t speed_left, uint8_t speed_right, std::function<void()> callback) {
    gCtx.motors().drive(mmLeft, mmRight, speed_left, speed_right, std::move(callback));
}

void rkMotorsDriveLeftAsync(float mm, uint8_t speed, std::function<void()> callback) {
    gCtx.motors().driveById(gCtx.motors().idLeft(), mm, speed, std::move(callback));
}

void rkMotorsDriveRightAsync(float mm, uint8_t speed, std::function<void()> callback) {
    gCtx.motors().driveById(gCtx.motors().idRight(), mm, speed, std::move(callback));
}

void rkMotorsDriveByIdAsync(uint8_t id, float mm, uint8_t speed, std::function<void()> callback) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }
    gCtx.motors().driveById(rb::MotorId(id), mm, speed, std::move(callback));
}

float rkMotorsGetPositionLeft(bool fetch) {
    return rkMotorsGetPositionById((uint8_t)gCtx.motors().idLeft() + 1, fetch);
}

float rkMotorsGetPositionRight(bool fetch) {
    return rkMotorsGetPositionById((uint8_t)gCtx.motors().idRight() + 1, fetch);
}

float rkMotorsGetPositionById(uint8_t id, bool fetch) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return 0.f;
    }

    if(fetch) {
        SemaphoreHandle_t binary = xSemaphoreCreateBinary();
        auto& m = rb::Manager::get().motor(rb::MotorId(id));
        m.requestInfo([=](rb::Motor& m) {
            xSemaphoreGive(binary);
        });
        xSemaphoreTake(binary, portMAX_DELAY);
        vSemaphoreDelete(binary);
    }

    return gCtx.motors().position(rb::MotorId(id));
}

void rkMotorsSetPositionLeft(float positionMm) {
    rkMotorsSetPositionById((uint8_t)gCtx.motors().idLeft() + 1, positionMm);
}

void rkMotorsSetPositionRight(float positionMm) {
    rkMotorsSetPositionById((uint8_t)gCtx.motors().idRight() + 1, positionMm);
}

void rkMotorsSetPositionById(uint8_t id, float positionMm) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }

    gCtx.motors().setPosition(rb::MotorId(id), positionMm);
}

void rkMotorsJoystick(int32_t x, int32_t y) {
    gCtx.motors().joystick(x, y);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void forward(float mm, float speed){
    gCtx.motors().forward(mm, speed);
}
void backward(float mm, float speed){
    gCtx.motors().backward(mm, speed);
}   
void turn_on_spot_left(float angle, float speed){
    gCtx.motors().turn_on_spot_left(angle, speed);
}
void turn_on_spot_right(float angle, float speed){
    gCtx.motors().turn_on_spot_right(angle, speed);
}
void radius_right(float radius, float angle, float speed){
    gCtx.motors().radius_right(radius, angle, speed);
}
void radius_left(float radius, float angle, float speed){
    gCtx.motors().radius_left(radius, angle, speed);
}
void forward_acc(float mm, float speed){
    gCtx.motors().forward_acc(mm, speed);
}
void backward_acc(float mm, float speed){
    gCtx.motors().backward_acc(mm, speed);
}
void back_buttons(float speed){
    gCtx.motors().back_buttons(speed);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rkLedRed(bool on) {
    Manager::get().leds().red(on);
}

void rkLedYellow(bool on) {
    Manager::get().leds().yellow(on);
}

void rkLedGreen(bool on) {
    Manager::get().leds().green(on);
}

void rkLedBlue(bool on) {
    Manager::get().leds().blue(on);
}

void rkLedAll(bool on) {
    auto& l = Manager::get().leds();
    l.red(on);
    l.yellow(on);
    l.green(on);
    l.blue(on);
}

void rkLedById(uint8_t id, bool on) {
    if (id == 0) {
        ESP_LOGE(TAG, "%s: invalid id %d, LEDs are indexed from 1, just like on the board (LED1, LED2...)!", __func__, id);
        return;
    } else if (id > 4) {
        ESP_LOGE(TAG, "%s: maximum LED id is 4, you are using %d!", __func__, id);
        return;
    }

    auto& l = Manager::get().leds();
    l.byId(LedId((1 << (id - 1))), on);
}

bool rkButtonIsPressed(rkButtonId id, bool waitForRelease) {
    auto& b = Manager::get().buttons();
    for (int i = 0; i < 3; ++i) {
        const bool pressed = b.byId(ButtonId(id));
        if (!pressed)
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (waitForRelease) {
        rkButtonWaitForRelease(id);
    }
    return true;
}

void rkButtonOnChangeAsync(std::function<bool(rkButtonId, bool)> callback) {
    Manager::get().buttons().onChange([callback](rb::ButtonId id, bool pressed) -> bool {
        return callback(rkButtonId(id), pressed);
    });
}

void rkButtonWaitForRelease(rkButtonId id) {
    int counter = 0;
    auto& b = Manager::get().buttons();
    while (true) {
        const bool pressed = b.byId(ButtonId(id));
        if (!pressed) {
            if (++counter > 3)
                return;
        } else {
            counter = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


uint16_t rkIrLeft() {
    return gCtx.irRead(gCtx.irChanLeft());
}

uint16_t rkIrRight() {
    return gCtx.irRead(gCtx.irChanRight());
}

uint32_t rkUltraMeasure(uint8_t id) {
    if (id == 0) {
        ESP_LOGE(TAG, "%s: invalid id %d, Ultrasounds are indexed from 1, just like on the board (U1, U2...)!", __func__, id);
        return 0;
    } else if (id > 4) {
        ESP_LOGE(TAG, "%s: maximum Ultrasound id is 4, you are using %d!", __func__, id);
        return 0;
    }

    return Manager::get().ultrasound(id - 1).measure();
}

void rkUltraMeasureAsync(uint8_t id, std::function<bool(uint32_t)> callback) {
    if (id == 0) {
        ESP_LOGE(TAG, "%s: invalid id %d, Ultrasounds are indexed from 1, just like on the board (U1, U2...)!", __func__, id);
        callback(0);
        return;
    } else if (id > 4) {
        ESP_LOGE(TAG, "%s: maximum Ultrasound id is 4, you are using %d!", __func__, id);
        callback(0);
        return;
    }

    Manager::get().ultrasound(id - 1).measureAsync(std::move(callback));
}

void rkBuzzerSet(bool on) {
    Manager::get().piezo().setState(on);
}

void rk_laser_init(const char* name,TwoWire& bus,  Adafruit_VL53L0X& lox, uint8_t   xshut_pin,uint8_t    new_address){

    return rk_laser_init_basic(name,  bus, lox, xshut_pin, new_address);
}

int rk_laser_measure(const char* name){
    return rk_laser_measure_basic(name);
}

#define MAX_COLOR_SENSORS 2

struct ColorSensor {
  const char*           name;
  Adafruit_TCS34725*    tcs;
  TwoWire*              bus;
};
static ColorSensor colorSensors[MAX_COLOR_SENSORS];
static uint8_t    colorCount = 0;

/**
 * @brief Inicializuje TCS34725 barevný senzor na zadané I2C sběrnici.
 *
 * Funkce ukládá instanci senzoru do interního pole podle jména a spouští
 * komunikaci s modulem. Každý senzor je identifikován unikátním jménem.
 *
 * @param name    Textový identifikátor senzoru (např. "front" nebo "down").
 * @param bus     Referenční I2C sběrnice (Wire nebo Wire1) pro komunikaci.
 * @param tcs     Reference na instanci Adafruit_TCS34725 pro daný senzor.
 * @return true   Pokud se senzor úspěšně inicializoval.
 * @return false  Pokud se nepodařilo spojení s modulem.
 */
bool rkColorSensorInit(const char* name, TwoWire& bus, Adafruit_TCS34725& tcs)
{
  if (colorCount >= MAX_COLOR_SENSORS) {
    Serial.println("ERROR: Max pocet color sensoru prekrocen");
    return false;
  }

  // nastav I2C sběrnici
  tcs.begin(TCS34725_ADDRESS, &bus);
  if (!tcs.begin(TCS34725_ADDRESS, &bus)) {
    Serial.print("ERROR: Nelze pripojit k color senzoru ");
    Serial.println(name);
    return false;
  }

  // ulož konfiguraci
  ColorSensor& s = colorSensors[colorCount++];
  s.name = name;
  s.tcs  = &tcs;
  s.bus  = &bus;

  return true;
}

/**
 * @brief Načte hodnoty RGB z barevného senzoru podle jeho identifikátoru.
 *
 * Funkce najde senzor v interním seznamu podle jména a zavolá getRGB, 
 * přičte kompanzní faktor a vrátí normalizované hodnoty v rozsahu 0.0–1.0.
 *
 * @param name  Textový identifikátor senzoru (stejný jako při init).
 * @param r     Ukazatel na float pro červenou složku (0.0–1.0).
 * @param g     Ukazatel na float pro zelenou složku (0.0–1.0).
 * @param b     Ukazatel na float pro modrou složku (0.0–1.0).
 * @return true   Pokud se měření úspěšně provedlo.
 * @return false  Pokud senzor není nalezen nebo měření selhalo.
 */
bool rkColorSensorGetRGB(const char* name, float* r, float* g, float* b)
{
    for (uint8_t i = 0; i < colorCount; i++) {
        ColorSensor& s = colorSensors[i];
        if (strcmp(s.name, name) == 0) {
            // Načti RGB hodnoty (0-255) přímo do poskytnutých proměnných
            s.tcs->getRGB(r, g, b);
            return true;
        }
    }
    return false;
}

void rkServosSetPosition(uint8_t id, float angleDegrees) {
    id -= 1;
    if (id >= rb::StupidServosCount) {
        ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
        return;
    }
    gCtx.stupidServoSet(id, angleDegrees);
}

float rkServosGetPosition(uint8_t id) {
    id -= 1;
    if (id >= rb::StupidServosCount) {
        ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
        return NAN;
    }
    return gCtx.stupidServoGet(id);
}

void rkServosDisable(uint8_t id) {
    id -= 1;
    if (id >= rb::StupidServosCount) {
        ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
        return;
    }
    rb::Manager::get().stupidServo(id).disable();
}

lx16a::SmartServoBus& rkSmartServoBus(uint8_t servo_count) {
    static rk::SmartServoBusInitializer init(servo_count);
    return init.bus();
}

// Smart Servo funkce
void rkSmartServoInit(lx16a::SmartServoBus& bus, int id, int low, int high) {
    bus.setAutoStop(id, false);
    bus.limit(id, lx16a::Angle::deg(low), lx16a::Angle::deg(high));
    bus.setAutoStopParams(
        lx16a::SmartServoBus::AutoStopParams{
            .max_diff_centideg = 400,
            .max_diff_readings = 2,
        });
    printf("Smart Servo %d inicializováno (limity: %d°-%d°)\n", id, low, high);
}

void rkSmartServoMove(lx16a::SmartServoBus& bus, int id, int angle, int speed) {
    if (angle < 0 || angle > 240) {
        printf("Chyba: Úhel musí být v rozsahu 0–240 stupňů.\n");
        return;
    }
    bus.setAutoStop(id, false);
    bus.set(id, lx16a::Angle::deg(angle), speed);
    printf("Smart Servo %d move na %d stupňů rychlostí %d\n", id, angle, speed);
}

void rkSmartServoSoftMove(lx16a::SmartServoBus& bus, int id, int angle, int speed) {
    if (angle < 0 || angle > 240) {
        Serial.println("Chyba: Úhel musí být v rozsahu 0–240 stupňů.");
        return;
    }
    bus.setAutoStop(id, true);
    bus.set(id, lx16a::Angle::deg(angle), speed);
    printf("Smart Servo %d soft_move na %d stupňů rychlostí %d\n", id, angle, speed);
}

void print_wifi(const char* message) {
    gCtx.motors().print_wifi(message);
}
void handleWebClients() {
    gCtx.motors().handleWebClient();
}

void wifi_control_wasd() {
    
    // Hlavní smyčka pro obsluhu webových klientů
    bool still = true;
    while(still) {
        still = rk::Wifi::handleWebClients();
        delay(50);
    }
}

void wifi_terminal() {
    while(true){
        rk::Wifi::handleWebClients_terminal();
        delay(50);
    }
}
// Pomocná funkce pro parsování parametrů
static bool parseParams(const String& params, float* out, int count) {
    int idx = 0, last = 0;
    for (int i = 0; i < count; ++i) {
        idx = params.indexOf(',', last);
        String val = (idx == -1) ? params.substring(last) : params.substring(last, idx);
        val.trim();
        if (!val.length()) return false;
        out[i] = val.toFloat();
        last = idx + 1;
        if (idx == -1 && i < count - 1) return false;
    }
    return true;
}

// Funkce pro zpracování příkazů
static void processCommand(const String &cmd) {
    // FORWARD - forward(mm, speed)
    if (cmd.startsWith("forward(")) {
        float params[2];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát forward");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 2)) {
            Serial.println("Chyba v parametrech forward");
            return;
        }
        forward(params[0], params[1]);
        Serial.println("forward zavoláno");
        return;
    }
    
    // FORWARD_ACC - forward_acc(mm, speed)
    if (cmd.startsWith("forward_acc(")) {
        float params[2];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát forward_acc");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 2)) {
            Serial.println("Chyba v parametrech forward_acc");
            return;
        }
        forward_acc(params[0], params[1]);
        Serial.println("forward_acc zavoláno");
        return;
    }
    
    // BACKWARD - backward(mm, speed)
    if (cmd.startsWith("backward(")) {
        float params[2];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát backward");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 2)) {
            Serial.println("Chyba v parametrech backward");
            return;
        }
        backward(params[0], params[1]);
        Serial.println("backward zavoláno");
        return;
    }
    
    // BACKWARD_ACC - backward_acc(mm, speed)
    if (cmd.startsWith("backward_acc(")) {
        float params[2];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát backward_acc");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 2)) {
            Serial.println("Chyba v parametrech backward_acc");
            return;
        }
        backward_acc(params[0], params[1]);
        Serial.println("backward_acc zavoláno");
        return;
    }
    
    // TURN_ON_SPOT_LEFT - turn_on_spot_left(angle, speed)
    if (cmd.startsWith("turn_on_spot_left(")) {
        float params[2];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát turn_on_spot_left");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 2)) {
            Serial.println("Chyba v parametrech turn_on_spot_left");
            return;
        }
        turn_on_spot_left(params[0], params[1]);
        Serial.println("turn_on_spot_left zavoláno");
        return;
    }
    
    // TURN_ON_SPOT_RIGHT - turn_on_spot_right(angle, speed)
    if (cmd.startsWith("turn_on_spot_right(")) {
        float params[2];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát turn_on_spot_right");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 2)) {
            Serial.println("Chyba v parametrech turn_on_spot_right");
            return;
        }
        turn_on_spot_right(params[0], params[1]);
        Serial.println("turn_on_spot_right zavoláno");
        return;
    }
    
    // RADIUS_LEFT - radius_left(radius, angle, speed)
    if (cmd.startsWith("radius_left(")) {
        float params[3];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát radius_left");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 3)) {
            Serial.println("Chyba v parametrech radius_left");
            return;
        }
        radius_left(params[0], params[1], params[2]);
        Serial.println("radius_left zavoláno");
        return;
    }
    
    // RADIUS_RIGHT - radius_right(radius, angle, speed)
    if (cmd.startsWith("radius_right(")) {
        float params[3];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát radius_right");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 3)) {
            Serial.println("Chyba v parametrech radius_right");
            return;
        }
        radius_right(params[0], params[1], params[2]);
        Serial.println("radius_right zavoláno");
        return;
    }
    
    // BACK_BUTTONS - back_buttons(speed)
    if (cmd.startsWith("back_buttons(")) {
        float params[1];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát back_buttons");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 1)) {
            Serial.println("Chyba v parametrech back_buttons");
            return;
        }
        back_buttons(params[0]);
        Serial.println("back_buttons zavoláno");
        return;
    }
    
    Serial.println("Neznámý příkaz");
}

void rkSerialTerminal() {
    // Kontrola, zda je Serial inicializován
    if (!Serial) {
        ESP_LOGE(TAG, "Serial není inicializován! Volej Serial.begin(115200) v setup()");
        return;
    }
    
    Serial.println("=== ROBOTKA SERIAL TERMINAL ===");
    Serial.println("Dostupné příkazy:");
    Serial.println("forward(mm, speed)           - např. forward(1000, 50)");
    Serial.println("forward_acc(mm, speed)       - např. forward_acc(800, 50)");
    Serial.println("backward(mm, speed)          - např. backward(800, 40)");
    Serial.println("backward_acc(mm, speed)      - např. backward_acc(600, 40)");
    Serial.println("turn_on_spot_left(angle, speed) - např. turn_on_spot_left(90, 40)");
    Serial.println("turn_on_spot_right(angle, speed)- např. turn_on_spot_right(90, 40)");
    Serial.println("radius_left(radius, angle, speed) - např. radius_left(200, 90, 40)");
    Serial.println("radius_right(radius, angle, speed)- např. radius_right(200, 90, 40)");
    Serial.println("back_buttons(speed)          - např. back_buttons(30)");
    Serial.println("Zadej příkaz:");
    
    while(true) {
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
                processCommand(line);
            }
        }
        delay(15);
    }
}
