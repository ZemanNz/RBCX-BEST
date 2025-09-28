#include "_librk_motors.h"
#include "RBCX.h"
#include <Arduino.h> // Přidání této hlavičky pro funkce delay a millis
#include <thread> // Přidání této hlavičky pro std::thread
#include <iostream>

namespace rk {

Motors::Motors()
    : m_id_left(rb::MotorId::M1)
    , m_id_right(rb::MotorId::M4) {
}

Motors::~Motors() {
}

void Motors::init(const rkConfig& cfg) {
    m_id_left = (rb::MotorId)(cfg.motor_id_left - 1);
    m_id_right = (rb::MotorId)(cfg.motor_id_right - 1);
    m_polarity_switch_left = cfg.motor_polarity_switch_left;
    m_polarity_switch_right = cfg.motor_polarity_switch_right;

    m_wheel_circumference = M_PI * cfg.motor_wheel_diameter;
    m_max_speed = cfg.motor_max_ticks_per_second;
    prevod_motoru = cfg.prevod_motoru;
    roztec_kol = cfg.roztec_kol;

    auto& man
        = rb::Manager::get();

    // Set motor power limits
    man
        .setMotors()
        .pwmMaxPercent(m_id_left, cfg.motor_max_power_pct)
        .pwmMaxPercent(m_id_right, cfg.motor_max_power_pct)
        .set();

    if (rb::Manager::get().coprocFwVersion().number >= 0x010100) {
        const MotorConfig motorConf = {
            .velEpsilon = 3,
            .posEpsilon = 8,
            .maxAccel = cfg.motor_max_acceleration,
        };

        man.motor(m_id_left).setConfig(motorConf);
        man.motor(m_id_right).setConfig(motorConf);
    }
}

void Motors::setPower(int8_t left, int8_t right) {
    if (m_polarity_switch_left)
        left = -left;
    if (m_polarity_switch_right)
        right = -right;

    rb::Manager::get()
        .setMotors()
        .power(m_id_left, pctToPower(left))
        .power(m_id_right, pctToPower(right))
        .set();
}

void Motors::setPower(int8_t left, int8_t right, uint8_t pwm_pct_left, uint8_t pwm_pct_right) {
    if (m_polarity_switch_left)
        left = -left;
    if (m_polarity_switch_right)
        right = -right;

    rb::Manager::get()
        .setMotors()
        .pwmMaxPercent(m_id_left, pwm_pct_left)
        .pwmMaxPercent(m_id_right, pwm_pct_right)
        .power(m_id_left, pctToPower(left))
        .power(m_id_right, pctToPower(right))
        .set();
}

void Motors::setPowerById(rb::MotorId id, int8_t power) {
    if ((m_polarity_switch_left && id == m_id_left) || (m_polarity_switch_right && id == m_id_right))
        power = -power;

    rb::Manager::get()
        .setMotors()
        .power(id, pctToPower(power))
        .set();
}

void Motors::setSpeed(int8_t left, int8_t right) {
    if (m_polarity_switch_left)
        left = -left;
    if (m_polarity_switch_right)
        right = -right;

    rb::Manager::get()
        .setMotors()
        .speed(m_id_left, pctToSpeed(left))
        .speed(m_id_right, pctToSpeed(right))
        .set();
}

void Motors::setSpeed(int8_t left, int8_t right, uint8_t pwm_pct_left, uint8_t pwm_pct_right) {
    if (m_polarity_switch_left)
        left = -left;
    if (m_polarity_switch_right)
        right = -right;

    rb::Manager::get()
        .setMotors()
        .pwmMaxPercent(m_id_left, pwm_pct_left)
        .pwmMaxPercent(m_id_right, pwm_pct_right)
        .speed(m_id_left, pctToSpeed(left))
        .speed(m_id_right, pctToSpeed(right))
        .set();
}

void Motors::setSpeedById(rb::MotorId id, int8_t power) {
    if ((m_polarity_switch_left && id == m_id_left) || (m_polarity_switch_right && id == m_id_right))
        power = -power;

    rb::Manager::get()
        .setMotors()
        .speed(id, pctToSpeed(power))
        .set();
}


void Motors::driveById(rb::MotorId id, float mm, uint8_t speed, std::function<void()> callback) {
    if ((m_polarity_switch_left && id == m_id_left) || (m_polarity_switch_right && id == m_id_right))
        mm = -mm;

    auto& m = rb::Manager::get().motor(id);
    
    m.requestInfo([this, mm, speed, id, callback](rb::Motor& m) {
        rb::Manager::get()
            .setMotors()
            .driveToValue(id, m.position() + mmToTicks(mm), pctToSpeed(speed), [this, callback, id](rb::Motor& m) {
                callback();
            })
            .set();
    });
}

float Motors::position(rb::MotorId id) {
    auto res = ticksToMm(rb::Manager::get().motor(id).position());
    if ((m_polarity_switch_left && id == m_id_left) || (m_polarity_switch_right && id == m_id_right))
        res = -res;
    return res;
}

void Motors::setPosition(rb::MotorId id, float positionMm) {
    auto ticks = mmToTicks(positionMm);
    if ((m_polarity_switch_left && id == m_id_left) || (m_polarity_switch_right && id == m_id_right))
        ticks = -ticks;
    rb::Manager::get().motor(id).setCurrentPosition(ticks);
}

void Motors::joystick(int32_t x, int32_t y) {
    x = scale(x);
    y = scale(y);

    int r = ((y - (x / 1.5f)));
    int l = ((y + (x / 1.5f)));

    r = rb::clamp(r, -100, 100);
    l = rb::clamp(l, -100, 100);

    if (r < 0 && l < 0) {
        std::swap(r, l);
    }
    setSpeed(l, r);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int Motors::timeout_ms(float mm, float speed){
    return static_cast<int>(2750 * mm / speed); // vynásobeno 2 pro jistotu
}

void Motors::forward(float mm, float speed) {
    auto& man = rb::Manager::get();
    
    float m_kp = 0.12f; // Proporcionální konstanta
    float m_min_speed = 20.0f; // Minimální rychlost motorů
    float m_max_correction = 10.0f; // Maximální korekce rychlosti
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);
    
    int target_ticks = mmToTicks(mm);
    int left_pos = 0;
    int right_pos = 0;
    std::cout << "Target ticks: " << target_ticks << std::endl;
    // Základní rychlosti s přihlédnutím k polaritě
    float base_speed_left = m_polarity_switch_left ? -speed : speed;
    float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
    unsigned long start_time = millis();
    int timeoutMs = timeout_ms(mm, speed);
    
    while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
          (millis() - start_time < timeoutMs)) {
        
        // Čtení pozic
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });
        std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
        // P regulátor - pracuje s absolutními hodnotami pozic
        int error = abs(left_pos) - abs(right_pos);
        float correction = error * m_kp;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
        // Výpočet korigovaných rychlostí
        float speed_left = base_speed_left;
        float speed_right = base_speed_right;
        
        // Aplikace korekce podle polarity
        if (error > 0) {
            // Levý je napřed - zpomalit levý
            if (m_polarity_switch_left) {
                speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
            } else {
                speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
            }
        } else if (error < 0) {
            // Pravý je napřed - zpomalit pravý
            if (m_polarity_switch_right) {
                speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
            } else {
                speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
            }
        }
        
        // Zajištění minimální rychlosti
        if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
            speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
        }
        if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
            speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
        }
        
        // Nastavení výkonu motorů
        man.motor(m_id_left).power(pctToSpeed(speed_left));
        man.motor(m_id_right).power(pctToSpeed(speed_right));
        std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
        delay(10);
    }
    
    // Zastavení motorů
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
}


// void Motors::forward(float mm, float speed){
//     float speed_right = speed;
//     float speed_left = speed;
//     auto& man = rb::Manager::get();
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
//     int target_position =  mmToTicks(mm);
//     int p = 1;
//     int left_pos = 0;
//     int right_pos = 0;
//     if (m_polarity_switch_left){
//         speed_left = -speed_left;
//     }

//     if (m_polarity_switch_right){
//         speed_right = -speed_right;
//     }   
//     std::cout << "Target ticks: " << target_position << std::endl;
//     while(target_position > abs(left_pos) || target_position > abs(right_pos)){
//         man.motor(m_id_left).power(pctToSpeed(speed_left));
//         man.motor(m_id_right).power(pctToSpeed(speed_right));
//         delay(10);
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//             left_pos = info.position();
//          });
//          man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//             right_pos = info.position();
//          });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         p= (left_pos + right_pos);
//         speed_left = speed_left - (p/100);
//         speed_right = speed_right - (p/100);
//         std::cout << "P: " << p << std::endl;
//         std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
//     }
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }
// void Motors::forward(float mm, float speed) {
//     // 1. Reset pozic
//     auto& man = rb::Manager::get();
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
    
//     // 2. Cílová vzdálenost
//     int target_ticks = mmToTicks(mm);
    
//     // 3. Inicializace P regulátoru
//     float kP = 0.1f;  // Konstanta P regulátoru
//     int last_left_pos = 0;
//     int last_right_pos = 0;
//     std::cout << "Target ticks: " << target_ticks << std::endl;
//     int left_pos = 0;
//     int right_pos = 0;
//     // 4. Hlavní cyklus - ČEKÁME NA OBA MOTORY!
//     while (abs(left_pos) < target_ticks || 
//            abs(right_pos) < target_ticks) {
        
//         // Aktuální pozice
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//            left_pos = info.position();
//         });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//            right_pos = info.position();
//         });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         // P regulátor - koriguje rozdíl mezi motory
//         int error = left_pos - right_pos;  // Rozdíl v ujeté vzdálenosti
//         int correction = error * kP;       // Korekce
        
//         // Nastavení rychlostí s korekcí
//         float speed_left_corrected = speed - correction;
//         float speed_right_corrected = speed + correction;
        
//         // Aplikace polarity
//         if (m_polarity_switch_left) speed_left_corrected = -speed_left_corrected;
//         if (m_polarity_switch_right) speed_right_corrected = -speed_right_corrected;
        
//         // Použití SPEED místo POWER pro regulovaný pohyb
//         man.motor(m_id_left).power(pctToSpeed(speed_left_corrected));
//         man.motor(m_id_right).power(pctToSpeed(speed_right_corrected));
        
//         delay(10);
        
//         // Uložení pozic pro příští iteraci
//         last_left_pos = left_pos;
//         last_right_pos = right_pos;
//     }
    
//     // 5. Zastavení motorů
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }
/////////////////////////////////////////////////////////////////////////////////////////////////

int32_t Motors::scale(int32_t val) {
    return val * 100 / RBPROTOCOL_AXIS_MAX;
}

int16_t Motors::pctToPower(int8_t pct) {
    return rb::clamp(pct * INT16_MAX / 100, -INT16_MAX, INT16_MAX);
}

int16_t Motors::pctToSpeed(float pct) const {
    return rb::clamp(static_cast<int16_t>(pct * m_max_speed / 100), static_cast<int16_t>(-INT16_MAX), static_cast<int16_t>(INT16_MAX));
}

int32_t Motors::mmToTicks(float mm) const {
    return (mm / m_wheel_circumference) * prevod_motoru;
}

float Motors::ticksToMm(int32_t ticks) const {
    return float(ticks) / prevod_motoru * m_wheel_circumference;
}

}; // namespacer rk
