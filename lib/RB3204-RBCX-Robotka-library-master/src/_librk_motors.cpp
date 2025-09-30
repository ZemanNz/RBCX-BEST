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
    rozdil_v_kolech_levy = cfg.rozdil_v_kolech_levy;
    rozdil_v_kolech_pravy = cfg.rozdil_v_kolech_pravy;

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
    return static_cast<int>(295 * mm / speed);
}

void Motors::forward(float mm, float speed) {
    auto& man = rb::Manager::get();
    
    float m_kp = 0.23f; // Proporcionální konstanta
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
        man.motor(m_id_left).power(pctToSpeed(speed_left * rozdil_v_kolech_levy));
        man.motor(m_id_right).power(pctToSpeed(speed_right * rozdil_v_kolech_pravy));
        std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
        delay(10);
    }
    
    // Zastavení motorů
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
}

void Motors::backward(float mm, float speed) {
    auto& man = rb::Manager::get();
    
    float m_kp = 0.23f; // Proporcionální konstanta
    float m_min_speed = 20.0f; // Minimální rychlost motorů
    float m_max_correction = 10.0f; // Maximální korekce rychlosti
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);
    
    int target_ticks = -mmToTicks(mm);
    int left_pos = 0;
    int right_pos = 0;
    std::cout << "Target ticks: " << target_ticks << std::endl;
    // Základní rychlosti s přihlédnutím k polaritě
    float base_speed_left = m_polarity_switch_left ? speed : -speed;
    float base_speed_right = m_polarity_switch_right ? speed : -speed;
    
    unsigned long start_time = millis();
    int timeoutMs = timeout_ms(mm, speed);
    
    while((abs(target_ticks) > abs(left_pos) || target_ticks > abs(right_pos)) && 
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
        int error = abs(left_pos) * rozdil_v_kolech_pravy - abs(right_pos) * rozdil_v_kolech_levy;
        float correction = error * m_kp;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
        // Výpočet korigovaných rychlostí
        float speed_left = base_speed_left;
        float speed_right = base_speed_right;
        
        // Aplikace korekce podle polarity
        if (error > 0) {
            // Levý je napřed - zpomalit levý
            if (m_polarity_switch_left) {
                speed_left -= correction;  // Přidat k záporné rychlosti = zpomalit
            } else {
                speed_left += correction;  // Odečíst od kladné rychlosti = zpomalit
            }
        } else if (error < 0) {
            // Pravý je napřed - zpomalit pravý
            if (m_polarity_switch_right) {
                speed_right += correction;  // Odečíst od záporné rychlosti = zpomalit
            } else {
                speed_right -= correction;  // Přidat ke kladné rychlosti = zpomalit
            }
        }
        
        // Zajištění minimální rychlosti
        if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
            speed_left = (speed_left > 0) ? -m_min_speed : +m_min_speed;
        }
        if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
            speed_right = (speed_right > 0) ? -m_min_speed : +m_min_speed;
        }
        
        // Nastavení výkonu motorů
        man.motor(m_id_left).power(pctToSpeed(speed_left ));
        man.motor(m_id_right).power(pctToSpeed(speed_right));
        std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
        delay(10);
    }
    
    // Zastavení motorů
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
}

// void Motors::turn_on_spot_left(float angle, float speed) {
//     auto& man = rb::Manager::get();
    
//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);

//     int target_ticks = mmToTicks((M_PI * roztec_kol) * (angle / 360.0f));
//     int left_pos = 0;
//     int right_pos = 0;
//     std::cout << "Target ticks: " << target_ticks << std::endl;
//     // Základní rychlosti s přihlédnutím k polaritě
//     float base_speed_left = m_polarity_switch_left ? speed : -speed;
//     float base_speed_right = m_polarity_switch_right ? -speed : speed;

//     float speed_left = base_speed_left;
//     float speed_right = base_speed_right;

//     man.motor(m_id_left).power(pctToSpeed(speed_left));
//     man.motor(m_id_right).power(pctToSpeed(speed_right));

//     while(target_ticks > (abs(left_pos) +50) || target_ticks > (abs(right_pos) + 50)) {
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//              left_pos = info.position();
//           });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//              right_pos = info.position();
//           });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         delay(10);
//     }
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }

void Motors::turn_on_spot_left(float angle, float speed) {
    auto& man = rb::Manager::get();
    
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);
    float koeficient_korekce = 1.085f; // Korekční faktor pro jemné doladění otáčení
    float distance_per_wheel = (M_PI * roztec_kol) * (angle / 360.0f) * koeficient_korekce; // Vzdálenost, kterou musí každé kolo urazit
    int target_ticks = mmToTicks(distance_per_wheel);
    
    int left_pos = 0;
    int right_pos = 0;
    
    // Parametry regulátoru
    float m_kp = 0.15f;
    float m_max_correction = 20.0f;
    float m_min_speed = 15.0f;
    float m_min_decel_speed = 12.0f; // MINIMÁLNÍ RYCHLOST BĚHEM ZPOMALOVÁNÍ
    
    // Proměnné pro plynulé zrychlení/zpomalení
    float current_speed_left = 0;
    float current_speed_right = 0;
    const float acceleration = 2.0f;
    const float deceleration_start = 0.88f; // Ještě pozdější zpomalování
    
    // Základní směry rychlosti pro otáčení na místě doleva
    float target_speed_left = m_polarity_switch_left ? speed : -speed;
    float target_speed_right = m_polarity_switch_right ? -speed : speed;
    
    unsigned long last_time = millis();
    const unsigned long timeout_ms =8000;
    
    while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
          (millis() - last_time < timeout_ms)) {
        
        // Čtení pozic
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });
        
        // Výpočet ujeté vzdálenosti (průměr obou kol)
        float progress = (abs(left_pos) + abs(right_pos)) / 2.0f;
        float progress_ratio = progress / target_ticks;
        
        // PODMÍNKA PRO BRZKÉ UKONČENÍ - pokud jsme velmi blízko cíle
        if (abs(left_pos) > target_ticks + 80) {
            // Jsme velmi blízko cíle (98%) - okamžitě zastavujeme
            break;
        }
        
        // PLYNULÉ ZRYCHLENÍ A ZPOMALENÍ
        if (progress_ratio < deceleration_start) {
            // Fáze zrychlení - plynule zrychluj k cílové rychlosti
            current_speed_left = approachValue(current_speed_left, target_speed_left, acceleration);
            current_speed_right = approachValue(current_speed_right, target_speed_right, acceleration);
        } else {
            // Fáze zpomalení - plynule zpomaluj, ale NE POD MINIMÁLNÍ RYCHLOST
            float remaining_ratio = 1.0f - progress_ratio;
            float decel_speed = target_speed_left * (remaining_ratio / (1.0f - deceleration_start));
            
            // OMEZENÍ RYCHLOSTI - během zpomalování nikdy neklesneme pod minimum
            if (abs(decel_speed) < m_min_decel_speed) {
                decel_speed = (decel_speed > 0) ? m_min_decel_speed : -m_min_decel_speed;
            }
            
            current_speed_left = approachValue(current_speed_left, decel_speed, acceleration * 2.0f);
            current_speed_right = approachValue(current_speed_right, 
                                              target_speed_right * (remaining_ratio / (1.0f - deceleration_start)), 
                                              acceleration * 2.0f);
        }
        
        // P REGULÁTOR - vyrovnávání rychlosti motorů
        int error = abs(left_pos) - abs(right_pos);
        float correction = error * m_kp;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
        // Aplikace korekce
        float speed_left_corrected = current_speed_left;
        float speed_right_corrected = current_speed_right;
        
        if (error > 0) {
            // Levý je napřed - zpomalit levý, zrychlit pravý
            if (m_polarity_switch_left) {
                speed_left_corrected += correction;
            } else {
                speed_left_corrected -= correction;
            }
            if (m_polarity_switch_right) {
                speed_right_corrected -= correction;
            } else {
                speed_right_corrected += correction;
            }
        } else if (error < 0) {
            // Pravý je napřed - zpomalit pravý, zrychlit levý
            if (m_polarity_switch_left) {
                speed_left_corrected -= correction;
            } else {
                speed_left_corrected += correction;
            }
            if (m_polarity_switch_right) {
                speed_right_corrected += correction;
            } else {
                speed_right_corrected -= correction;
            }
        }
        
        // Zajištění minimální rychlosti během fáze zrychlení
        if (progress_ratio < deceleration_start && abs(speed_left_corrected) < m_min_speed) {
            speed_left_corrected = (speed_left_corrected > 0) ? m_min_speed : -m_min_speed;
        }
        if (progress_ratio < deceleration_start && abs(speed_right_corrected) < m_min_speed) {
            speed_right_corrected = (speed_right_corrected > 0) ? m_min_speed : -m_min_speed;
        }
        
        // Zajištění minimální rychlosti během zpomalování
        if (progress_ratio >= deceleration_start) {
            if (abs(speed_left_corrected) < m_min_decel_speed && abs(speed_left_corrected) > 0) {
                speed_left_corrected = (speed_left_corrected > 0) ? m_min_decel_speed : -m_min_decel_speed;
            }
            if (abs(speed_right_corrected) < m_min_decel_speed && abs(speed_right_corrected) > 0) {
                speed_right_corrected = (speed_right_corrected > 0) ? m_min_decel_speed : -m_min_decel_speed;
            }
        }
        
        // Nastavení výkonu motorů
        man.motor(m_id_left).power(pctToSpeed(speed_left_corrected));
        man.motor(m_id_right).power(pctToSpeed(speed_right_corrected));
        
        delay(10);
    }
    
    // Konečné zastavení
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
}

// Pomocná funkce pro plynulé přiblížení k hodnotě
float Motors::approachValue(float current, float target, float step) {
    if (current < target) {
        return std::min(current + step, target);
    } else if (current > target) {
        return std::max(current - step, target);
    }
    return current;
}

void Motors::turn_on_spot_right(float angle, float speed) {
    auto& man = rb::Manager::get();
    
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);

    float koeficient_korekce = 1.085f; // Korekční faktor pro jemné doladění otáčení
    float distance_per_wheel = (M_PI * roztec_kol) * (angle / 360.0f) * koeficient_korekce; // Vzdálenost, kterou musí každé kolo urazit
    int target_ticks = mmToTicks(distance_per_wheel);
    
    int left_pos = 0;
    int right_pos = 0;
    
    // Parametry regulátoru
    float m_kp = 0.15f;
    float m_max_correction = 20.0f;
    float m_min_speed = 15.0f;
    float m_min_decel_speed = 12.0f;
    
    // Proměnné pro plynulé zrychlení/zpomalení
    float current_speed_left = 0;
    float current_speed_right = 0;
    const float acceleration = 2.0f;
    const float deceleration_start = 0.88f;
    
    // ZÁKLADNÍ SMĚRY RYCHLOSTI PRO OTÁČENÍ NA MÍSTĚ DOPRAVA - OPACNÉ NEŽ DOLEVA
    float target_speed_left = m_polarity_switch_left ? -speed : speed;  // Levý dopředu
    float target_speed_right = m_polarity_switch_right ? speed : -speed; // Pravý dozadu
    
    unsigned long last_time = millis();
    const unsigned long timeout_ms = 8000;
    
    while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
          (millis() - last_time < timeout_ms)) {
        
        // Čtení pozic
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });
        
        // Výpočet ujeté vzdálenosti (průměr obou kol)
        float progress = (abs(left_pos) + abs(right_pos)) / 2.0f;
        float progress_ratio = progress / target_ticks;
        
        // PODMÍNKA PRO BRZKÉ UKONČENÍ - pokud jsme velmi blízko cíle
        if (abs(left_pos) > target_ticks + 80) {
            break;
        }
        
        // PLYNULÉ ZRYCHLENÍ A ZPOMALENÍ
        if (progress_ratio < deceleration_start) {
            current_speed_left = approachValue(current_speed_left, target_speed_left, acceleration);
            current_speed_right = approachValue(current_speed_right, target_speed_right, acceleration);
        } else {
            float remaining_ratio = 1.0f - progress_ratio;
            float decel_speed = target_speed_left * (remaining_ratio / (1.0f - deceleration_start));
            
            if (abs(decel_speed) < m_min_decel_speed) {
                decel_speed = (decel_speed > 0) ? m_min_decel_speed : -m_min_decel_speed;
            }
            
            current_speed_left = approachValue(current_speed_left, decel_speed, acceleration * 2.0f);
            current_speed_right = approachValue(current_speed_right, 
                                              target_speed_right * (remaining_ratio / (1.0f - deceleration_start)), 
                                              acceleration * 2.0f);
        }
        
        // P REGULÁTOR - vyrovnávání rychlosti motorů
        int error = abs(left_pos) - abs(right_pos);
        float correction = error * m_kp;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
        // Aplikace korekce
        float speed_left_corrected = current_speed_left;
        float speed_right_corrected = current_speed_right;
        
        if (error > 0) {
            // Levý je napřed - zpomalit levý, zrychlit pravý
            if (m_polarity_switch_left) {
                speed_left_corrected += correction;
            } else {
                speed_left_corrected -= correction;
            }
            if (m_polarity_switch_right) {
                speed_right_corrected -= correction;
            } else {
                speed_right_corrected += correction;
            }
        } else if (error < 0) {
            // Pravý je napřed - zpomalit pravý, zrychlit levý
            if (m_polarity_switch_left) {
                speed_left_corrected -= correction;
            } else {
                speed_left_corrected += correction;
            }
            if (m_polarity_switch_right) {
                speed_right_corrected += correction;
            } else {
                speed_right_corrected -= correction;
            }
        }
        
        // Zajištění minimální rychlosti během fáze zrychlení
        if (progress_ratio < deceleration_start && abs(speed_left_corrected) < m_min_speed) {
            speed_left_corrected = (speed_left_corrected > 0) ? m_min_speed : -m_min_speed;
        }
        if (progress_ratio < deceleration_start && abs(speed_right_corrected) < m_min_speed) {
            speed_right_corrected = (speed_right_corrected > 0) ? m_min_speed : -m_min_speed;
        }
        
        // Zajištění minimální rychlosti během zpomalování
        if (progress_ratio >= deceleration_start) {
            if (abs(speed_left_corrected) < m_min_decel_speed && abs(speed_left_corrected) > 0) {
                speed_left_corrected = (speed_left_corrected > 0) ? m_min_decel_speed : -m_min_decel_speed;
            }
            if (abs(speed_right_corrected) < m_min_decel_speed && abs(speed_right_corrected) > 0) {
                speed_right_corrected = (speed_right_corrected > 0) ? m_min_decel_speed : -m_min_decel_speed;
            }
        }
        
        // Nastavení výkonu motorů
        man.motor(m_id_left).power(pctToSpeed(speed_left_corrected));
        man.motor(m_id_right).power(pctToSpeed(speed_right_corrected));
        
        delay(10);
    }
    
    // Konečné zastavení
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
}

// void Motors::turn_on_spot_right(float angle, float speed) {
//     auto& man = rb::Manager::get();
    
//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);

//     int target_ticks = mmToTicks((M_PI * roztec_kol) * (angle / 360.0f));
//     int left_pos = 0;
//     int right_pos = 0;
//     std::cout << "Target ticks: " << target_ticks << std::endl;
//     // Základní rychlosti s přihlédnutím k polaritě
//     float base_speed_left = m_polarity_switch_left ? -speed : speed;
//     float base_speed_right = m_polarity_switch_right ? speed : -speed;

//     float speed_left = base_speed_left;
//     float speed_right = base_speed_right;

//     man.motor(m_id_left).power(pctToSpeed(speed_left));
//     man.motor(m_id_right).power(pctToSpeed(speed_right));

//     while(target_ticks > (abs(left_pos) +50) || target_ticks > (abs(right_pos) + 50)) {
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//              left_pos = info.position();
//           });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//              right_pos = info.position();
//           });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         delay(10);
//     }
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
