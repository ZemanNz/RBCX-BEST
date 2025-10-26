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
    m_wheel_circumference_left = M_PI * cfg.left_wheel_diameter;
    m_wheel_circumference_right = M_PI * cfg.right_wheel_diameter;
    m_max_speed = cfg.motor_max_ticks_per_second;
    prevod_motoru = cfg.prevod_motoru;
    roztec_kol = cfg.roztec_kol;
    rozdil_v_kolech_levy = cfg.rozdil_v_kolech_levy;
    rozdil_v_kolech_pravy = cfg.rozdil_v_kolech_pravy;
    Button1 = cfg.Button1;
    Button2 = cfg.Button2;
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
    // WiFi inicializace
    if (cfg.enable_wifi_log && cfg.wifi_ssid && cfg.wifi_password) {
        initWifi(cfg.wifi_ssid, cfg.wifi_password);
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


void Motors::drive(float left, float right, float speed_left, float speed_right, dual_callback_t callback) {

    // 1. Ošetření polarity motorů
    if (m_polarity_switch_left)
        left = -left;

    if (m_polarity_switch_right)
        right = -right;

    // 2. Příprava proměnné pro callback, která bude předána do řídicích funkcí motorů
    rb::Motor::callback_t cb;

    if (callback) {
        std::lock_guard<std::mutex> lock(m_dual_callbacks_mu);
        auto itr = m_dual_callbacks.emplace(m_dual_callbacks.end(), DualCb(std::move(callback)));

        cb = [this, itr](rb::Motor& m) {
            std::unique_lock<std::mutex> lock(m_dual_callbacks_mu);
            if (!itr->completed && ++itr->count >= 1) {
                itr->completed = true;
                dual_callback_t localCb;
                localCb.swap(itr->final_cb);
                m_dual_callbacks.erase(itr);
                lock.unlock();
                Serial.println("Callback odblokovan motorama.");
                localCb();
            }
        };

        // 3. Timeout – připravíme monitor, který zajistí spuštění callbacku, pokud se neprovede včas
        float expectedTimeLeft = 10000;
        float expectedTimeRight = 10000;
        float maxExpectedTime = std::max(expectedTimeLeft, expectedTimeRight);
        const float timeoutSec = maxExpectedTime * 1.37f;

        std::thread([this, itr, timeoutSec]() {
            unsigned long startTime = millis();
            while ((millis() - startTime) < (timeoutSec * 1000)) {
                {
                    std::lock_guard<std::mutex> lock(m_dual_callbacks_mu);
                    if (itr->completed)
                        return;
                }
                delay(10);
            }

            // Po vypršení timeoutu zkontrolujeme znovu
            std::unique_lock<std::mutex> lock(m_dual_callbacks_mu);
            if (!itr->completed) {
                itr->completed = true;
                dual_callback_t localCb;
                localCb.swap(itr->final_cb);
                m_dual_callbacks.erase(itr);
                lock.unlock();
                Serial.println("Timeout expiroval, callback odblokovan timeoutem.");
                localCb();
            }
        }).detach();
    }

    // 4. Získání referencí na motory
    auto& ml = rb::Manager::get().motor(m_id_left);
    auto& mr = rb::Manager::get().motor(m_id_right);

    // 5. Načtení informací o levém motoru (není třeba callback)
    ml.requestInfo(nullptr);

    // 6. Načtení informací o pravém motoru a nastavení pohybu motorů
    mr.requestInfo([this, left, right, speed_left, speed_right, cb](rb::Motor& mr) {
        auto& ml = rb::Manager::get().motor(m_id_left);

        rb::Manager::get()
            .setMotors()
            .driveToValue(m_id_left, ml.position() + mmToTicks(left), pctToSpeed(speed_left), cb)
            .driveToValue(m_id_right, mr.position() + mmToTicks(right), pctToSpeed(speed_right), cb)
            .set();
    });

    Serial.printf("Leva rychlost: %d, Prava rychlost: %d\n", pctToSpeed(speed_left), pctToSpeed(speed_right));
    Serial.printf("Leva vzdalenost: %d, Prava vzdalenost: %d\n", mmToTicks(left), mmToTicks(right));
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

// void Motors::forward(float mm, float speed) {
//     auto& man = rb::Manager::get();
    
//     float m_kp = 0.23f; // Proporcionální konstanta
//     float m_min_speed = 20.0f; // Minimální rychlost motorů
//     float m_max_correction = 10.0f; // Maximální korekce rychlosti
//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
//     int target_ticks = mmToTicks(mm);
//     int left_pos = 0;
//     int right_pos = 0;
//     float pocet_mereni=0.0f;
//     float soucet_error=0.0f;
//     float integral=0.0f;
//     std::cout << "Target ticks: " << target_ticks << std::endl;
//     // Základní rychlosti s přihlédnutím k polaritě
//     float base_speed_left = m_polarity_switch_left ? -speed : speed;
//     float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
//     unsigned long start_time = millis();
//     int timeoutMs = timeout_ms(mm, speed);
    
//     while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
//           (millis() - start_time < timeoutMs)) {
        
//         // Čtení pozic
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//              left_pos = info.position();
//           });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//              right_pos = info.position();
//           });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         // P regulátor - pracuje s absolutními hodnotami pozic
//         int error = abs(left_pos) - abs(right_pos);
//         pocet_mereni += 1.0f;
//         soucet_error += error;
//         integral = (soucet_error / pocet_mereni);
//         std::cout << "Průměrná chyba: " << (integral) << " ticků" << std::endl;
//         error = integral + error; // Použití průměrné chyby pro korekci
//         float correction = error * m_kp;
//         correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
//         // Výpočet korigovaných rychlostí
//         float speed_left = base_speed_left;
//         float speed_right = base_speed_right;
        
//         // Aplikace korekce podle polarity
//         if (error > 0) {
//             // Levý je napřed - zpomalit levý
//             if (m_polarity_switch_left) {
//                 speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
//             } else {
//                 speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
//             }
//         } else if (error < 0) {
//             // Pravý je napřed - zpomalit pravý
//             if (m_polarity_switch_right) {
//                 speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
//             } else {
//                 speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
//             }
//         }
        
//         // Zajištění minimální rychlosti
//         if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
//             speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
//         }
//         if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
//             speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
//         }
        
//         // Nastavení výkonu motorů
//         man.motor(m_id_left).power(pctToSpeed(speed_left * rozdil_v_kolech_levy));
//         man.motor(m_id_right).power(pctToSpeed(speed_right * rozdil_v_kolech_pravy));
//         std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
//         delay(10);
//     }
    
//     // Zastavení motorů
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }
// void Motors::forward(float mm, float speed) {
//     auto& man = rb::Manager::get();
    
//     float m_kp = 0.23f; // Proporcionální konstanta
//     float m_min_speed = 20.0f; // Minimální rychlost motorů
//     float m_max_correction = 10.0f; // Maximální korekce rychlosti
//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
//     int target_ticks_left = mmToTicks_left(mm);
//     int target_ticks_right = mmToTicks_right(mm);
//     int left_pos = 0;
//     int right_pos = 0;
//     float pocet_mereni=0.0f;
//     float soucet_error=0.0f;
//     float integral=0.0f;
//     std::cout << "Target ticks left: " << target_ticks_left << "tick right" << target_ticks_right << std::endl;
//     print_wifi("Target ticks left: " + String(target_ticks_left) + " tick right: " + String(target_ticks_right) + "\n");
//     // Základní rychlosti s přihlédnutím k polaritě
//     float base_speed_left = m_polarity_switch_left ? -speed : speed;
//     float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
//     unsigned long start_time = millis();
//     int timeoutMs = timeout_ms(mm, speed);
    
//     while((target_ticks_left > abs(left_pos) || target_ticks_right> abs(right_pos)) && 
//           (millis() - start_time < timeoutMs)) {
        
//         // Čtení pozic
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//              left_pos = info.position();
//           });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//              right_pos = info.position();
//           });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         print_wifi("Left pos: " + String(left_pos) + " Right pos: " + String(right_pos) + "\n");
//         // P regulátor - pracuje s absolutními hodnotami pozic
//         int error = abs(left_pos * rozdil_v_kolech_pravy) - abs(right_pos * rozdil_v_kolech_levy);
//         std::cout << "Error: " << error << " ticků" << std::endl;
//         print_wifi("Error: " + String(error) + " ticků\n");
//         pocet_mereni += 1.0f;
//         soucet_error += error;
//         integral = (soucet_error / pocet_mereni);
//         std::cout << "Průměrná chyba: " << (integral) << " ticků" << std::endl;
//         print_wifi("Průměrná chyba: " + String(integral) + " ticků\n");
//         error = integral + error; // Použití průměrné chyby pro korekci
//         float correction = error * m_kp;
//         correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
//         // Výpočet korigovaných rychlostí
//         float speed_left = base_speed_left;
//         float speed_right = base_speed_right;
        
//         // Aplikace korekce podle polarity
//         if (error > 0) {
//             // Levý je napřed - zpomalit levý
//             if (m_polarity_switch_left) {
//                 speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
//             } else {
//                 speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
//             }
//         } else if (error < 0) {
//             // Pravý je napřed - zpomalit pravý
//             if (m_polarity_switch_right) {
//                 speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
//             } else {
//                 speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
//             }
//         }
        
//         // Zajištění minimální rychlosti
//         if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
//             speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
//         }
//         if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
//             speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
//         }
        
//         // Nastavení výkonu motorů
//         man.motor(m_id_left).power(pctToSpeed(speed_left * rozdil_v_kolech_levy));
//         man.motor(m_id_right).power(pctToSpeed(speed_right * rozdil_v_kolech_pravy));
//         std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
//         print_wifi("Speed left: " + String(speed_left) + " Speed right: " + String(speed_right) + "\n");
//         delay(10);
//     }
    
//     // Zastavení motorů
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }


//     auto& man = rb::Manager::get();
    
//     float m_kp = 0.23f; // Proporcionální konstanta
//     float m_min_speed = 20.0f; // Minimální rychlost motorů
//     float m_max_correction = 10.0f; // Maximální korekce rychlosti
//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
//     int target_ticks = mmToTicks(mm);
//     int left_pos = 0;
//     int right_pos = 0;
//     float pocet_mereni=0.0f;
//     float soucet_error=0.0f;
//     float integral=0.0f;
//     std::cout << "Target ticks: " << target_ticks << std::endl;
//     // Základní rychlosti s přihlédnutím k polaritě
//     float base_speed_left = m_polarity_switch_left ? -speed : speed;
//     float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
//     unsigned long start_time = millis();
//     int timeoutMs = timeout_ms(mm, speed);
    
//     while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
//           (millis() - start_time < timeoutMs)) {
        
//         // Čtení pozic
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//              left_pos = info.position();
//           });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//              right_pos = info.position();
//           });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         // P regulátor - pracuje s absolutními hodnotami pozic
//         int error = abs(left_pos) - abs(right_pos);
//         pocet_mereni += 1.0f;
//         soucet_error += error;
//         integral = (soucet_error / pocet_mereni);
//         std::cout << "Průměrná chyba: " << (integral) << " ticků" << std::endl;
//         error = integral + error; // Použití průměrné chyby pro korekci
//         float correction = error * m_kp;
//         correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
//         // Výpočet korigovaných rychlostí
//         float speed_left = base_speed_left;
//         float speed_right = base_speed_right;
        
//         // Aplikace korekce podle polarity
//         if (error > 0) {
//             // Levý je napřed - zpomalit levý
//             if (m_polarity_switch_left) {
//                 speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
//             } else {
//                 speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
//             }
//         } else if (error < 0) {
//             // Pravý je napřed - zpomalit pravý
//             if (m_polarity_switch_right) {
//                 speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
//             } else {
//                 speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
//             }
//         }
        // void Motors::forward(float mm, float speed) {
//     auto& man = rb::Manager::get();
    
//     float m_kp = 0.23f; // Proporcionální konstanta
//     float m_min_speed = 20.0f; // Minimální rychlost motorů
//     float m_max_correction = 10.0f; // Maximální korekce rychlosti
//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
//     int target_ticks = mmToTicks(mm);
//     int left_pos = 0;
//     int right_pos = 0;
//     float pocet_mereni=0.0f;
//     float soucet_error=0.0f;
//     float integral=0.0f;
//     std::cout << "Target ticks: " << target_ticks << std::endl;
//     // Základní rychlosti s přihlédnutím k polaritě
//     float base_speed_left = m_polarity_switch_left ? -speed : speed;
//     float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
//     unsigned long start_time = millis();
//     int timeoutMs = timeout_ms(mm, speed);
    
//     while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
//           (millis() - start_time < timeoutMs)) {
        
//         // Čtení pozic
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//              left_pos = info.position();
//           });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//              right_pos = info.position();
//           });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         // P regulátor - pracuje s absolutními hodnotami pozic
//         int error = abs(left_pos) - abs(right_pos);
//         pocet_mereni += 1.0f;
//         soucet_error += error;
//         integral = (soucet_error / pocet_mereni);
//         std::cout << "Průměrná chyba: " << (integral) << " ticků" << std::endl;
//         error = integral + error; // Použití průměrné chyby pro korekci
//         float correction = error * m_kp;
//         correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
//         // Výpočet korigovaných rychlostí
//         float speed_left = base_speed_left;
//         float speed_right = base_speed_right;
        
//         // Aplikace korekce podle polarity
//         if (error > 0) {
//             // Levý je napřed - zpomalit levý
//             if (m_polarity_switch_left) {
//                 speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
//             } else {
//                 speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
//             }
//         } else if (error < 0) {
//             // Pravý je napřed - zpomalit pravý
//             if (m_polarity_switch_right) {
//                 speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
//             } else {
//                 speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
//             }
//         }
        
//         // Zajištění minimální rychlosti
//         if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
//             speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
//         }
//         if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
//             speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
//         }
        
//         // Nastavení výkonu motorů// void Motors::forward(float mm, float speed) {
//     auto& man = rb::Manager::get();
    
//     float m_kp = 0.23f; // Proporcionální konstanta
//     float m_min_speed = 20.0f; // Minimální rychlost motorů
//     float m_max_correction = 10.0f; // Maximální korekce rychlosti
//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
//     int target_ticks = mmToTicks(mm);
//     int left_pos = 0;
//     int right_pos = 0;
//     float pocet_mereni=0.0f;
//     float soucet_error=0.0f;
//     float integral=0.0f;
//     std::cout << "Target ticks: " << target_ticks << std::endl;
//     // Základní rychlosti s přihlédnutím k polaritě
//     float base_speed_left = m_polarity_switch_left ? -speed : speed;
//     float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
//     unsigned long start_time = millis();
//     int timeoutMs = timeout_ms(mm, speed);
    
//     while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
//           (millis() - start_time < timeoutMs)) {
        
//         // Čtení pozic
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//              left_pos = info.position();
//           });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//              right_pos = info.position();
//           });
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         // P regulátor - pracuje s absolutními hodnotami pozic
//         int error = abs(left_pos) - abs(right_pos);
//         pocet_mereni += 1.0f;
//         soucet_error += error;
//         integral = (soucet_error / pocet_mereni);
//         std::cout << "Průměrná chyba: " << (integral) << " ticků" << std::endl;
//         error = integral + error; // Použití průměrné chyby pro korekci
//         float correction = error * m_kp;
//         correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
//         // Výpočet korigovaných rychlostí
//         float speed_left = base_speed_left;
//         float speed_right = base_speed_right;
        
//         // Aplikace korekce podle polarity
//         if (error > 0) {
//             // Levý je napřed - zpomalit levý
//             if (m_polarity_switch_left) {
//                 speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
//             } else {
//                 speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
//             }
//         } else if (error < 0) {
//             // Pravý je napřed - zpomalit pravý
//             if (m_polarity_switch_right) {
//                 speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
//             } else {
//                 speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
//             }
//         }
        
//         // Zajištění minimální rychlosti
//         if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
//             speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
//         }
//         if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
//             speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
//         }
        
//         // Nastavení výkonu motorů
//         man.motor(m_id_left).power(pctToSpeed(speed_left * rozdil_v_kolech_levy));
//         man.motor(m_id_right).power(pctToSpeed(speed_right * rozdil_v_kolech_pravy));
//         std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
//         delay(10);
//     }
    
//     // Zastavení motorů
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }
//         man.motor(m_id_left).power(pctToSpeed(speed_left * rozdil_v_kolech_levy));
//         man.motor(m_id_right).power(pctToSpeed(speed_right * rozdil_v_kolech_pravy));
//         std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
//         delay(10);
//     }
    
//     // Zastavení motorů
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }
//         // Zajištění minimální rychlosti
//         if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
//             speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
//         }
//         if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
//             speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
//         }
        
//         // Nastavení výkonu motorů
//         man.motor(m_id_left).power(pctToSpeed(speed_left * rozdil_v_kolech_levy));
//         man.motor(m_id_right).power(pctToSpeed(speed_right * rozdil_v_kolech_pravy));
//         std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
//         delay(10);
//     }
    
//     // Zastavení motorů
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
// }ní korekce rychlosti
void Motors::forward(float mm, float speed) {
    auto& man = rb::Manager::get();
    
    float m_kp = 0.23f; // Proporcionální konstanta
    float m_min_speed = 20.0f; // Minimální rychlost motorů
    float m_max_correction = 8.5f; // Maximál// void Motors::forward(float mm, float speed) {
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);

    int target_ticks_left = mmToTicks_left(mm);
    int target_ticks_right = mmToTicks_right(mm);
    float left_pos = 0;
    float right_pos = 0;
    float pocet_mereni=0.0f;
    float soucet_error=0.0f;
    float integral=0.0f;
    float progres_left = 0.0f;
    float progres_right = 0.0f;
    float rozdil_progres = 0.0f;
    std::cout << "Target ticks left: " << target_ticks_left << " tick right" << target_ticks_right << std::endl;
    // Základní rychlosti s přihlédnutím k polaritě
    float base_speed_left = m_polarity_switch_left ? -speed : speed;
    float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
    unsigned long start_time = millis();
    int timeoutMs = timeout_ms(mm, speed);
    
    while((target_ticks_left > abs(left_pos) || target_ticks_right > abs(right_pos)) && 
          (millis() - start_time < timeoutMs)) {
        
        // Čtení pozic
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });

        std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
        print_wifi("Left pos: " + String(left_pos) + " Right pos: " + String(right_pos) + "\n");
        // P regulátor - pracuje s absolutními hodnotami pozic
        progres_left = (float(abs(left_pos)) / float(target_ticks_left));
        progres_right = (float(abs(right_pos)) / float(target_ticks_right));
        rozdil_progres = progres_left - progres_right;

        float error = abs(left_pos) - abs(right_pos)  ;
        std::cout << "Error: " << error << " ticků" << std::endl;
        print_wifi("Error: " + String(error) + " ticků\n");

        float correction = rozdil_progres * m_kp * 1800;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        std::cout << "Progres L: " << progres_left << ", Progres R: " << progres_right << ", Diff: " << rozdil_progres << ", Correction: " << correction << std::endl;
        // Výpočet korigovaných rychlostí
        float speed_left = base_speed_left;
        float speed_right = base_speed_right;
        
        // Aplikace korekce podle polarity
        if (error > 0) {
            // Levý je napřed - zpomalit levý
            if (m_polarity_switch_left) {
                speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
                speed_right += correction;  // Odečíst od kladné rychlosti = zrychlit
            } else {
                speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
                speed_right -= correction;  // Přidat ke kladné rychlosti = zrychlit
            }
        } else if (error < 0) {
            // Pravý je napřed - zpomalit pravý
            if (m_polarity_switch_right) {
                speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
                speed_left -= correction;  // Přidat k záporné rychlosti = zrychlit
            } else {
                speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
                speed_left += correction;  // Odečíst od kladné rychlosti = zrychlit
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
        man.motor(m_id_left).power(pctToSpeed(speed_left ));
        man.motor(m_id_right).power(pctToSpeed(speed_right ));
        std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
        print_wifi("Speed left: " + String(speed_left) + " Speed right: " + String(speed_right) + "\n");
        delay(10);
    }
    
    // Zastavení motorů
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
}

// void Motors::forward(float mm, float speed) {
//     auto& man = rb::Manager::get();
    
//     // Nastavení PID konstant
//     float m_kp = 0.15f;  // Zvýšeno z 0.23
//     float m_ki = 0.14f; // Integrační konstanta
//     float m_kd = 0.10f;  // Derivační konstanta
//     float m_min_speed = 20.0f;
//     float m_max_correction = 20.0f; // Zvýšeno z 10.0

//     // Reset pozic
//     man.motor(m_id_left).setCurrentPosition(0);
//     man.motor(m_id_right).setCurrentPosition(0);
    
//     int target_ticks_left = mmToTicks_left(mm);
//     int target_ticks_right = mmToTicks_right(mm);
//     int left_pos = 0;
//     int right_pos = 0;
    
//     // Proměnné pro PID
//     int last_error = 0;
//     float integral = 0.0f;
    
//     printf_wifi("🎯 Start forward - %.1f mm, %.1f%% speed", mm, speed);
//     printf_wifi("🎯 Target ticks - L: %d, R: %d", target_ticks_left, target_ticks_right);
//     std::cout << "Target ticks left: " << target_ticks_left << " tick right" << target_ticks_right << std::endl;
//     // Základní rychlosti s polaritou
//     float base_speed_left = speed;
//     float base_speed_right = speed;
//     if (m_polarity_switch_left) base_speed_left = -base_speed_left;
//     if (m_polarity_switch_right) base_speed_right = -base_speed_right;
    
//     unsigned long start_time = millis();
//     int timeoutMs = timeout_ms(mm, speed);
//     float pomer_ujeti_levy;
//     float pomer_ujeti_pravy;
//     while((target_ticks_left > abs(left_pos) || target_ticks_right > abs(right_pos)) && 
//           (millis() - start_time < timeoutMs)) {
        
//         // Čtení pozic
//         man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
//             left_pos = info.position();
//         });
//         man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
//             right_pos = info.position();
//         });
        
//         pomer_ujeti_levy = float(abs(left_pos)) / float(target_ticks_left);
//         pomer_ujeti_pravy = float(abs(right_pos)) / float(target_ticks_right);

//         float kdo_ujel_vice_procent = (pomer_ujeti_levy - pomer_ujeti_pravy); // Korekční faktor na základě rozdílu ujeté vzdálenosti
//         float korekce_l = (1.0 - kdo_ujel_vice_procent);
//         float korekce_p = (1.0 + kdo_ujel_vice_procent);

//         print_wifi(" L: " + String(pomer_ujeti_levy) + ", R: " + String(pomer_ujeti_pravy) + ", Diff: " + String(kdo_ujel_vice_procent) + "\n");
//         std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
//         std::cout << "Pomer ujeti levy: " << pomer_ujeti_levy << ", pravy: " << pomer_ujeti_pravy << ", Diff: " << kdo_ujel_vice_procent << std::endl;
//         // Výpočet erroru s ohledem na polaritu
//         int left_abs = m_polarity_switch_left ? -left_pos : left_pos;
//         int right_abs = m_polarity_switch_right ? -right_pos : right_pos;
//         int error = abs(left_abs) - abs(right_abs);
        
//         // PID výpočet
//         integral += error;
//         float derivative = error - last_error;
//         float correction = m_kp * error + m_ki * integral + m_kd * derivative;
//         last_error = error;
        
//         // Omezení korekce
//         correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
//         //printf_wifi("📊 L: %d, R: %d, Err: %d, Corr: %.1f", left_abs, right_abs, error, correction);
        
//         // Výpočet rychlostí
//         float speed_left = base_speed_left;
//         float speed_right = base_speed_right;
        
//         // if (error > 0) {
//         //     // Levý napřed - zpomalit levý
//         //     speed_left -= correction;
//         // } else if (error < 0) {
//         //     // Pravý napřed - zpomalit pravý
//         //     speed_right -= correction;
//         // }
        
//         // Minimální rychlost
//         if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
//             speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
//         }
//         if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
//             speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
//         }
        
//         // Aplikace korekčních faktorů kol
//         float final_speed_left = speed_left - korekce_l *4;
//         float final_speed_right = speed_right - korekce_p *4;
        
//         // Nastavení motorů
//         man.motor(m_id_left).power(pctToSpeed(final_speed_left));
//         man.motor(m_id_right).power(pctToSpeed(final_speed_right));
        
//         printf_wifi("🎛️  L: %.1f, R: %.1f", final_speed_left, final_speed_right);
//         std::cout << "Speed left: " << final_speed_left << ", Speed right: " << final_speed_right << std::endl;
        
//         delay(10);
//     }
    
//     // Zastavení motorů
//     man.motor(m_id_left).power(0);
//     man.motor(m_id_right).power(0);
    
//     printf_wifi("✅ Forward completed - L: %d, R: %d", left_pos, right_pos);
//     std::cout << "Forward completed - L: " << left_pos << ", R: " << right_pos << std::endl;
// }

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
    const unsigned long timeout_ms =10000;
    
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
    const unsigned long timeout_ms = 10000;
    
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

void Motors::radius_right(float radius, float angle, float speed) {
    auto& man = rb::Manager::get();
    
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);

    // Výpočet drah pro zatáčku VPRAVO
    float distance_left = (((radius + roztec_kol) * PI * angle) / 180) * konstanta_radius_vnejsi_kolo;  // vnější kolo
    float distance_right = (( radius * PI * angle) / 180)* konstanta_radius_vnitrni_kolo;               // vnitřní kolo

    int target_ticks_left = mmToTicks(distance_left);
    int target_ticks_right = mmToTicks(distance_right);

    // Základní výpočet rychlostí
    float speed_left = speed;  // vnější kolo
    float speed_right = speed * (radius / (roztec_kol + radius));  // vnitřní kolo
    
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Target ticks:   L=" << target_ticks_left << "  R=" << target_ticks_right << std::endl;

    // Úprava polarity
    if (m_polarity_switch_left)
        speed_left = -speed_left;
    if (m_polarity_switch_right)
        speed_right = -speed_right;
    
    // P regulátor - konstanty
    const float Kp = 1.47f;  // Zkus začít s 0.5 a uprav podle potřeby
    const float max_speed_adjust = 1.9f;  // Maximální úprava rychlosti
    
    man.motor(m_id_left).power(pctToSpeed(speed_left));
    man.motor(m_id_right).power(pctToSpeed(speed_right));

    int timeoutMs = 10000;
    unsigned long start_time = millis();
    bool left_done = false;
    bool right_done = false;

    int left_pos = 0;
    int right_pos = 0;
    float adjusted_speed_left = speed_left;
    float adjusted_speed_right = speed_right;

    while(millis() - start_time < timeoutMs) {
        // Synchronní čtení pozic
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });
        
        // Výpočet progresu (0.0 - 1.0)
        float progress_left = (float)abs(left_pos) / abs(target_ticks_left);
        float progress_right = (float)abs(right_pos) / abs(target_ticks_right);
        
        // Výpis informací o progresu, pozicích a rychlostech
        std::cout << "Left pos: " << left_pos << "/" << target_ticks_left 
                  << " (" << progress_left * 100 << "%), Speed_bace: " << speed_left << " | "
                  << "Right pos: " << right_pos << "/" << target_ticks_right 
                  << " (" << progress_right * 100 << "%), Speed_bace: " << speed_right << " | "
                  << "actually_speed_left : " << adjusted_speed_left  << std::endl;
        
        // P regulátor - upravujeme rychlosti podle rozdílu v progresu
        float progress_error = progress_left - progress_right;
        float speed_adjust = Kp * progress_error;
        
        // Omezení úpravy rychlosti
        if (speed_adjust > max_speed_adjust) speed_adjust = max_speed_adjust;
        if (speed_adjust < -max_speed_adjust) speed_adjust = -max_speed_adjust;
        
        // Upravené rychlosti
        adjusted_speed_left = speed_left;
        adjusted_speed_right = speed_right;
        
        if (!left_done && !right_done) {
            // Pokud vnější kolo (levé) je napřed, zpomalíme ho a/nebo zrychlíme vnitřní
            if (progress_error > 0) {
                adjusted_speed_left = speed_left * (1.0f - abs(speed_adjust));
                adjusted_speed_right = speed_right * (1.0f + abs(speed_adjust));
            }
            // Pokud vnitřní kolo (pravé) je napřed, zpomalíme ho a/nebo zrychlíme vnější
            else if (progress_error < 0) {
                adjusted_speed_left = speed_left * (1.0f + abs(speed_adjust));
                adjusted_speed_right = speed_right * (1.0f - abs(speed_adjust));
            }
            
            // Aplikace upravených rychlostí
            man.motor(m_id_left).power(pctToSpeed(adjusted_speed_left));
            man.motor(m_id_right).power(pctToSpeed(adjusted_speed_right));
        }
        
        // Kontrola dokončení
        if (abs(left_pos)  >= abs(target_ticks_left) && !left_done) {
            left_done = true;
            man.motor(m_id_left).power(0);
        }
        
        if (abs(right_pos) >= abs(target_ticks_right) && !right_done) {
            right_done = true;
            man.motor(m_id_right).power(0);
        }
        
        if (left_done && right_done) {
            break;
        }
        
        delay(10);
    }
    
    // Zastavení motorů
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
    
    std::cout << "Radius right completed!" << std::endl;
}

void Motors::radius_left(float radius, float angle, float speed) {
    auto& man = rb::Manager::get();
    
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);

    // Výpočet drah pro zatáčku VLEVO - POZOR: OPACNE NEZ PRO RIGHT!
    float distance_left = ((radius * PI * angle) / 180) * konstanta_radius_vnitrni_kolo;               // vnitřní kolo ------->> konstanta vnitrniho kola pro radius == 1.0084
    float distance_right = (((radius + roztec_kol) * PI * angle) / 180) * konstanta_radius_vnejsi_kolo;  // vnější kolo ------->> konstanta vnejsiho kola pro radius == 1.035

    int target_ticks_left = mmToTicks(distance_left);
    int target_ticks_right = mmToTicks(distance_right);

    // Základní výpočet rychlostí PRO ZATÁČKU VLEVO
    float speed_left = speed * (radius / (roztec_kol + radius));  // vnitřní kolo
    float speed_right = speed;  // vnější kolo
    
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Target ticks:   L=" << target_ticks_left << "  R=" << target_ticks_right << std::endl;

    // Úprava polarity
    if (m_polarity_switch_left)
        speed_left = -speed_left;
    if (m_polarity_switch_right)
        speed_right = -speed_right;
    
    // P regulátor - konstanty (stejné jako pro right)
    const float Kp = 1.47f;
    const float max_speed_adjust = 1.9f;
    
    man.motor(m_id_left).power(pctToSpeed(speed_left));
    man.motor(m_id_right).power(pctToSpeed(speed_right));

    int timeoutMs = 10000;
    unsigned long start_time = millis();
    bool left_done = false;
    bool right_done = false;

    int left_pos = 0;
    int right_pos = 0;
    float adjusted_speed_left = speed_left;
    float adjusted_speed_right = speed_right;

    while(millis() - start_time < timeoutMs) {
        // Synchronní čtení pozic
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });
        
        // Výpočet progresu (0.0 - 1.0)
        float progress_left = (float)abs(left_pos) / abs(target_ticks_left);
        float progress_right = (float)abs(right_pos) / abs(target_ticks_right);
        
        // Výpis informací o progresu, pozicích a rychlostech
        std::cout << "Left pos: " << left_pos << "/" << target_ticks_left 
                  << " (" << progress_left * 100 << "%), Speed_bace: " << speed_left << " | "
                  << "Right pos: " << right_pos << "/" << target_ticks_right 
                  << " (" << progress_right * 100 << "%), Speed_bace: " << speed_right << " | "
                  << "actually_speed_left : " << adjusted_speed_left  << std::endl;
        
        // P regulátor - upravujeme rychlosti podle rozdílu v progresu
        float progress_error = progress_left - progress_right;
        float speed_adjust = Kp * progress_error;
        
        // Omezení úpravy rychlosti
        if (speed_adjust > max_speed_adjust) speed_adjust = max_speed_adjust;
        if (speed_adjust < -max_speed_adjust) speed_adjust = -max_speed_adjust;
        
        // Upravené rychlosti
        adjusted_speed_left = speed_left;
        adjusted_speed_right = speed_right;
        
        if (!left_done && !right_done) {
            // Pokud levé kolo (vnitřní) je napřed, zpomalíme ho a zrychlíme pravé (vnější)
            if (progress_error > 0) {
                adjusted_speed_left = speed_left * (1.0f - abs(speed_adjust));
                adjusted_speed_right = speed_right * (1.0f + abs(speed_adjust));
            }
            // Pokud pravé kolo (vnější) je napřed, zpomalíme ho a zrychlíme levé (vnitřní)
            else if (progress_error < 0) {
                adjusted_speed_left = speed_left * (1.0f + abs(speed_adjust));
                adjusted_speed_right = speed_right * (1.0f - abs(speed_adjust));
            }
            
            // Aplikace upravených rychlostí
            man.motor(m_id_left).power(pctToSpeed(adjusted_speed_left));
            man.motor(m_id_right).power(pctToSpeed(adjusted_speed_right));
        }
        
        // Kontrola dokončení
        if (abs(left_pos) >= abs(target_ticks_left) && !left_done) {
            left_done = true;
            man.motor(m_id_left).power(0);
        }
        
        if (abs(right_pos) >= abs(target_ticks_right) && !right_done) {
            right_done = true;
            man.motor(m_id_right).power(0);
        }
        
        if (left_done && right_done) {
            break;
        }
        
        delay(10);
    }
    
    // Zastavení motorů
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
    
    std::cout << "Radius left completed!" << std::endl;
}

void Motors::forward_acc(float mm, float speed) {
    auto& man = rb::Manager::get();
    
    float m_kp = 0.18f;
    float m_min_speed = 18.0f; // ZVÝŠENO NA 18%
    float m_max_correction = 7.0f;
    
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);
    
    int target_ticks = mmToTicks(mm);
    int left_pos = 0;
    int right_pos = 0;
    float pocet_mereni=0.0f;
    float soucet_error=0.0f;
    float integral=0.0f;
    // Proměnné pro plynulé zrychlení/zpomalení
    float current_speed_left = 0;
    float current_speed_right = 0;
    const float acceleration = 1.5f;
    const float acceleration_phase = 0.3f;
    float deceleration_start;
    if(mm > 1000){
        deceleration_start = 0.75f; // Pozdější zpomalování pro delší vzdálenosti
    } else {
        deceleration_start = 0.6f; // Dřívější zpomalování pro kratší vzdálenosti
    }
    
    // Základní rychlosti s přihlédnutím k polaritě
    float target_speed_left = m_polarity_switch_left ? -speed : speed;
    float target_speed_right = m_polarity_switch_right ? -speed : speed;
    
    unsigned long start_time = millis();
    int timeoutMs = timeout_ms(mm, speed) * 2;
    
    std::cout << "🎯 forward_acc START - " << mm << "mm, " << speed << "%, cílových " << target_ticks << " ticků" << std::endl;
    std::cout << "🎯 Target speeds - L: " << target_speed_left << "%, R: " << target_speed_right << "%" << std::endl;
    
    while((target_ticks > abs(left_pos) || target_ticks > abs(right_pos)) && 
          (millis() - start_time < timeoutMs)) {
        
        // Čtení pozic
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        
        
        // Výpočet pokroku
        float progress = (abs(left_pos) + abs(right_pos)) / 2.0f;
        float progress_ratio = progress / target_ticks;
        
        // PODMÍNKA PRO BRZKÉ UKONČENÍ
        if (progress_ratio > 0.998f) {
            std::cout << "🎯 BRZKÉ UKONČENÍ - pokrok: " << (progress_ratio * 100) << "%" << std::endl;
            break;
        }
        
        // PLYNULÉ ZRYCHLENÍ A ZPOMALENÍ
        if (progress_ratio < acceleration_phase) {
            // FÁZE ZRYCHLENÍ (0-30%)
            current_speed_left = approachValue(current_speed_left, target_speed_left, acceleration);
            current_speed_right = approachValue(current_speed_right, target_speed_right, acceleration);
            std::cout << "⬆️ ZRYCHLENÍ - " << (progress_ratio * 100) << "% - L: " << current_speed_left << "%, R: " << current_speed_right << "%" << std::endl;
        } 
        else if (progress_ratio < deceleration_start) {
            // FÁZE KONSTANTNÍ RYCHLOSTI (30-80%)
            current_speed_left = target_speed_left;
            current_speed_right = target_speed_right;
            std::cout << "⚡ KONSTANTNÍ - " << (progress_ratio * 100) << "% - L: " << current_speed_left << "%, R: " << current_speed_right << "%" << std::endl;
        } 
        else {
            // FÁZE ZPOMALENÍ (80-100%)
            float remaining = target_ticks - progress;
            float total_decel_distance = target_ticks * (1.0f - deceleration_start);
            
            // VÝPOČET ZPOMALOVACÍ RYCHLOSTI - lineární pokles
            float decel_ratio = remaining / total_decel_distance;
            
            // ODDĚLENÉ VÝPOČTY PRO KAŽDÝ MOTOR PODLE JEHO POLARITY
            float decel_speed_left = target_speed_left * decel_ratio;
            float decel_speed_right = target_speed_right * decel_ratio;
            
            // ZAJIŠTĚNÍ MINIMÁLNÍ RYCHLOSTI - PRO KAŽDÝ MOTOR ZVLÁŠŤ
            if (abs(decel_speed_left) < m_min_speed && abs(decel_speed_left) > 0) {
                decel_speed_left = (decel_speed_left > 0) ? m_min_speed : -m_min_speed;
            }
            if (abs(decel_speed_right) < m_min_speed && abs(decel_speed_right) > 0) {
                decel_speed_right = (decel_speed_right > 0) ? m_min_speed : -m_min_speed;
            }
            
            // ODDĚLENÉ PŘIŘAZENÍ PRO KAŽDÝ MOTOR
            current_speed_left = approachValue(current_speed_left, decel_speed_left, acceleration);
            current_speed_right = approachValue(current_speed_right, decel_speed_right, acceleration);
            
            std::cout << "⬇️ ZPOMALENÍ - " << (progress_ratio * 100) << "% - Cílové rychlosti: L: " << decel_speed_left << "%, R: " << decel_speed_right << "%" << std::endl;
        }
        
        // P REGULÁTOR - vyrovnávání rychlosti motorů
        int error = abs(left_pos) - abs(right_pos);
        float correction = error * m_kp;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        
        // Aplikace korekce - POUZE PŘI DOSTATEČNÉ RYCHLOSTI A PŘED ZPOMALOVÁNÍM
        float speed_left_corrected = current_speed_left;
        float speed_right_corrected = current_speed_right;
        
        // OMEZENÍ P REGULÁTORU - aplikujeme pouze když:
        // 1. Nejsme ve fázi zpomalování
        // 2. Rychlost je dostatečně vysoká (alespoň 25% cílové rychlosti)
        // 3. Chyba je významná (alespoň 5 ticků)
        bool should_apply_correction = (progress_ratio < (deceleration_start + 0.1f)) && 
                                      (abs(current_speed_left) > abs(target_speed_left) * 0.2f) &&
                                      (abs(current_speed_right) > abs(target_speed_right) * 0.2f);
        
        if (should_apply_correction) {
            pocet_mereni++;
            soucet_error += error;
            integral = (soucet_error / pocet_mereni);
            std::cout << "Průměrná chyba: " << (integral) << " ticků" << std::endl;
            error = integral + error; // Použití průměrné chyby pro korekci
            if (error > 0) {
                // Levý je napřed - zpomalit levý
                if (m_polarity_switch_left) {
                    speed_left_corrected += correction;
                } else {
                    speed_left_corrected -= correction;
                }
                std::cout << "🔄 KOREKCE - Levý napřed o " << error << " ticků, korekce: " << correction << "%" << std::endl;
            } else if (error < 0) {
                // Pravý je napřed - zpomalit pravý
                if (m_polarity_switch_right) {
                    speed_right_corrected -= correction;
                } else {
                    speed_right_corrected += correction;
                }
                std::cout << "🔄 KOREKCE - Pravý napřed o " << abs(error) << " ticků, korekce: " << correction << "%" << std::endl;
            } 
        }      
        // Zajištění minimální rychlosti
        if (abs(speed_left_corrected) < m_min_speed && abs(speed_left_corrected) > 0) {
            speed_left_corrected = (speed_left_corrected > 0) ? m_min_speed : -m_min_speed;
            std::cout << "📏 MINIMÁLNÍ RYCHLOST LEVÝ: " << speed_left_corrected << "%" << std::endl;
        }
        if (abs(speed_right_corrected) < m_min_speed && abs(speed_right_corrected) > 0) {
            speed_right_corrected = (speed_right_corrected > 0) ? m_min_speed : -m_min_speed;
            std::cout << "📏 MINIMÁLNÍ RYCHLOST PRAVÝ: " << speed_right_corrected << "%" << std::endl;
        }
        
        // Nastavení výkonu motorů s korekčními faktory
        man.motor(m_id_left).power(pctToSpeed(speed_left_corrected * rozdil_v_kolech_levy));
        man.motor(m_id_right).power(pctToSpeed(speed_right_corrected * rozdil_v_kolech_pravy));
        
        std::cout << "📊 POZICE - L: " << left_pos << " ticků, R: " << right_pos << " ticků, Pokrok: " << (progress_ratio * 100) << "%" << std::endl;
        std::cout << "🎛️  VÝKON - L: " << (speed_left_corrected * rozdil_v_kolech_levy) << "%, R: " << (speed_right_corrected * rozdil_v_kolech_pravy) << "%" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        
        delay(15);
    }
    
    // Konečné zastavení
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
    
    std::cout << "✅ forward_acc UKONČENO - Pozice L: " << left_pos << ", R: " << right_pos << ", Cíl: " << target_ticks << std::endl;
}

void Motors::backward_acc(float mm, float speed) {
    forward_acc(-mm, speed);
}

void Motors::back_buttons(float speed) {
    auto& man = rb::Manager::get();
    
    float m_kp = 0.23f; // Proporcionální konstanta
    float m_min_speed = 20.0f; // Minimální rychlost motorů
    float m_max_correction = 10.0f; // Maximální korekce rychlosti
    
    float pocet_mereni=0.0f;
    float soucet_error=0.0f;
    float integral=0.0f;
    
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);
    
    int left_pos = 0;
    int right_pos = 0;
    // Základní rychlosti s přihlédnutím k polaritě
    float base_speed_left = m_polarity_switch_left ? speed : -speed;
    float base_speed_right = m_polarity_switch_right ? speed : -speed;
    
    unsigned long start_time = millis();
    int timeoutMs = 10000;
    
    while(((digitalRead(Button1) == LOW) || (digitalRead(Button2) == LOW)) && (millis() - start_time < timeoutMs)) {
        
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
        pocet_mereni += 1.0;
        soucet_error += error;
        integral = soucet_error/pocet_mereni;
        
        float correction = (error + integral) * m_kp;
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
        man.motor(m_id_left).power(pctToSpeed(speed_left * rozdil_v_kolech_levy));
        man.motor(m_id_right).power(pctToSpeed(speed_right * rozdil_v_kolech_pravy));
        std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
        delay(10);
        if(digitalRead(Button1) == LOW ) {
            std::cout << "TLACITKO 1 STISKNUTO" << std::endl;
            start_time = millis();
            timeoutMs = 2000;
        }
        if(digitalRead(Button2) == LOW ) {
            std::cout << "TLACITKO 2 STISKNUTO" << std::endl;
            start_time = millis();
            timeoutMs = 2000;
        }
    }
    if(digitalRead(Button1) == LOW && digitalRead(Button2) == LOW ) {
        std::cout << "OBE TLACITKA Stisknuta" << std::endl;
    }
    // Zastavení motorů
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
}




void Motors::wall_following(float distance_to_drive, float speed, float distance_of_wall, bool is_wall_on_right,
                   std::function<int()> first_sensor, 
                   std::function<int()> second_sensor) {

    auto& man = rb::Manager::get();
    
    float m_kp = 0.28f; // Proporcionální konstanta
    float m_min_speed = 20.0f; // Minimální rychlost motorů
    float m_max_correction = 8.0f; // Maximální korekce rychlosti
    
    float pocet_mereni=0.0f;
    float soucet_error=0.0f;
    float integral=0.0f;
    
    // Reset pozic
    man.motor(m_id_left).setCurrentPosition(0);
    man.motor(m_id_right).setCurrentPosition(0);
    
    int left_pos = 0;
    int right_pos = 0;

    float speed_left = 0;
    float speed_right = 0;
    // Základní rychlosti s přihlédnutím k polaritě
    float base_speed_left = m_polarity_switch_left ? -speed : speed;
    float base_speed_right = m_polarity_switch_right ? -speed : speed;
    
    unsigned long start_time = millis();
    int timeoutMs = timeout_ms(distance_to_drive, speed);
    int target_ticks = mmToTicks(distance_to_drive);
    float celkovy_error = 0;

    while((abs(target_ticks) > abs(left_pos) || abs(target_ticks) > abs(right_pos)) && 
          (millis() - start_time < timeoutMs)) {
        
        man.motor(m_id_left).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(m_id_right).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });

        int front_distance_senzor = first_sensor();
        int back_distance_senzor = second_sensor();

        if(front_distance_senzor <= 0 || front_distance_senzor > 300 || back_distance_senzor <= 0 || back_distance_senzor > 300){
            std::cout << "Chyba senzoru vzdálenosti!" << std::endl;
            front_distance_senzor = distance_of_wall;
            back_distance_senzor = distance_of_wall;
        }

        // P regulátor

        // if((front_distance_senzor >= distance_of_wall + 18) && (back_distance_senzor >= distance_of_wall + 18)){
        //     std::cout << "Daleko" << std::endl;
        //     celkovy_error = front_distance_senzor - distance_of_wall;
        // }
        // else if((front_distance_senzor + 18 < distance_of_wall) && (back_distance_senzor + 18 < distance_of_wall)){
        //     std::cout << "PŘÍLIŠ BLÍZKO ZDI" << std::endl;
        //     celkovy_error = front_distance_senzor - distance_of_wall;
        // }
        // else{
        //     celkovy_error = 0;
        // }

        celkovy_error = (front_distance_senzor + back_distance_senzor)/2 - distance_of_wall;
        float error = 0;
        if(abs(celkovy_error) > 40){
            error = celkovy_error * 0.5f;
            std::cout << "Velká chyba: " << error << std::endl;
        }
        else{
            error =  (front_distance_senzor - back_distance_senzor) * 1.8 + celkovy_error * 0.5f;
            std::cout << "Malá chyba: " << error << std::endl;
        }

        float correction = error * m_kp;
        correction = abs(std::max(-m_max_correction, std::min(correction, m_max_correction)));
        speed_left = base_speed_left;
        speed_right = base_speed_right;

        // Aplikace korekce podle polarity
        if(is_wall_on_right){
            if (error > 0) {
                if (m_polarity_switch_left) {
                    speed_left -= correction;
                    speed_right -= correction;
                } else {
                    speed_left += correction;
                    speed_right += correction;
                }
            } else if (error < 0) {
                if (m_polarity_switch_right) {
                    speed_right -= correction;
                    speed_left -= correction;
                } else {
                    speed_right += correction;
                    speed_left += correction;
                }
            }
        }
        else{
            if (error > 0) {
                if (m_polarity_switch_left) {
                    speed_left += correction;
                    speed_right += correction;//ok
                } else {
                    speed_left -= correction;
                    speed_right -= correction;//ok
                }
            } else if (error < 0) {
                if (m_polarity_switch_right) {
                    speed_right += correction;
                    speed_left += correction;//ok
                } else {
                    speed_right -= correction;
                    speed_left -= correction;//ok
                }
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
        
        // Výpis informací pro ladění (volitelné)
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "celkovy_error: " << celkovy_error << std::endl;
        std::cout << "Error: " << error << std::endl;
        std::cout << "Correction: " << correction << std::endl;
        std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
        std::cout << "Front sensor: " << front_distance_senzor << ", Back sensor: " << back_distance_senzor << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        // printf_wifi("celkovy error:  %.1f", celkovy_error);
        // printf_wifi("Error:  %.1f", error);
        // printf_wifi("Correction:  %.1f", correction);
        // printf_wifi("Speed left:  %.1f", speed_left);
        // printf_wifi("Speed right:  %.1f", speed_right);
        // printf_wifi("Front sensor:  %d", front_distance_senzor);
        // printf_wifi("Back sensor:  %d", back_distance_senzor);
        // printf_wifi("-----------------------------");
        // handleWebClients();
        // Krátké zpoždění pro stabilizaci
        delay(50);
        }
    
    // Zastavení motorů po ukončení
    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);
    
}

void Motors::orient_to_wall(bool button_or_right, std::function<int()> first_sensor, //first senzor se musi vzdy davat do prava, nebo do predu!!!
                                   std::function<int()> second_sensor, float speed) { 
    int distance_first = 0;
    int distance_second = 0;

    auto& man = rb::Manager::get();

    float speed_left = speed;
    float speed_right = speed;

    for(byte i = 0; i < 3; i++){
        distance_first = first_sensor();
        distance_second = second_sensor();
        delay(60);
    }

    distance_first = first_sensor();
    delay(80);
    distance_second = second_sensor();
    std::cout<< "Distance first: " << distance_first << " , Distance second: " << distance_second <<std::endl;
    if(distance_first > 800 && distance_second > 800){
        std::cout<< "Jsme moc daleko, nebo nevim jak se mam srovnat"<<std::endl;
        return;
    }

    int start_error = distance_first - distance_second;

    if(button_or_right){
        if(distance_first > distance_second){ //pray kolo couve a levy jede dopredu
            if(m_polarity_switch_right){
                speed_right = speed;
                speed_left = speed;
            }
            else{
                speed_right = -speed;
                speed_left = -speed;
            }
        }
        else{ //levy kolo couve a pravy jede dopredu
            if(m_polarity_switch_right){
                speed_left = -speed;
                speed_right = -speed;
            }
            else{
                speed_left = speed;
                speed_right = speed;
            }
        }
    }
    else{
        if(distance_first > distance_second){ //pray kolo jede dopredu a levy couve
            if(m_polarity_switch_right){
                speed_right = -speed;
                speed_left = -speed;
            }
            else{
                speed_right = speed;
                speed_left = speed;
            }
        }
        else{ //levy kolo jede dopredu a pravy couve
            if(m_polarity_switch_right){
                speed_left = speed;
                speed_right = speed;
            }
            else{
                speed_left = -speed;
                speed_right = -speed;
            }
        }
    }
    unsigned long start_time = millis();
    int timeut_ms = 5000;

    while(timeut_ms > millis() - start_time){
        distance_first = first_sensor();
        distance_second = second_sensor();

        int error = distance_first - distance_second;

        if(abs(error) <= 5){ //nastavit
            break;
        }

        if((start_error > 0) && (error > 0)){ //porad stejne
            man.motor(m_id_left).power(pctToSpeed(speed_left));
            man.motor(m_id_right).power(pctToSpeed(speed_right));
        }
        else if((start_error < 0) && (error < 0)){ //porad stejne
            man.motor(m_id_left).power(pctToSpeed(speed_left));
            man.motor(m_id_right).power(pctToSpeed(speed_right));
        }
        else{
            man.motor(m_id_left).power(pctToSpeed(-speed_left)); // prejel to a musi jet zpet
            man.motor(m_id_right).power(pctToSpeed(-speed_right));
        }

        delay(30);
    }

    man.motor(m_id_left).power(0);
    man.motor(m_id_right).power(0);

}

// int predni[3];
    // int zadni[3];
    
    // for(byte i = 0; i < 5; i++){
    //     if(i %2 == 0){
    //     predni[i] = first_sensor();
    //     }
    //     else{
    //     zadni[i] = second_sensor();
    //     }
    //     delay(60);
    // }
    // byte a = 0;
    // byte b = 0;
    // for(byte i = 0; i < 6; i++){
    //     if(i %2 == 0){
    //     predni[a] = first_sensor();
    //     a++;
    //     }
    //     else{
    //     zadni[b] = second_sensor();
    //     b++;
    //     }
    //     delay(60);
    // }

    // std::cout<< "Predni senzory: " << predni[0] << " " << predni[1] << " " << predni[2] << std::endl;
    // std::cout<< "Zadni senzory: " << zadni[0] << " " << zadni[1] << " " << zadni[2] << std::endl;

    // if(predni[3]> 800 && predni[4] > 800 && predni[5] > 800 && zadni[3]> 800 && zadni[4] > 800 && zadni[5] > 800){
    //     std::cout<< "Jsme moc daleko, nebo nevim jak se mam srovnat"<<std::endl;
    //     return;
    // }
    // else if(predni[3]< 800 || predni[4] < 800 || predni[5] < 800 || zadni[3]< 800 || zadni[4] < 800 || zadni[5] < 800){
    //     distance_first = min(std::min(predni[3], predni[4]), predni[5]);
    //     distance_second = min(std::min(zadni[3], zadni[4]), zadni[5]);
    //     if(distance_first < distance_second){
    //         std::cout<< "Predni senzor je blize zdi nez zadni"<<std::endl;
    //     }
    //     else{
    //         std::cout<< "Zadni senzor je blize zdi nez predni, jedeme dopredu"<<std::endl;
    //     }
    // }


/////////////////////////////////////////////////////////////////////////////////////////////////

int32_t Motors::scale(int32_t val) {
    return val * 100 / RBPROTOCOL_AXIS_MAX;
}

int16_t Motors::pctToPower(int8_t pct) {
    return rb::clamp(pct * INT16_MAX / 100, -INT16_MAX, INT16_MAX);
}


int16_t Motors::pctToSpeed(float pct) {
    // Omezení vstupu
    pct = rb::clamp(pct, -100.0f, 100.0f);
    
    // Přímý výpočet s přetypováním až na konci
    int32_t speed = static_cast<int32_t>((pct * m_max_speed) / 100.0f);
    
    // Omezení rozsahu
    return rb::clamp(speed, -INT16_MAX, INT16_MAX);
}

int32_t Motors::mmToTicks(float mm) const {
    return (mm / m_wheel_circumference) * prevod_motoru;
}
// int32_t Motors::mmToTicks_left(float mm) const {
//     return (mm / m_wheel_circumference) * prevod_motoru * rozdil_v_kolech_levy;
// }
// int32_t Motors::mmToTicks_right(float mm) const {
//     return (mm / m_wheel_circumference) * prevod_motoru * rozdil_v_kolech_pravy;
// }
float Motors::ticksToMm(int32_t ticks) const {
    return float(ticks) / prevod_motoru * m_wheel_circumference;
}
int32_t Motors::mmToTicks_left(float mm) const {
    return (mm / m_wheel_circumference_left) * prevod_motoru;
}
int32_t Motors::mmToTicks_right(float mm) const {
    return (mm / m_wheel_circumference_right) * prevod_motoru;
}


// WiFi inicializace
// WiFi inicializace
void Motors::initWifi(const char* ssid, const char* password) {
    std::cout << "Inicializace WiFi..." << std::endl;
    
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        attempts++;
        std::cout << ".";
    }
    std::cout << std::endl;
    
    if (WiFi.status() == WL_CONNECTED) {
        m_wifi_initialized = true;
        m_server = new WebServer(8080);
        
        std::cout << "WiFi pripojeno!" << std::endl;
        std::cout << "IP: " << WiFi.localIP().toString().c_str() << std::endl;
        std::cout << "Web server spusten na: http://" << WiFi.localIP().toString().c_str() << ":8080" << std::endl;
        
        // Nastavení webových stránek - OPRAVENÁ VERZE
        m_server->on("/", [this]() {
            std::cout << "HTTP GET / received" << std::endl;
            String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Log</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background: white;
            color: black;
        }
        #log {
            background: white;
            border: 1px solid #ccc;
            padding: 10px;
            height: 80vh;
            overflow-y: auto;
            font-family: monospace;
            white-space: pre-wrap;
            line-height: 1.3;
        }
        .controls {
            margin: 10px 0;
        }
        button {
            padding: 8px 15px;
            margin-right: 10px;
            cursor: pointer;
            background: #f0f0f0;
            border: 1px solid #ccc;
            border-radius: 3px;
        }
        button:hover {
            background: #e0e0e0;
        }
    </style>
</head>
<body>
    <h1>Robot Log</h1>
    <div><strong>Adresa:</strong> http://)rawliteral" + WiFi.localIP().toString() + R"rawliteral(:8080</div>
    
    <div class="controls">
        <button onclick="clearLog()">Smazat log</button>
        <button onclick="toggleAutoRefresh()" id="refreshBtn">Vypnout auto-obnovování</button>
    </div>
    
    <div id="log">Načítám...</div>

    <script>
        let autoRefresh = true;
        let refreshInterval = setInterval(updateLog, 500);
        let shouldScroll = true;
        
        // Sleduj uživatelský scroll
        const logElement = document.getElementById('log');
        logElement.addEventListener('scroll', function() {
            // Pokud uživatel scrolluje nahoru, zastav auto-scroll
            shouldScroll = (logElement.scrollTop + logElement.clientHeight >= logElement.scrollHeight - 10);
        });
        
        function updateLog() {
            if (!autoRefresh) return;
            
            fetch('/log')
                .then(response => response.text())
                .then(data => {
                    const wasScrolledToBottom = shouldScroll;
                    logElement.textContent = data;
                    
                    // Scrolluj dolů pouze pokud byl uživatel na konci
                    if (wasScrolledToBottom) {
                        logElement.scrollTop = logElement.scrollHeight;
                    }
                })
                .catch(err => console.error('Chyba:', err));
        }
        
        function clearLog() {
            fetch('/clear')
                .then(() => updateLog())
                .catch(err => console.error('Chyba:', err));
        }
        
        function toggleAutoRefresh() {
            autoRefresh = !autoRefresh;
            const button = document.getElementById('refreshBtn');
            
            if (autoRefresh) {
                button.textContent = 'Vypnout auto-obnovování';
                refreshInterval = setInterval(updateLog, 500);
            } else {
                button.textContent = 'Zapnout auto-obnovování';
                clearInterval(refreshInterval);
            }
        }
        
        // První načtení
        updateLog();
    </script>
</body>
</html>
)rawliteral";
            m_server->send(200, "text/html", html);
        });
        
        m_server->on("/log", [this]() {
            m_server->send(200, "text/plain", m_wifi_log_buffer);
        });
        
        m_server->on("/clear", [this]() {
            m_wifi_log_buffer = "";
            m_server->send(200, "text/plain", "Log smazán");
        });

        m_server->on("/test", [this]() {
            m_server->send(200, "text/plain", "Robot WiFi test OK!");
        });

        m_server->onNotFound([this]() {
            m_server->send(404, "text/plain", "Cesta nenalezena");
        });
        
        m_server->begin();
        printf_wifi("Web server started on http://%s:8080", WiFi.localIP().toString().c_str());
    } else {
        std::cout << "WiFi se nepodařilo připojit!" << std::endl;
    }
}

// Základní print funkce
void Motors::print_wifi(const String& message) {
    if (!m_wifi_initialized) return;
    
    // Pokud zpráva obsahuje \n, rozděl ji na více řádků
    int startIndex = 0;
    int newLineIndex;
    
    do {
        newLineIndex = message.indexOf('\n', startIndex);
        String line;
        
        if (newLineIndex == -1) {
            line = message.substring(startIndex);
        } else {
            line = message.substring(startIndex, newLineIndex);
        }
        
        // Přidat časové razítko pouze pokud řádek není prázdný
        if (line.length() > 0) {
            String timestamp = "[" + String(millis() / 1000.0, 1) + "s] ";
            m_wifi_log_buffer += timestamp + line + "\n";
        }
        
        startIndex = newLineIndex + 1;
    } while (newLineIndex != -1);
    
    // ZVĚTŠENÝ buffer - 20 000 znaků
    if (m_wifi_log_buffer.length() > 40000) {
        int newStart = m_wifi_log_buffer.indexOf('\n', m_wifi_log_buffer.length() - 15000);
        if (newStart != -1) {
            m_wifi_log_buffer = m_wifi_log_buffer.substring(newStart + 1);
        }
    }
}


// Přetížení pro const char*
void Motors::print_wifi(const char* message) {
    print_wifi(String(message));
}

// Formátovaný výpis (printf style)
void Motors::printf_wifi(const char* format, ...) {
    if (!m_wifi_initialized) return;
    
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    print_wifi(String(buffer));
}

// Zpracování webových požadavků
void Motors::handleWebClient() {
    if (m_wifi_initialized && m_server) {
        m_server->handleClient();
    }
}

}; // namespacer rk
