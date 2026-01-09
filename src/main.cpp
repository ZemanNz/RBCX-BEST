#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "robotka.h"

// Definice pinů XSHUT (upravte si podle reality, pokud je máte jinak)
#define PIN_XSHUT_1 27  // Přední
#define PIN_XSHUT_2 14  // Spodní
#define PIN_XSHUT_3 26  // Senzor 3

// Adresy
#define ADRESA_1 0x30
#define ADRESA_2 0x31
#define ADRESA_3 0x32

// Objekty senzorů
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();

// Funkce pro nastavení jednoho senzoru
bool nastavitSenzor(Adafruit_VL53L0X &sensor, int pin, uint8_t novaAdresa) {
  Serial.printf("--> Startuji senzor na pinu %d... ", pin);
  
  // 1. Zapnout senzor
  digitalWrite(pin, HIGH);
  delay(50); // Počkáme, až se probere

  // 2. Inicializace (vždy na defaultní adrese 0x29)
  if (!sensor.begin(0x29, false, &Wire)) {
    Serial.println("CHYBA: Senzor neodpověděl na 0x29!");
    return false;
  }

  // 3. Změna adresy na finální
  if (!sensor.setAddress(novaAdresa)) {
    Serial.println("CHYBA: Nepodařilo se změnit adresu!");
    return false;
  }

  Serial.printf("OK. Přesunuto na 0x%X\n", novaAdresa);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);

  Serial.println("\n=== START DIAGNOSTIKY VL53L0X ===");

  // 1. HARD RESET VŠECH SENZORŮ
  // Všechny vypneme, aby nerušily I2C sběrnici
  pinMode(PIN_XSHUT_1, OUTPUT); digitalWrite(PIN_XSHUT_1, LOW);
  pinMode(PIN_XSHUT_2, OUTPUT); digitalWrite(PIN_XSHUT_2, LOW);
  pinMode(PIN_XSHUT_3, OUTPUT); digitalWrite(PIN_XSHUT_3, LOW);
  
  Serial.println("Všechny senzory vypnuty (XSHUT = LOW). Čekám...");
  delay(100);

  // 2. Inicializace Robotky a I2C
  rkConfig cfg;
  rkSetup(cfg);
  
  // I2C na 100kHz pro jistotu (stabilita > rychlost)
  Wire.begin(21, 22, 10000);

  // 3. Postupné zapínání
  // SENZOR 1
  if (!nastavitSenzor(sensor1, PIN_XSHUT_1, ADRESA_1)) {
    Serial.println("!!! Konec testu - chyba senzoru 1 !!!");
    while(1);
  }

  // SENZOR 2
  if (!nastavitSenzor(sensor2, PIN_XSHUT_2, ADRESA_2)) {
    Serial.println("!!! Konec testu - chyba senzoru 2 !!!");
    while(1);
  }

  // SENZOR 3
  if (!nastavitSenzor(sensor3, PIN_XSHUT_3, ADRESA_3)) {
    Serial.println("!!! Konec testu - chyba senzoru 3 !!!");
    while(1);
  }

  Serial.println("=== HOTOVO: Všechny 3 senzory běží ===");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  
  // Měření 1
  sensor1.rangingTest(&measure, false);
  int d1 = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  // Měření 2
  sensor2.rangingTest(&measure, false);
  int d2 = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  // Měření 3
  sensor3.rangingTest(&measure, false);
  int d3 = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  Serial.printf("S1: %4d mm | S2: %4d mm | S3: %4d mm\n", d1, d2, d3);
  delay(200);
}