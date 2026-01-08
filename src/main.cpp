#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// --- Nastavení pinů (dle vašeho zadání) ---
#define XSHUT1 14
#define XSHUT2 27 // POZOR: Pin 13 je často LED na desce, může blikat při bootu.

// --- Adresy ---
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// --- Objekty senzorů ---
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Proměnná pro data
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// --- VAŠE FUNKCE PRO SKENOVÁNÍ I2C ---
void scan_i2c() {
  Serial.println("--- SKENOVÁNÍ I2C SBĚRNICE ---");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Nalezeno zařízení na adrese: 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("Žádné I2C zařízení nenalezeno.");
  Serial.println("------------------------------");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);

  // 1. Inicializace pinů XSHUT
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  // 2. Vypnutí obou senzorů (Reset)
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(50);

  // 3. Start I2C
  Wire.begin(21, 22);
  
  Serial.println("\n===== START INITIALIZACE =====");
  
  // Kontrolní scan - mělo by být prázdno (nebo jiná zařízení, ale ne 0x29)
  Serial.println("Krok 1: Všechny VL53L0X vypnuty.");
  scan_i2c();

  // --- Zapnutí a nastavení Senzoru 1 ---
  Serial.println("Krok 2: Zapínám Senzor 1...");
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  
  // Teď by měl být vidět na 0x29
  scan_i2c(); 

  // Inicializace + změna adresy na 0x30
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("CHYBA: Senzor 1 se nepodařilo nastartovat!"));
    while (1);
  }
  Serial.println("Senzor 1 nastaven na 0x30.");
  
  // Teď by měl být vidět na 0x30
  scan_i2c();


  // --- Zapnutí a nastavení Senzoru 2 ---
  Serial.println("Krok 3: Zapínám Senzor 2...");
  digitalWrite(XSHUT2, HIGH);
  delay(10);

  // Teď by měl být vidět senzor 1 (0x30) a senzor 2 (0x29)
  scan_i2c();

  // Inicializace + změna adresy na 0x31
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("CHYBA: Senzor 2 se nepodařilo nastartovat!"));
    while (1);
  }
  Serial.println("Senzor 2 nastaven na 0x31.");

  // --- Finální kontrola ---
  Serial.println("Krok 4: HOTOVO. Finální stav sběrnice:");
  scan_i2c(); // Zde musíte vidět 0x30 a 0x31
}

void loop() {
  // Měření
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  // Výpis 1
  Serial.print("S1 [0x30]: ");
  if (measure1.RangeStatus != 4) {
    Serial.print(measure1.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print("---");
  }

  Serial.print("  |  ");

  // Výpis 2
  Serial.print("S2 [0x31]: ");
  if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print("---");
  }
  
  Serial.println();
  delay(200);
}

