// main.cpp — Stage 1: Blink test
// Confirms: ESP32 boots, USB upload works, Serial Monitor works
//
// Project: Treadmill BLE Bridge
// Board: AZDelivery ESP32 Dev Kit C V2 (ESP-WROOM-32)

#include <Arduino.h>

// Built-in LED on AZDelivery ESP32 Dev Kit C V2
// GPIO 2 is the standard onboard LED for most ESP32 dev boards
#define LED_PIN 2

void setup() {
    // Start serial communication for debugging
    // 115200 is the standard baud rate for ESP32
    Serial.begin(115200);

    // Small delay to let Serial initialize after boot
    delay(1000);

    Serial.println("=== Treadmill BLE Bridge ===");
    Serial.println("Stage 1: Blink test");
    Serial.println("If you see this + LED blinking — hardware is OK!");
    Serial.println();

    // Configure the LED pin as output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);   // LED on
    Serial.println("LED: ON");
    delay(1000);                   // Wait 1 second

    digitalWrite(LED_PIN, LOW);    // LED off
    Serial.println("LED: OFF");
    delay(1000);                   // Wait 1 second
}
