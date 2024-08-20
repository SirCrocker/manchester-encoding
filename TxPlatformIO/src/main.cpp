/* Transmitter Example (Simple) */

#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_TX_PIN D6

void setup() {
    Serial.begin(115200);
    Manch.beginTransmit(BR_9600, MANCH_TX_PIN, MFLAG_NONE);
}

void loop() {
    
    if (Serial.available()) {
        Serial.read();
        Serial.println("data sent.");
        uint8_t data = 0x65;

    
        Manch.transmit(data);
    }
}