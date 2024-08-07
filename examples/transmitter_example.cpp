/* Transmitter Example (Simple) */

#include <Arduino.h>
#include "../ManchesterEnc.h"

#define MANCH_TX_PIN D5

void setup() {
    Serial.begin(115200);
    Manch.beginTransmit(BR_300, MANCH_TX_PIN);
}

void loop() {
    uint8_t data = (uint8_t)"a";

    Manch.transmit(data);
}