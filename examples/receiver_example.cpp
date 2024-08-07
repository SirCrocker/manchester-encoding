/* Receiver Example (Simple) */

#include <Arduino.h>
#include "../ManchesterEnc.h"

#define MANCH_RECV_PIN D5

void setup() {
    Serial.begin(115200);
    Manch.beginReceive(BR_300, MANCH_RECV_PIN);
}

void loop() {
    uint8_t data = 0;

    if (Manch.getData(data)) {
        Serial.println((char)data);
    }

}