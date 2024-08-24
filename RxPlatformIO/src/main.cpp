/* Receiver Example (Simple) */

#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_RECV_PIN D6

void printBinary(uint8_t value)
{
    for ( uint8_t mask = 1 << 7; mask; mask >>= 1 )
    {
        Serial.print(value & mask ? 1 : 0);
    }
}

void setup() {
    Serial.begin(115200);
    Manch.beginReceive(BR_50000, MANCH_RECV_PIN, MFLAG_NONE);
}

void loop() {

    uint8_t data = 0;

    if (Manch.getData(&data)) {
        Serial.print((char)data);

    }
    // }

}