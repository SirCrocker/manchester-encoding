/* Transmitter Example (Simple) */

#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_RECV_PIN D6

void printBinary(uint16_t value)
{
    for ( uint16_t mask = 1 << 15; mask; mask >>= 1 )
    {
        Serial.print(value & mask ? 1 : 0);
    }
}

void setup() {
    Serial.begin(115200);
    Manch.beginReceive(BR_9600, MANCH_RECV_PIN);
}

void loop() {

    uint8_t data = 0;

    if (Manch.getData(&data)) {
        Serial.println("--------");
    }
        printBinary(Manch.getDeleteMe());
        Serial.println();
    // }

}