/* Transmitter Example (Simple) */

#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_TX_PIN D6

void setup() {
    Serial.begin(115200);
    Manch.beginTransmit(BR_50000, MANCH_TX_PIN, MFLAG_NONE);
}

void loop() {
    
    if (Serial.available()) {
        
        String toSend = Serial.readString();
  
        for (char *it = toSend.begin(); it != toSend.end(); it++)
        {
            Manch.transmit(*it);
            Serial.print(*it);
            yield();
        }
        
    }
}