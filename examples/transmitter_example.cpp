/* Transmitter Example (With Channel Encoding) */

#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_TX_PIN D6

void setup() {
    Serial.begin(115200);
    Manch.beginTransmit(BR_19200, MANCH_TX_PIN, MFLAG_CHANNEL_ENC);
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