/* Receiver Example (With Channel Encoding) */

#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_RECV_PIN D6

void setup() {
    Serial.begin(115200);
    Manch.beginTransmit(BR_19200, MANCH_RECV_PIN, MFLAG_DEF_CHANNEL_ENC);
}

void loop() {

    uint8_t data = 0;

    if (Manch.getData(&data)) {
        Serial.print((char)data);
    }

}