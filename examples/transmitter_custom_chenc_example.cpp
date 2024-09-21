/* Transmitter Example (With Channel Encoding) */
// TODO: UNTESTED

#include <Arduino.h>
#include "kronecker-chenc.h"
#include "ManchesterEnc.h"

#define MANCH_TX_PIN D6

/**
 * @brief Compatibility wrapper for kronecker channel encoding.
 * 
 * @param data the byte to encode.
 * @param size the size of the buffer that has the encoded bytes.
 * 
 * @attention It currently only supports single byte encoding/decoding.
 * 
 * @return a pointer to the first element of an array containing the bytes to sent.
 */
uint8_t* kroneckerEncodeWrapper(uint8_t data, size_t* size) { 
    // Encoding with kronecker-chenc.
    *size = 2;
    return encode_kronecker_tmpd4s2(data);
}

void setup() {
    Serial.begin(115200);
    Manch.beginTransmit(BR_19200, MANCH_TX_PIN, MFLAG_CUS_CHANNEL_ENC);
    Manch.setEncodingFunction(kroneckerEncodeWrapper);
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