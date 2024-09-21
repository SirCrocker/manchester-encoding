/* Receiver Example (With Custom Channel Encoding) */
// TODO: UNTESTED

#include <Arduino.h>
#include "kronecker-chenc.h"
#include "ManchesterEnc.h"

#define MANCH_RECV_PIN D6
#define TRAINING_SYMBOLS 0b10101010

/**
 * @brief Compatibility wrapper for kronecker channel decoding. The function is called with each
 * byte received, and when then sufficient number of bytes are received, it decodes them.
 * 
 * @param data the bytes in order of reception.
 * @param decoded_message a pointer where the decoded message will be saved.
 * 
 * @attention It currently only supports single byte encoding/decoding.
 * 
 * @return true if a byte was decoded, false otherwise.
 */
bool kroneckerDecodeWrapper(uint8_t data, uint8_t *decoded_message) {
    // Decoding with rank-one detector
    static uint8_t vals[2] = {0, 0}; // No dynamic allocation in embedded
    static uint8_t num_saved = 0;
    vals[num_saved] = data;

    if (num_saved == 1) { // Index starts at 0
        // decode
        *decoded_message = rank_one_detector_tmpd4s2(vals[0], vals[1], TRAINING_SYMBOLS);   // Bitwise Majority 3
        num_saved = 0;
        return true;
    }

    num_saved++;
    return false;
}

void setup() {
    Serial.begin(115200);
    Manch.beginTransmit(BR_19200, MANCH_RECV_PIN, MFLAG_CUS_CHANNEL_ENC);
    Manch.setDecodingFunction(kroneckerDecodeWrapper);
}

void loop() {

    uint8_t data = 0;

    if (Manch.getData(&data)) {
        Serial.print((char)data);
    }

}