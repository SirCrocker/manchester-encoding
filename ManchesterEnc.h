#pragma once
#ifndef _MANCHESTERENC_H_
#define _MANCHESTERENC_H_

/************************** 
-- INCLUDES -- 
**************************/
#include <Arduino.h>

/**************************
-- ENUMS -- 
**************************/

/* enc_convention_t
Convention used to represent the data:
 - IEEE 802.3:
    0 is HI-LO & 1 is LO-HI
- GE Thomas:
    0 is LO-HI & 1 is HI-LO
*/
typedef enum convention {
    IEEE802_3 = 0,
    GE_THOMAS = 1
} enc_convention_t;

/* baud_rate_t
Baud Rate to use, it is directly related to bandwidth, not data rate!
*/
typedef enum baudRates {
    BR_300    = 300,    // Tested with Sync
    BR_600    = 600,    // Tested without Sync
    BR_1200   = 1200,   // Tested without sync
    BR_2400   = 2400,   // Tested without sync
    BR_4800   = 4800,   // Tested without sync
    BR_9600   = 9600,   // Tested without sync
    BR_19200  = 19200,  // Tested without sync
    BR_38400  = 38400,  // Tested without sync
    BR_50000  = 50000,  // Tested with Sync
    BR_57600  = 57600,  // Untested
    BR_76800  = 76800,  // Untested
    BR_115200 = 115200  // Untested  
} baud_rate_t;


typedef enum rxState {
    RX_IDLE,
    RX_SYNC,
    RX_RECEIVING
} rx_state_t;

/************************** 
-- MACROs -- 
*************************/

// Convention to use
#ifndef MANCH_CONVENTION
#define MANCH_CONVENTION IEEE802_3
#endif // MANCH_CONVENTION

#define MANCH_RECV_BUFFER_SIZE 400

#define MANCH_SAMPLES_PER_MIDBIT 3  // Samples per mid-bit

#define MFLAG_NONE              0       // No flags.
#define MFLAG_DEF_CHANNEL_ENC   1 << 0  // Apply channel encoding (default coding).
#define MFLAG_CUS_CHANNEL_ENC   1 << 1  // Apply channel encoding (default coding).
#define MFLAG_TX_ISALWAYS_ONE   1 << 2  // Set transmitter to always on. // TODO: UNTESTED

#define MMASK_CHANNEL_ENC MFLAG_DEF_CHANNEL_ENC | MFLAG_CUS_CHANNEL_ENC

/************************** 
-- CLASSES & FUNCTIONS -- 
*************************/

class ManchesterEncoding {
public:
    static ManchesterEncoding &getInstance();

    ManchesterEncoding(const ManchesterEncoding &) = delete;
    ManchesterEncoding &operator=(const ManchesterEncoding &) = delete;

    /**
     * @brief Setups the transmission
     * 
     * @param baud_rate baud rate with which the data will be sent
     * @param pin pin that will send the data
     * 
     * @attention baud rate must be the same as the one on the receiver
     * 
     * @return
     * 
     * @related beginReceive
     * 
     * @todo Implement parameter that is a flag (it will indicate if we want to use channel encoding or not)
     */
    void beginTransmit(baud_rate_t baud_rate, uint8_t pin);

    /**
     * @brief Setups the transmission
     * 
     * @param baud_rate baud rate with which the data will be sent
     * @param pin pin that will send the data
     * @param flags set options via flags (flags are prefixed with MFLAG)
     * 
     * @attention baud rate must be the same as the one on the receiver
     * 
     * @return
     * 
     * @related beginReceive
     * 
     * @todo Implement parameter that is a flag (it will indicate if we want to use channel encoding or not)
     */
    void beginTransmit(baud_rate_t baud_rate, uint8_t pin, uint16_t flags);
    
    /**
     * @brief Setups the reception
     * 
     * @param baud_rate baud rate with which the data was transmitted
     * @param pin pin that will read the received values
     * 
     * @attention baud rate must be the same as the one on the transmitter
     * 
     * @return
     * 
     * @related beginTransmit
     * 
     * @todo Implement parameter that is a flag (it will indicate if we want to use channel encoding or not)
     */
    void beginReceive(baud_rate_t baud_rate, uint8_t pin);

    /**
     * @brief Setups the reception
     * 
     * @param baud_rate baud rate with which the data was transmitted
     * @param pin pin that will read the received values
     * @param flags set options via flags (flags are prefixed with MFLAG)
     * 
     * @attention baud rate must be the same as the one on the transmitter
     * 
     * @return
     * 
     * @related beginTransmit
     * 
     * @todo Implement parameter that is a flag (it will indicate if we want to use channel encoding or not)
     */
    void beginReceive(baud_rate_t baud_rate, uint8_t pin, uint16_t flags);

    /**
     * @brief Transmit a byte using manchester encoding.
     * 
     * @param message byte to transmit.
     * 
     * @note The convention used depends on the macro MANCH_CONVENTION.
     * 
     * @return
     */
    void transmit(uint8_t message);

    /**
     * @brief Gets the data (if available)
     * 
     * @param data retrieved data will be saved here.
     * 
     * @return bool that determines if data was available.
     */
    bool getData(uint8_t *data);

    /**
     * @brief Set the function that will apply channel encoding to the data.
     * 
     * @param encoding_function pointer to the encoding function. 
     * 
     * @attention The function must return a pointer to a buffer where the bytes are stored and have two parameters:
     * the byte to encode and the size of the buffer.
     * 
     * @see transmitter custom chenc example.
     */
    void setEncodingFunction(uint8_t* (*encoding_function) (uint8_t, size_t*));
    
    /**
     * @brief Set the function that will apply channel decoding to the data.
     * 
     * @param decoding_function pointer to the decoding function.
     * 
     * @attention The function must return a bool indicating if decoding was successful and have two parameters:
     * a byte received and a pointer where the decoded message will be saved.
     * 
     * @see receiver custom chenc example.
     */
    void setDecodingFunction(bool (*decoding_function) (uint8_t, uint8_t*));

private:
    ManchesterEncoding() = default;

    /**
     * @brief Transmit a one using manchester encoding.
     *
     * @note The convention used depends on the macro MANCH_CONVENTION.
     *  
     * @return
     */
    void transmitOne();

    /**
     * @brief Transmit a zero using manchester encoding.
     * 
     * @note The convention used depends on the macro MANCH_CONVENTION.
     * 
     * @return
     */
    void transmitZero();

    /**
     * @brief Decodes the raw bits, removing headers and trailers, leaving only the data.
     * 
     * @details Raw bits are the ones obtained after manchester decoding.
     * 
     * @return
     */
    void decodeRawBits();

    /**
     * @brief Implements channel encoding over the data (1 byte).
     * 
     * @param data the byte to encode.
     * @param size the size of the buffer that has the encoded bytes.
     * 
     * @details The channel encoding corresponds to repetition code of block-length three, which means
     * that each byte is sent three times. This has a code rate of 1/3.
     * 
     * @attention It currently only supports single byte encoding/decoding.
     * 
     * @return a pointer to the first element of an array containing the bytes to sent.
     */
    uint8_t* encodeData(uint8_t data, size_t *size);

    /**
     * @brief Implements channel decoding over the data (3 bytes). The function is called with each
     * byte received, and when then sufficient number of bytes are received, it decodes them.
     * 
     * @param data the bytes in order of reception.
     * @param decoded_message a pointer where the decoded message will be saved.
     * 
     * @details The channel encoding corresponds to repetition code of block-length three, which means
     * that each byte is sent three times. This has a code rate of 1/3.
     * 
     * @attention It currently only supports single byte encoding/decoding.
     * 
     * @return true if a byte was decoded, false otherwise.
     */
    bool decodeData(uint8_t data, uint8_t *decoded_message);

    uint8_t m_txpin;
    uint8_t m_rxpin;
    uint32_t m_ticks_sample; // Ticks per sample
    unsigned int m_txdelay;
    // TODO: Makes this allocation dependent on if transmitter or receiver mode will be used (in constructor, this would then be a pointer)
    uint8_t m_byte_buffer[MANCH_RECV_BUFFER_SIZE / 2];
    uint8_t m_buffer_read_pos;
    uint8_t m_buffer_save_pos;

    uint8_t* (*m_pencode_fun) (uint8_t, size_t*); // Custom encode function
    bool (*m_pdecode_fun) (uint8_t, uint8_t*);    // Custom decode function
    uint16_t m_flags = MFLAG_NONE;

};

extern ManchesterEncoding &Manch;

#endif // _MANCHESTERENC_H_