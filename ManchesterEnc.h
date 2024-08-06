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
     1 is HI-LOW & 0 is LOW-HI
- GE Thomas:
    1 is LOW-HI & 0 is HI-LOW 
*/
typedef enum convention {
    IEEE802_3 = 0,
    GE_THOMAS = 1
} enc_convention_t;

/* baud_rate_t
Baud Rate to use, it is directly related to bandwidth, not data rate!
*/
typedef enum baudRates {
    BR_300    = 300,  // Untested
    BR_600    = 600,  // Untested
    BR_1200   = 1200,  // Untested
    BR_2400   = 2400,  // Untested
    BR_4800   = 4800,  // Untested
    BR_9600   = 9600,  // Untested
    BR_19200  = 19200,  // Untested
    BR_38400  = 38400,  // Untested
    BR_57600  = 57600,  // Untested
    BR_76800  = 76800,  // Untested
    BR_115200 = 115200  // Untested  
} baud_rate_t;


typedef enum rxState {
    RX_IDLE,
    RX_RECEIVING
} rx_state_t;

/************************** 
-- MACROs -- 
*************************/

// Convention to use
#ifndef MANCH_CONVENTION
#define MANCH_CONVENTION IEEE802_3
#endif // MANCH_CONVENTION

#define MANCH_RECV_BUFFER_SIZE 64

#define MANCH_SAMPLES_PER_MIDBIT 3  // Samples per mid-bit
#define MANCH_IDLE_CHECK_VALUE 0x80
#define MANCH_IDLE_CHECK_LOWER_LIMIT MANCH_IDLE_CHECK_VALUE - MANCH_SAMPLES_PER_MIDBIT * 4
#define MANCH_IDLE_CHECK_UPPER_LIMIT MANCH_IDLE_CHECK_VALUE + MANCH_SAMPLES_PER_MIDBIT * 4

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
     */
    void beginTransmit(baud_rate_t baud_rate, uint8_t pin);
    
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
     */
    void beginReceive(baud_rate_t baud_rate, uint8_t pin);

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
     * @param data data "recuperada" will be saved here.
     * 
     * @return bool that determines if data was available.
     */
    bool getData(uint8_t &data);

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
     * @return
     */
    void decodeRawBits();

    uint8_t m_txpin;
    uint8_t m_rxpin;
    uint32_t m_ticks_sample; // Ticks per sample
    unsigned int m_txdelay;
    // TODO: Makes this allocation dependent on if transmitter or receiver mode will be used (in constructor, this would then be a pointer)
    uint8_t m_byte_buffer[MANCH_RECV_BUFFER_SIZE / 2];
    uint8_t m_buffer_read_pos;
    uint8_t m_buffer_save_pos;

};

/**
 * @brief Reads the value from the receive pin and determines if a valid bit
 * was received, then stores it for future processing.
 * 
 * @note This function will be used in an interrupt.
 * 
 * @link https://arduino-esp8266.readthedocs.io/en/latest/reference.html#interrupts
 * 
 * @return
 */
static void interruptFunction();

/**
 * @brief Saves a midbit value
 * 
 * @param midbit_val midbit value to save (1 or 0)
 * 
 * @note This function will be used in an interrupt.
 * 
 * @link https://arduino-esp8266.readthedocs.io/en/latest/reference.html#interrupts
 * 
 * @return
 */
static void saveReceivedMidbit(uint8_t midbit_val);

extern ManchesterEncoding &Manch;

#endif // _MANCHESTERENC_H_