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
typedef enum convention{
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


/************************** 
-- MACROs -- 
*************************/

// Convention to use
#ifndef MANCH_CONVENTION
#define MANCH_CONVENTION IEEE802_3
#endif // MANCH_CONVENTION

#define MANCH_SAMPLES_PER_SYMBOL 3  // Samples per mid-bit

/************************** 
-- CLASSES & FUNCTIONS -- 
*************************/

class ManchesterEncoding {
public:
    static ManchesterEncoding &getInstance();

    ManchesterEncoding(const ManchesterEncoding &) = delete;
    ManchesterEncoding &operator=(const ManchesterEncoding &) = delete;

    /**
     * @brief beginTxs
     * 
     * @attention (WIP) - Work In Progress
     * 
     * @return
     */
    void beginTransmit(baud_rate_t baud_rate, uint8_t pin);
    
    /**
     * @brief beginRxs
     * 
     * @attention (WIP) - Work In Progress
     * 
     * @return
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

    uint8_t m_txpin;
    uint8_t m_rxpin;
    uint32_t m_ticks_sample; // Ticks per sample
    unsigned int m_txdelay;

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

extern ManchesterEncoding &Manch;

#endif // _MANCHESTERENC_H_