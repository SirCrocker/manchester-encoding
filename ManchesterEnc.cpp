#include "ManchesterEnc.h"

static uint8_t g_rx_pin = 0x64;  // Placeholder value, ESP8266 does not have 100 pins
static uint8_t G_SAMPLE_MASK = 0xff >> (8 - MANCH_SAMPLES_PER_SYMBOL);  // For checking the received value

volatile static uint8_t g_samples_vals = 0xAA;
volatile static uint8_t g_sample_counter = 0x00;

// Singleton behaviour
ManchesterEncoding &ManchesterEncoding::getInstance() {
    static ManchesterEncoding instance;
    return instance;
}

ManchesterEncoding &Manch = Manch.getInstance();

// ------------------------

void ManchesterEncoding::beginTransmit(baud_rate_t baud_rate, uint8_t pin) {
    m_txpin = pin;
    m_txdelay = 1e6 / (double)baud_rate; // Delay in microseconds

}

void ManchesterEncoding::beginReceive(baud_rate_t baud_rate, uint8_t pin) {
    m_rxpin = pin;
    g_rx_pin = pin;
    
    if ( (g_rx_pin + 1) > NUM_DIGITAL_PINS) {
        #pragma diag_suppress 35  // Ignore below error in editor
        #error "Invalid pin defined for receiving."
    }

    uint32_t midbit_ticks = uint32_t((double)CPU_CLK_FREQ / (double)baud_rate);  // Number of ticks per symbol (related to symbol rate)
    m_ticks_sample = midbit_ticks / MANCH_SAMPLES_PER_SYMBOL; // Ticks between samples per midbit

    // Attach interrupts - I chose timer1 because timer0 is apparently used by WiFi
    noInterrupts();
    timer1_detachInterrupt();
    timer1_disable();
    timer1_attachInterrupt(interruptFunction);
    timer1_isr_init();
    timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
    timer1_write(m_ticks_sample);
    interrupts();
}

void ManchesterEncoding::transmitOne() {
#if MANCH_CONVENTION == 0 // IEEE 802.3
    digitalWrite(m_txpin, HIGH);
    delayMicroseconds(m_txdelay);

    digitalWrite(m_txpin, LOW);
    delayMicroseconds(m_txdelay);

#elif MANCH_CONVENTION == 1 // GE THOMAS
    digitalWrite(m_txpin, LOW);
    delayMicroseconds(m_txdelay);

    digitalWrite(m_txpin, HIGH);
    delayMicroseconds(m_txdelay);

#endif // CONVENTION
}

void ManchesterEncoding::transmitZero() {
#if MANCH_CONVENTION == 0 // IEEE 802.3
    digitalWrite(m_txpin, LOW);
    delayMicroseconds(m_txdelay);

    digitalWrite(m_txpin, HIGH);
    delayMicroseconds(m_txdelay);

#elif MANCH_CONVENTION == 1 // GE THOMAS
    digitalWrite(m_txpin, HIGH);
    delayMicroseconds(m_txdelay);

    digitalWrite(m_txpin, LOW);
    delayMicroseconds(m_txdelay);

#endif // CONVENTION
}

void ManchesterEncoding::transmit(uint8_t message) {
    
    uint8_t mask = 0x80; // Transmit from MSb to LSb
    for (uint8_t bit = 0; bit < 7; bit++) {
        if (mask & message) {
            transmitOne();
        } else {
            transmitZero();
        }
        mask >>= 1;
    }

}

void IRAM_ATTR interruptFunction() {
    uint8_t val_read = digitalRead(g_rx_pin);
    
    // Check for transition
    // if (val_read != (g_samples_vals & 0x01)) {
    //     // Value transitioned
    //     g_sample_counter = 0;
    // }
        
    // Store value
    g_samples_vals <<= 1;
    g_samples_vals |= val_read;

    g_sample_counter++;

    if (g_sample_counter == 3)
    {
        if (g_samples_vals & G_SAMPLE_MASK) // Check this when a transitions happens?
        {
            /* Save a 1 */
        } else {
            /* Save a 0 */
        }

        g_sample_counter = 0;
    }
       
    
}