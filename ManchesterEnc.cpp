#include "ManchesterEnc.h"

/* ------------ GLOBAL VARIABLES ------------ */

static uint8_t g_rx_pin = 0x64;  // Placeholder value, ESP8266 does not have 100 pins
static uint8_t G_SAMPLE_MASK = 0xff >> (8 - MANCH_SAMPLES_PER_MIDBIT);  // For checking the received value

volatile static uint8_t g_samples_vals = 0x00;  // The read values will be saved here until a transition happens
volatile static uint8_t g_sample_counter = 0x00; // Counts the number of samples taken (we expect MANCH_SAMPLES_PER_MIDBIT)
volatile static rx_state_t g_state = RX_IDLE;  // Receiving State
volatile static uint8_t g_idle_check_num = MANCH_IDLE_CHECK_VALUE;  // Number of saved raw_bits (should fluctuate around this value)
volatile static uint8_t g_rawbits_buffer[MANCH_RECV_BUFFER_SIZE] = {0};
volatile static uint8_t g_buffer_save_pos = 0;
volatile static uint8_t g_buffer_read_pos = 0;

/* ------------ CLASS METHODS FOR SINGLETON BEHAVIOUR ------------ */

ManchesterEncoding &ManchesterEncoding::getInstance() {
    static ManchesterEncoding instance;
    return instance;
}

ManchesterEncoding &Manch = Manch.getInstance();

/* ------------ CLASS METHODS (CONT.) ------------ */

void ManchesterEncoding::beginTransmit(baud_rate_t baud_rate, uint8_t pin) {
    m_txpin = pin;

    pinMode(m_txpin, OUTPUT);
    digitalWrite(m_txpin, LOW);

    m_txdelay = 1e6 / (double)baud_rate; // Delay in microseconds

}

void ManchesterEncoding::beginReceive(baud_rate_t baud_rate, uint8_t pin) {
    m_buffer_read_pos = 0;
    m_buffer_save_pos = 0;
    
    m_rxpin = pin;
    g_rx_pin = pin;
    
    if ( (g_rx_pin + 1) > NUM_DIGITAL_PINS) {
        return;
    }

    pinMode(m_rxpin, INPUT);

    uint32_t midbit_ticks = uint32_t((double)CPU_CLK_FREQ / (double)baud_rate);  // Number of ticks per symbol (related to symbol rate)
    m_ticks_sample = midbit_ticks / MANCH_SAMPLES_PER_MIDBIT; // Ticks between samples per midbit

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
    
    /**
     * TEMP: transition detection for receiver, this will be later replaced by the sync header.
     */
    #if MANCH_CONVENTION == 0 // IEEE 802.3
        transmitOne();
    #elif MANCH_CONVENTION == 1 // GE THOMAS
        transmitZero();
    #endif // CONVENTION

    uint8_t mask = 0x80; // Transmit from MSb to LSb
    for (uint8_t bit = 0; bit < 8; bit++) {
        if (mask & message) {
            transmitOne();
        } else {
            transmitZero();
        }
        mask >>= 1;
    }

    digitalWrite(m_txpin, LOW);

}

void ManchesterEncoding::decodeRawBits() {
    /** IMPLEMENT:
     * 1. Go through the raw bits in the buffer, until read_pos == save_pos
     * 2. For each raw byte, identify the sync/header, extract the data and use the trailer (if channel encoding is present)
     * 3. Save the data to a delivery buffer
     * 
     * ATTENTION: We assume that save_pos will not overpass read_pos for one or more full cycles
     */

    while (g_buffer_read_pos != g_buffer_save_pos) {
        m_byte_buffer[m_buffer_save_pos] = g_rawbits_buffer[g_buffer_read_pos];

        m_buffer_save_pos = (m_buffer_save_pos + 1) % (MANCH_RECV_BUFFER_SIZE / 2);
        g_buffer_read_pos = (g_buffer_read_pos + 1) % MANCH_RECV_BUFFER_SIZE;
    }

}

bool ManchesterEncoding::getData(uint8_t &data) {
    decodeRawBits(); // I hope this does not take too long...

    if (m_buffer_read_pos != m_buffer_save_pos) {
        data = m_byte_buffer[m_buffer_read_pos];
        m_buffer_read_pos = (m_buffer_read_pos + 1) % (MANCH_RECV_BUFFER_SIZE / 2);
        return true;
    }

    return false;
}

/* ------------ STANDALONE FUNCTIONS ------------ */

void IRAM_ATTR interruptFunction() {
    uint8_t val_read = digitalRead(g_rx_pin);
    
    // Check for transition
    if (val_read != (g_samples_vals & 0x01)) {
        // Value transitioned
        g_sample_counter = 0;
        g_state = RX_RECEIVING;
    }

    // Store value
    g_samples_vals <<= 1;
    g_samples_vals |= val_read;

    // Depending on state do one thing or another
    switch (g_state)
    {
    case RX_RECEIVING:
        g_sample_counter++;
        
        if (g_sample_counter >= MANCH_SAMPLES_PER_MIDBIT)
        {
            if (g_samples_vals & G_SAMPLE_MASK) // We received MANCH_SAMPLES 1s
            {
                /* Save a 1 (mid-bit) */
                g_idle_check_num--;
                saveReceivedMidbit(1);

            } else { // We received MANCH_SAMPLES 0s
                /* Save a 0 (mid-bit)*/
                g_idle_check_num++;
                saveReceivedMidbit(0);

            }

            g_sample_counter = 0;
        }
        
        if ( (g_idle_check_num < MANCH_IDLE_CHECK_LOWER_LIMIT) || g_idle_check_num > MANCH_IDLE_CHECK_UPPER_LIMIT ) {
            g_state = RX_IDLE;
            g_idle_check_num = MANCH_IDLE_CHECK_VALUE;
        }
        break;
    
    case RX_IDLE:
    default:
        g_sample_counter = 0;
        g_idle_check_num = MANCH_IDLE_CHECK_VALUE;
        break;
    }   
    
}

void IRAM_ATTR saveReceivedMidbit(uint16_t midbit_val) {
    static uint8_t num_saved_midbit = 0;
    static uint16_t midbit_values_received = 0x00;

    // If we have received 16 midbits (or 8 raw bits) we read and save the raw bits
    if (num_saved_midbit == 16) {
        num_saved_midbit = 0;
        
        // Depending on the convention, we speed up the midbit decoding by just looking at 1 bit 
        // [!] WARNING: this does not catch invalid raw_bits.
        uint16_t mask = 0x01;
        uint8_t i = MANCH_CONVENTION == IEEE802_3 ? 1: 0;
        uint8_t raw_bits = 0x00;
        for (; i < 16; i += 2)
        {
            raw_bits |= midbit_values_received & mask;
            raw_bits <<= 1;
            mask <<= i;
        }
        
        // Save the raw bit to a buffer
        g_rawbits_buffer[g_buffer_save_pos] = raw_bits;
        g_buffer_save_pos = (g_buffer_save_pos + 1) % MANCH_RECV_BUFFER_SIZE;

    }

    midbit_values_received <<= 1;
    midbit_values_received |= midbit_val;
    num_saved_midbit++;
}