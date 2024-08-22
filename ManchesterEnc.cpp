#include "ManchesterEnc.h"

/* ------------ STATIC FUNCTIONS ------------ */

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
static void saveReceivedMidbit(uint16_t midbit_val);

/* ------------ GLOBAL VARIABLES ------------ */

static uint8_t g_rx_pin = 0x64;  // Placeholder value, ESP8266 does not have 100 pins
static uint8_t G_SAMPLE_MASK = 0xff >> (8 - MANCH_SAMPLES_PER_MIDBIT);  // For checking the received value

volatile static uint8_t g_samples_vals = 0x00;  // The read values will be saved here until a transition happens
volatile static uint8_t g_sample_counter = 0x00; // Counts the number of samples taken (we expect MANCH_SAMPLES_PER_MIDBIT)
volatile static rx_state_t g_state = RX_IDLE;  // Receiving State

volatile static uint8_t g_idle_check_ones = 0;  // Number of saved raw_bits (should fluctuate around this value)
volatile static uint8_t g_idle_check_zeros = 0;  // Number of saved raw_bits (should fluctuate around this value)

volatile static uint8_t g_rawbits_buffer[MANCH_RECV_BUFFER_SIZE] = {0};
volatile static uint8_t g_buffer_save_pos = 0;
volatile static uint8_t g_buffer_read_pos = 0;
volatile static uint8_t g_num_saved_midbit = 0;

/* ------------ CLASS METHODS FOR SINGLETON BEHAVIOUR ------------ */

ManchesterEncoding &ManchesterEncoding::getInstance() {
    static ManchesterEncoding instance;
    return instance;
}

ManchesterEncoding &Manch = Manch.getInstance();

/* ------------ CLASS METHODS (CONT.) ------------ */

void ManchesterEncoding::beginTransmit(baud_rate_t baud_rate, uint8_t pin, uint16_t flags) {
    m_flags = flags;

    beginTransmit(baud_rate, pin);
}

void ManchesterEncoding::beginTransmit(baud_rate_t baud_rate, uint8_t pin) {
    m_txpin = pin;

    pinMode(m_txpin, OUTPUT);
    digitalWrite(m_txpin, LOW);

    m_txdelay = 1e6 / (double)baud_rate; // Delay in microseconds

    // TODO: A more sophisticated method (this is the time it takes to digitalWrite)
    m_txdelay -= 2;

}

void ManchesterEncoding::beginReceive(baud_rate_t baud_rate, uint8_t pin, uint16_t flags) {
    m_flags = flags;

    beginReceive(baud_rate, pin);
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
    m_ticks_sample = (uint32_t)((double)(midbit_ticks / MANCH_SAMPLES_PER_MIDBIT) * 0.95); // Ticks between samples per midbit (the 0.95 is heuristic)
    // TODO: Further analysis on the sampling rate

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

void ManchesterEncoding::transmitZero() {
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

void ManchesterEncoding::transmit(uint8_t message) {

    // Before sending we check if encoding is active and apply it if it is
    size_t num_of_bytes = 0;
    uint8_t *encoded_message = nullptr;

    switch (m_flags & MFLAG_CHANNEL_ENC)
    {
    case MFLAG_CHANNEL_ENC:
        encoded_message = encodeData(message, &num_of_bytes);
        break;
    
    default:
        // No encoding, we send 1 byte
        encoded_message = &message;
        num_of_bytes = 1;
        break;
    }

    // When data is encoded, more bytes will be sent (num_of_bytes)
    for (size_t ind = 0; ind < num_of_bytes; ind++) {
        uint8_t msg_to_send = encoded_message[ind];

        // Transmit a byte
        uint8_t mask = 0x80; // Transmit from MSb to LSb
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (mask & msg_to_send) {
                transmitOne();
            } else {
                transmitZero();
            }
            mask >>= 1;
        }
        
    }

    // We check if the default behaviour of the transmitter is HIGH or LOW and act accordingly
    digitalWrite(m_txpin, (m_flags & MFLAG_ALWAYS_ONE) ? HIGH: LOW);

    // This works like a trailer (it makes the receiver go to IDLE state)
    delayMicroseconds(m_txdelay *12);

}

void ManchesterEncoding::decodeRawBits() {
    // If channel encoding, decode the bits, otherwise return them raw.

    switch (m_flags & MFLAG_CHANNEL_ENC)
    {
    case MFLAG_CHANNEL_ENC:
        {
        // Decoder
        bool decoded = false;
        uint8_t decoded_message = 0;

        while (g_buffer_read_pos != g_buffer_save_pos) {
            
            decoded = decodeData(g_rawbits_buffer[g_buffer_read_pos], &decoded_message);
            g_buffer_read_pos = (g_buffer_read_pos + 1) % MANCH_RECV_BUFFER_SIZE;

            if (decoded) {
                m_byte_buffer[m_buffer_save_pos] = decoded_message;
                m_buffer_save_pos = (m_buffer_save_pos + 1) % (MANCH_RECV_BUFFER_SIZE / 2);
            }

        }
        }
        break;
    
    default:
        // Pass them raw
        while (g_buffer_read_pos != g_buffer_save_pos) {
            m_byte_buffer[m_buffer_save_pos] = g_rawbits_buffer[g_buffer_read_pos];

            m_buffer_save_pos = (m_buffer_save_pos + 1) % (MANCH_RECV_BUFFER_SIZE / 2);
            g_buffer_read_pos = (g_buffer_read_pos + 1) % MANCH_RECV_BUFFER_SIZE;
        }

        break;
    }

}

bool ManchesterEncoding::getData(uint8_t *data) {
    decodeRawBits(); // I hope this does not take too long...

    if (m_buffer_read_pos != m_buffer_save_pos) {
        *data = m_byte_buffer[m_buffer_read_pos];
        m_buffer_read_pos = (m_buffer_read_pos + 1) % (MANCH_RECV_BUFFER_SIZE / 2);
        return true;
    }

    return false;
}

uint8_t* ManchesterEncoding::encodeData(uint8_t data, size_t* size) { 
    // Encoding by sending the same message thrice (repetition code of block-length three)
    *size = 3;
    static uint8_t encoded_message[3] = {0}; // No dynamic allocation in embedded

    // Change buffer value
    encoded_message[0] = data;
    encoded_message[1] = data;
    encoded_message[2] = data;

    return encoded_message;
}

bool ManchesterEncoding::decodeData(uint8_t data, uint8_t *decoded_message) {
    // Decoding by receiving the same message thrice (repetition code of block-length three)
    static uint8_t vals[3] = {0, 0, 0}; // No dynamic allocation in embedded
    static uint8_t num_saved = 0;
    vals[num_saved] = data;

    if (num_saved == 2) { // Index starts at 0
        // decode
        *decoded_message = (vals[0] & (vals[1] | vals[2])) | (vals[1] & vals[2]);   // Bitwise Majority 3
        num_saved = 0;
        return true;
    }

    num_saved++;
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
            // If we enter, 3 equal values were received, the check for transition makes sure of that.

            if (g_samples_vals & G_SAMPLE_MASK) // We received MANCH_SAMPLES 1s
            {
                /* Save a 1 (mid-bit) */
                g_idle_check_ones++;
                if (g_idle_check_ones < 3) {
                    saveReceivedMidbit(1);
                } else {
                    g_state = RX_IDLE;
                    g_sample_counter = 0;
                    g_num_saved_midbit = 0;
                }
                g_idle_check_zeros = 0;

            } else { // We received MANCH_SAMPLES 0s
                /* Save a 0 (mid-bit)*/
                g_idle_check_zeros++;

                if (g_idle_check_zeros < 3) {
                    saveReceivedMidbit(0);
                } else {
                    g_state = RX_IDLE;
                    g_sample_counter = 0;
                    g_num_saved_midbit = 0;
                }
                g_idle_check_ones = 0;
            }

            g_sample_counter = 0;
        }
        
        break;
    
    case RX_IDLE:
    default:
        // Reset variables
        g_sample_counter = 0;
        g_num_saved_midbit = 0;
        g_idle_check_ones = 0;
        g_idle_check_zeros = 0;
        break;
    }   
    
}

void IRAM_ATTR saveReceivedMidbit(uint16_t midbit_val) {
    static uint16_t midbit_values_received = 0x0000;

    midbit_values_received <<= 1;
    midbit_values_received |= midbit_val;
    g_num_saved_midbit++;

    // If we have received 16 midbits (or 8 raw bits) we read and save the raw bits
    if (g_num_saved_midbit == 16) {
        g_num_saved_midbit = 0;

        // Depending on the convention, we speed up the midbit decoding by just looking at 1 bit 
        // [!] WARNING: this does not catch invalid raw_bits.
        #if MANCH_CONVENTION == 0 // IEEE 802.3
        int8_t i = 16;
        #elif MANCH_CONVENTION == 1 // GE THOMAS
        int8_t i = 15;
        #endif // CONVENTION

        uint16_t mask = 0x01;
        uint8_t raw_bits = 0x00;
        for (; i >= 0; i -= 2)
        {
            raw_bits <<= 1;
            raw_bits |= (midbit_values_received >> i) & mask;
        }

        // Save the raw bit to a buffer
        g_rawbits_buffer[g_buffer_save_pos] = raw_bits;
        g_buffer_save_pos = (g_buffer_save_pos + 1) % MANCH_RECV_BUFFER_SIZE;
        midbit_values_received = 0;
    }

}