# manchester-encoding

- [About](#about)
- [Installation](#installation)
- [Usage](#usage)
- [Support](#support)
- [Roadmap](#roadmap)
- [License](#license)
- [Based on](#based-on)
- [Implementation details](#implementation-details)

## About

manchester-encoding is an Arduino library that implements manchester encoding for the ESP8266 microcontroller. It uses digital pins for transmission and reception.

This library is made with the intention of using it in my master's thesis.

### Features
- Both IEEE and G.E. Thomas conventions are available.
- Channel encoding (repetition code of length 3) is available.
- Transmitter can be defaulted to always on or always off (_testing pending_).
- Baud rates ranging from 300 to 50,000 Bd are available.

## Installation

Drag and drop the files `Manchester.cpp` and `Manchester.h` to the directory where your libraries are installed, or use platformio package manager.

## Usage

Basic usage examples are shown in the next subsections. More examples are available in the [examples directory](./examples/).

### Transmitter

```c++
#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_TX_PIN D6

void setup() {
    // Begin serial monitor
    Serial.begin(115200);
    // Begin manchester encoding tx with baud rate of 300 Bd
    Manch.beginTransmit(BR_300, MANCH_TX_PIN);
    
    /* Do other stuff */
}

void loop() {
    // Check for messages
    if (Serial.available()) {
        
        // Read the message
        String toSend = Serial.readString();
  
        // Transmit each char in the message
        for (char *it = toSend.begin(); it != toSend.end(); it++)
        {
            Manch.transmit(*it);
            Serial.print(*it);
            yield();
        }
    }

    /* Do other stuff */
}
```

### Receiver
```c++
#include <Arduino.h>
#include "ManchesterEnc.h"

#define MANCH_RECV_PIN D6

void setup() {
    // Begin serial monitor
    Serial.begin(115200);
    // Begin manchester encoding rx with baud rate of 300 Bd
    Manch.beginReceive(BR_300, MANCH_RECV_PIN);

    /* Do other stuff */
}

void loop() {

    uint8_t data = 0;

    if (Manch.getData(&data)) {
        Serial.print((char)data);
        /* Process the data received */
    }

    /* Do other stuff */
}
```

## Support

For support or submitting bugs open an issue, I'll check them as soon as I am available.

## Roadmap

The library is almost done, so the remaining features are things that I do not need at the moment. However, there are still some things I will implement in the near future.

- [ ] Variable buffer size (definable at setup/begin).
- [ ] Capability of using custom channel encoding, passing a function at setup.
- [ ] Make it capable of receiving data from multiple pins.
- [ ] Extend the compatibility to more microcontrollers.

## License

manchester-encoding is licensed under the [MIT](./LICENSE) license.


## Based on
 - ATMEL - Manchester Coding Basics [APPLICATION NOTE] - 9164B–AUTO–07/15
 - mchr3k's [arduino-libs-manchester](https://github.com/mchr3k/arduino-libs-manchester/tree/master)

## Implementation details

If channel encoding is used, then a packet composed of multiple bytes is sent instead of a packet of one byte.
