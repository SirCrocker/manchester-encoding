### Manchester Encoding

-------------

Current implementation done only for the ESP8266.

#### Todo:
- [ ] Implement variable buffer size (definable at setup)
- [ ] Implement way to define custom functions for encoding/decoding the data.
- [ ] More thorough readme.


**Features**:
- Both IEEE and G.E. Thomas conventions are available.
- Channel encoding (repetition code of length 3) is available.
- Transmitter can be defaulted to always on or always off.
- Multiple baud rates are available.

**Definitions**
- **Symbol**/**mid-bit**: half of a bit, corresponds to the duration of the HIGH or LOW state in a sent bit (each bit has 1 HIGH and 1 LOW state).


**Based on**: 
 - ATMEL - Manchester Coding Basics [APPLICATION NOTE] - 9164B–AUTO–07/15
 - https://github.com/mchr3k/arduino-libs-manchester/tree/master


 **Implementation Details**:
 - Sync signal:
 - "Trailer Signal"
 - Sampling rate
 - Detecting method
 - If sending too much text at once, increase the buffer size