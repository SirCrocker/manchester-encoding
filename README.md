### Manchester Encoding

-------------

Current implementation done only for the ESP8266

#### Todo:
- [ ] Check if the receiver method for identifying 1s and 0s works or if a more relaxed implementation is needed.
- [ ] Implement sync
- [ ] Implement it

**Features**:
- Both IEEE and G.E. Thomas conventions are available.

**Definitions**
- **Symbol**/**mid-bit**: half of a bit, corresponds to the duration of the HIGH or LOW state in a sent bit (each bit has 1 HIGH and 1 LOW state).


**Based on**: 
 - ATMEL - Manchester Coding Basics [APPLICATION NOTE] - 9164B–AUTO–07/15
 - https://github.com/mchr3k/arduino-libs-manchester/tree/master