### Manchester Encoding

-------------

Current implementation done only for the ESP8266

#### Todo:
- [ ] Test sync and trailer (they are implemented awfully, but in the meantime its _ok_)
- [x] Implement sync (header+trailer should add 1 byte in length!)
- [ ] Implement it
- [x] Check if the receiver method for identifying 1s and 0s works or if a more relaxed implementation is needed.


**Features**:
- Both IEEE and G.E. Thomas conventions are available.

**Definitions**
- **Symbol**/**mid-bit**: half of a bit, corresponds to the duration of the HIGH or LOW state in a sent bit (each bit has 1 HIGH and 1 LOW state).


**Based on**: 
 - ATMEL - Manchester Coding Basics [APPLICATION NOTE] - 9164B–AUTO–07/15
 - https://github.com/mchr3k/arduino-libs-manchester/tree/master