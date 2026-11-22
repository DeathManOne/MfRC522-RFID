# **MFRC522 / RFID** - a complete driver
MFRC522 RFID reader driver for the Arduino framework on ESP32.

See the [changelog](CHANGELOG.md) for release notes.
See the [example sketches](examples/README.md) for wiring and individual tests.

## License

Licensed under the [GNU General Public License v3.0 or later](LICENSE).

## Hardware verification

Tests on an Arduino Nano ESP32 and MIFARE Classic 1K cards covered tag
detection and reselection, authentication, block reads and writes, full
forward and reverse dumps, value-block operations, trailer protection and
update, antenna gain, and soft power-down and wake-up. With two cards in
the field, the driver resolved a collision at UID bit 6 and selected one
of the expected UIDs.

Hardware verification is pending for:

- MIFARE Classic 4K addressing and operations.
- `MFSevenByteUidFirstInit()` on a MIFARE Classic card with a 7-byte UID.
- `MFChangeUid()` and `MFUnbrickUid()` on compatible rewritable cards.

## **INITIALIZE**
```c++
#include <MfRC522.h>
MfRC522::driver *_RFID;

int RFID_PIN_SDA = 5;
int RFID_ANTENNA_LEVEL = 4;
int RFID_WAIT_TAG_TIMEOUT = 15;

int sak;
std::string type;
std::string status;
std::vector<int> uid;
std::vector<int> datasRead;
std::vector<int> datasWrite = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5};
std::vector<int> key = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  SPI.begin();
  _RFID = new MfRC522::driver(RFID_PIN_SDA, SPI);
}

void loop() {
    if (!_RFID->piccWaitTag(RFID_WAIT_TAG_TIMEOUT, RFID_ANTENNA_LEVEL, status))
        { return; }
    if (!_RFID->piccRequest(status))
        { return; }
    if (!_RFID->piccSelect(uid, sak, type, status))
        { return; }
}
```

## **COMMANDS**

### **Unselect** / **Reestablishing** the connection
```c++
    _RFID->piccUnselect();
    bool success = _RFID->piccReestablishCommunication(uid, status);
```

### **Dump** all datas
```c++
    bool useKeyB = false; // Default to Key A. Set to true to use Key B.
    int sectorCount = 16; // Default for Classic 1K. Use 40 for Classic 4K.
    std::vector<std::string> datasFormatted =_RFID->MFDump(uid, key, useKeyB, sectorCount);
    for (std::string data : datasFormatted)
        { Serial.println(data.c_str()); }
```

### **Authentication**
```c++
    int sector = 1; // 0 to 15 for Classic 1K, 0 to 39 for Classic 4K.
    bool useKeyB = false; // optional
    bool success = _RFID->MFAuthenticate(sector, key, uid, status, useKeyB);
```

### **Read**
Authenticate the target sector before reading, writing, or using a value block.
The examples below use sector 1.
```c++
    int sector = 1;
    int block = 1;
    bool success = _RFID->MFRead(sector, block, datasRead, status);
    // Alternative using an absolute block address:
    // int address = 5; // Sector 1, block 1
    // bool success = _RFID->MFRead(address, datasRead, status);
```

### **Write**
```c++
    int sector = 1;
    int block = 1;
    bool success = _RFID->MFWrite(sector, block, datasWrite, status);
    // Alternative using an absolute block address:
    // int address = 5; // Sector 1, block 1
    // bool success = _RFID->MFWrite(address, datasWrite, status);
```

### **Value block**
- Always format the block with **MFFormatValueBlock** before **increment**, **decrement**, **restore** and **transfer**
- Only need to format **once**, **not more**, ***otherwise you will erase your data already written***

#### **Format value block**
```c++
    int sector = 1;
    int block = 1;
    int value = 0; // optional
    bool success = _RFID->MFFormatValueBlock(sector, block, status, value);
    // Alternative using an absolute block address:
    // int address = 5; // Sector 1, block 1
    // bool success = _RFID->MFFormatValueBlock(address, status, value);
```

#### **Decrement**
```c++
    int sector = 1;
    int block = 1;
    int delta = 1; // optional | default delta is 1
    bool success = _RFID->MFDecrement(sector, block, status, delta);
    // Alternative using an absolute block address:
    // int address = 5; // Sector 1, block 1
    // bool success = _RFID->MFDecrement(address, status, delta);
```

#### **Increment**
```c++
    int sector = 1;
    int block = 1;
    int delta = 1; // optional | default delta is 1
    bool success = _RFID->MFIncrement(sector, block, status, delta);
    // Alternative using an absolute block address:
    // int address = 5; // Sector 1, block 1
    // bool success = _RFID->MFIncrement(address, status, delta);
```

#### **Restore**
```c++
    int sector = 1;
    int block = 1;
    int result; // value from restore will be appear here
    bool success = _RFID->MFRestore(sector, block, result, status);
    // Alternative using an absolute block address:
    // int address = 5; // Sector 1, block 1
    // bool success = _RFID->MFRestore(address, result, status);
```

#### **Transfer**
Always use **MFTransfer** after
- **increment**
- **decrement**
- but not required after **restore**, it's already did in driver
```c++
    int sector = 1;
    int block = 1;
    int result; // value from restore will be appear here
    bool success = _RFID->MFTransfer(sector, block, result, status);
    // Alternative using an absolute block address:
    // int address = 5; // Sector 1, block 1
    // bool success = _RFID->MFTransfer(address, result, status);
```

### **If needed**

#### **Antenna gain** and **enable**/**disable**
```c++
    int antennalLevel = 4; // will be read but also be changed if needed by PCD
    _RFID->pcdAntennaLevel(antennalLevel);

    bool enable = true; // optional
    _RFID->pcdAntennaEnable(enable);
```

#### **Soft power**
```c++
    // shut down
    _RFID->softPowerDown();

    // power up
    _RFID->softPowerUp();
```

#### **Change a sector trailer**
Authenticate the sector with a key that permits reading and writing its
trailer. `MFChangeTrailer()` reads the trailer first to preserve its user
byte. The write is attempted only if that read succeeds.
```c++
    int sector = 1;
    std::vector<int> keyA = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // default for keyA
    std::vector<int> keyB = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // default for keyB
    std::vector<int> accessBits = {
        0, // default for block 0
        0, // default for block 1
        0, // default for block 2
        1 // default for block 3
    };
    bool success = _RFID->MFChangeTrailer(sector, keyA, keyB, accessBits, status);
```

#### **Delete**
```c++
    delete _RFID;
```




