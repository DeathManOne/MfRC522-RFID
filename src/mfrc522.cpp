#include "../include/mfrc522.h"
using namespace MfRC522;

driver::driver(int pinSDA, int pinRST, SPIClass &SPI) {
    this->_SPI = &SPI;
    this->_PIN_SDA = new int(pinSDA);
    this->_SETTINGS = new SPISettings(4000000u, MSBFIRST, SPI_MODE0);

    pinMode(*this->_PIN_SDA, OUTPUT);
    digitalWrite(*this->_PIN_SDA, HIGH);
}

driver::~driver() {
    delete this->_SPI;
    delete this->_PIN_SDA;
    delete this->_SETTINGS;
}

void driver::_spiTransferBegin() const {
    this->_SPI->beginTransaction(*this->_SETTINGS);
    digitalWrite(*this->_PIN_SDA, LOW);
}

void driver::_spiTransferEnd() const {
    digitalWrite(*this->_PIN_SDA, HIGH);
    this->_SPI->endTransaction();
}

int driver::_spiRead(int address) const {
    byte data;
    this->_spiTransferBegin();
    this->_SPI->transfer((((address << 1) & 0x7E) | 0x80));
    data = this->_SPI->transfer(0);
    this->_spiTransferEnd();
    return data;
}

void driver::_spiWrite(int address, int data) const {
    std::vector<int> tmp = {data};
    return this->_spiWrite(address, tmp);
}

void driver::_spiWrite(int address, std::vector<int> data) const {
    this->_spiTransferBegin();
    this->_SPI->transfer((address << 1) & 0x7E);
    for (int d : data)
        { this->_SPI->transfer(d); }
    this->_spiTransferEnd();
}

int driver::_pcdInitialize(int antennaLevel) const {
    this->_softReset();
    this->_spiWrite(MfRC522::RC_TX_MODE, 0x00);
    this->_spiWrite(MfRC522::RC_RX_MODE, 0x00);
    this->_spiWrite(MfRC522::RC_MOD_WIDTH, 0x26);
    this->_spiWrite(MfRC522::RC_T_MOD, 0x8D); // 0x8D | 0x80
    this->_spiWrite(MfRC522::RC_T_PRESCALER, 0x3E); // 0x3E | 0xA9
    this->_spiWrite(MfRC522::RC_T_RELOAD_HIGH, 0x03);
    this->_spiWrite(MfRC522::RC_T_RELOAD_LOW, 0xE8);
    this->_spiWrite(MfRC522::RC_TX_ASK, 0x40);
    this->_spiWrite(MfRC522::RC_MODE, 0x3D);
    
    this->pcdAntennaEnable(true);
    return this->pcdAntennaLevel(antennaLevel);
}

void driver::_pcdBitmaskSet(int address, int mask) const {
    int current = this->_spiRead(address);
    this->_spiWrite(address, current | mask);
}

void driver::_pcdBitmaskClear(int address, int mask) const {
    int current = this->_spiRead(address);
    this->_spiWrite(address, current & (~mask));
}

bool driver::_crcAppend(std::vector<int> &data) const {
    std::vector<int> crc;
    if (!this->_crcCalculate(data, crc))
        { return false; }
    data.push_back(crc[0]);
    data.push_back(crc[1]);
    return true;
}

bool driver::_crcCalculate(std::vector<int> data, std::vector<int> &crc) const {
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
    this->_spiWrite(MfRC522::RC_DIV_IRQ, 0x04);
    this->_pcdBitmaskSet(MfRC522::RC_FIFO_LEVEL, 0x80);
    this->_spiWrite(MfRC522::RC_FIFO_DATA, data);
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_CALCULATE_CRC);
    
    const uint32_t deadline = millis() + 89;
    do {
		byte irq = this->_spiRead(MfRC522::RC_DIV_IRQ);
		if ((irq & 0x04) != 0) {
			this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
			crc.push_back(this->_spiRead(MfRC522::RC_CRC_RESULT_LOW));
			crc.push_back(this->_spiRead(MfRC522::RC_CRC_RESULT_HIGHT));
            if (crc.size() != 2)
                { return false; }
			return true;
		}
	} while (static_cast<uint32_t>(millis()) < deadline);
    return false;
}

bool driver::_crcCheck(std::vector<int> data) const {
    std::vector<int> crc;
    std::vector<int> tmp = data;
    tmp.resize(tmp.size() - 2);

    if (!this->_crcCalculate(tmp, crc))
        { return false; }
    if (data[data.size() - 2] != crc[0]) { return false; }
    if (data[data.size() - 1] != crc[1]) { return false; }
    return true;
}

std::string driver::_piccCommunication(int cmd, std::vector<int> data, int framingBit, bool checkCrc, std::vector<int> &newData, int &validBits) const {
    int irqWait = 0x00;
    if (cmd == MfRC522::CMD_AUTHENTICATION) { irqWait = 0x10; }
    else if (cmd == MfRC522::CMD_TRANSCEIVE) { irqWait = 0x30; }

    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
    this->_spiWrite(MfRC522::RC_COM_IRQ, 0x7F);
    this->_pcdBitmaskSet(MfRC522::RC_FIFO_LEVEL, 0x80);
    this->_spiWrite(MfRC522::RC_BIT_FRAMING, framingBit);
    this->_spiWrite(MfRC522::RC_FIFO_DATA, data);
    this->_spiWrite(MfRC522::RC_COMMAND, cmd);

    if (cmd == MfRC522::CMD_TRANSCEIVE)
        { this->_pcdBitmaskSet(MfRC522::RC_BIT_FRAMING, 0x80); }

    bool completed = false;
    const uint32_t deadline = millis() + 1000;
    do {
        int irq = this->_spiRead(MfRC522::RC_COM_IRQ);
        if ((irq & irqWait) != 0) {
            completed = true;
            break;
        }
        if ((irq & 0x01) != 0)
            { return "PICC_TIMEOUT"; }
    } while (static_cast<uint32_t>(millis()) < deadline);
    if (!completed) { return "SOFT_TIMEOUT"; }
    this->_pcdBitmaskClear(MfRC522::RC_BIT_FRAMING, 0x80);

    int error = this->_spiRead(MfRC522::RC_ERROR);
    if ((error & 0x01) != 0) { return "PROTOCOL_ERROR"; }
    if ((error & 0x02) != 0) { return "PARITY_ERROR"; }
    if ((error & 0x04) != 0) { return "CRC_ERROR"; }
    if ((error & 0x10) != 0) { return "BUFFER_OVERFLOW"; }

    int newDataLength = this->_spiRead(MfRC522::RC_FIFO_LEVEL);
    while (newDataLength > 0) {
        newData.push_back(this->_spiRead(MfRC522::RC_FIFO_DATA));
        newDataLength -= 1;
    }

    validBits = this->_spiRead(MfRC522::RC_CONTROL) & 0x07;
    if (newData.size() > 0 && checkCrc) {
        if (newData.size() == 1 && validBits == 4) { return "MF_NACK"; }
        if (newData.size() < 2 || validBits != 0) { return "CRC_ERROR"; }
        if (!this->_crcCheck(newData)) { return "CRC_CHECK_ERROR"; }
    }
    if ((error & 0x08) != 0)
        { return "COLLISION"; }
    return "OK";
}

bool driver::_piccHalt() const {
    std::vector<int> buffer = MfRC522::MF_HALT;
    if (!this->_crcAppend(buffer))
        { return false; }
    this->_pcdBitmaskClear(MfRC522::RC_STATUS_2, 0x80);

    std::vector<int> newData;
    int validBits;
    std::string status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffer, 0x00, false, newData, validBits);

    this->MFDeauthenticate();
    return status == "PICC_TIMEOUT" || status == "SOFT_TIMEOUT";
}

void driver::_piccReset() const {
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
    this->MFDeauthenticate();
    this->_pcdBitmaskClear(MfRC522::RC_COLL, 0x80);
    delay(50);
}

std::string driver::_piccType(int sak) const {
    sak = sak & 0x7F;
    switch (sak) {
        case 0x00:
            return "MIFARE_UL";
            break;
        case 0x01:
            return "TNP3XXX";
            break;
        case 0x04:
            return "NOT_COMPLETE";
            break;
        case 0x08:
            return "MIFARE_1K";
            break;
        case 0x09:
            return "MIFARE_MINI";
            break;
        case 0x10:
        case 0x11:
            return "MIFARE_PLUS";
            break;
        case 0x18:
            return "MIFARE_4K";
            break;
        case 0x20:
            return "ISO_14443_4";
            break;
        case 0x40:
            return "ISO_18092";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

std::string driver::_MFTransceive(int data, bool acceptTimeout) const {
    std::vector<int> tmp = {data};
    return this->_MFTransceive(tmp, acceptTimeout);
}

std::string driver::_MFTransceive(std::vector<int> data, bool acceptTimeout) const {
    if (!this->_crcAppend(data))
        { return "CRC_ERROR"; }

    std::vector<int> newData;
    int validBits;
    std::string status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, data, 0x00, false, newData, validBits);
    if (acceptTimeout && status == "PICC_TIMEOUT") { return "OK"; }
    if (acceptTimeout && status == "SOFT_TIMEOUT") { return "OK"; }
    if (status != "OK") { return status; }

    if (newData.size() != 1) { return "DATA_ERROR"; }
    if (validBits != 4) { return "VALID_BITS_ERROR"; }
    if (newData[0] != 0x0A) { return "ACK_ERROR"; }
    return "OK";
}

bool driver::_MFGetValue(int blockAddress, int &value) const {
    std::vector<int> data;
    if (!this->MFRead(blockAddress, data))
        { return false; }
    
    value = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
    return true;
}

std::string driver::_MFTwoStep(int cmd, int blockAddress, int value) const {
    std::string status = this->_MFTransceive({cmd, blockAddress}, false);
    if (status != "OK") { return status; }

    std::vector<int> data = {value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF };
    return this->_MFTransceive(data, true);
}

bool driver::_MFAccessBits(std::vector<int> accessbits, std::vector<int> &data) const {
    if (accessbits.size() != 4) { return false; }
    for (int accessBit : accessbits) {
        if(accessBit < 0 || accessBit > 7)
            { return false; }
    }

    int c1 = (accessbits[3] & 4) << 1 | (accessbits[2] & 4) << 0 | (accessbits[1] & 4) >> 1 | (accessbits[0] & 4) >> 2;
    int c2 = (accessbits[3] & 2) << 2 | (accessbits[2] & 2) << 1 | (accessbits[1] & 2) << 0 | (accessbits[0] & 2) >> 1;
    int c3 = (accessbits[3] & 1) << 3 | (accessbits[2] & 1) << 2 | (accessbits[1] & 1) << 1 | (accessbits[0] & 1) << 0;

    std::vector<int> tmp = {(~c2 & 0xF) << 4 | (~c1 & 0xF)}; // byte 6
    tmp.push_back(c1 << 4 | (~c3 & 0xF)); // byte 7
    tmp.push_back(c3 << 4 | c2); // byte 8

    if (tmp.size() != 3) { return false; }
    data = tmp;
    return true;
}

std::string driver::_MFOpenBackdoor(bool maxTryPerUnbrickCode) const {
    std::vector<std::vector<int>> unbrickCodes = {
        {0x01, 0x02, 0x03, 0x04, 0x04, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x01, 0x23, 0x45, 0x67, 0x00, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x12, 0x34, 0x56, 0x78, 0x08, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    };
    for (std::vector<int> unbrickCode : unbrickCodes) {
        for (int passing = 0; passing < maxTryPerUnbrickCode; passing++) {
            if (!this->_piccHalt())
                { return "ERROR_HALT"; }

            std::string status = this->_MFTransceive(MfRC522::MF_UID_PERSONALIZE, false);
            if (status == "OK")
                { return this->_MFTransceive(MfRC522::MF_SET_MOD_TYPE, false); }
            if (passing >= maxTryPerUnbrickCode) { break; }
            if (this->MFWrite(0, unbrickCode)) { continue; }
        }
    }
    return "UID_BRICKED";
}

void driver::_softReset() const {
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_SOFT_RESET);
    delay(5);
    this->_spiWrite(MfRC522::RC_T_MOD, 0x87);
    this->_spiWrite(MfRC522::RC_T_PRESCALER, 0xFF);
    this->_spiWrite(MfRC522::RC_TX_ASK, 0x40);
    this->_spiWrite(MfRC522::RC_MODE, 0x3D);
}

int driver::pcdAntennaLevel(int level) const {
    if (level < 0) { level = 1; }
    if (level > 7) { level = 7; }
    this->_spiWrite(MfRC522::RC_RFC_FG, level << 4);
    return (this->_spiRead(MfRC522::RC_RFC_FG) & 0x70) >> 4;
}

void driver::pcdAntennaEnable(bool enable) const {
    if (enable)
        { this->_pcdBitmaskSet(MfRC522::RC_TX_CONTROL, 0x03); }
    else { this->_pcdBitmaskClear(MfRC522::RC_TX_CONTROL, 0x03); }
}

bool driver::piccWaitTag(int &antennaLevel, int timeout) const {
    bool newCardPresent = false;
    const uint32_t deadline = millis() + (1000 * timeout);
    do {
        this->_pcdInitialize(antennaLevel);
        this->_spiWrite(MfRC522::RC_COM_IRQ, 0x00);
        this->_spiWrite(MfRC522::RC_COM_I_EN, 0xA0);
        this->_spiWrite(MfRC522::RC_FIFO_DATA, 0x26);
        this->_spiWrite(MfRC522::RC_COMMAND, 0x0C);
        this->_spiWrite(MfRC522::RC_BIT_FRAMING, 0x87);
        newCardPresent = this->piccRequest(false);
        delay(10);
    } while (!newCardPresent && static_cast<uint32_t>(millis()) < deadline);
    if (!newCardPresent) { return false; }

    antennaLevel = this->_pcdInitialize(antennaLevel);
    return true;
}

bool driver::piccReestablishCommunication(std::vector<int> uid, std::string &status) const {
    if (!this->_piccHalt()) { return false; }
    if (!this->piccRequest(true)) { return false; }

    try {
        std::vector<int> newUid;
        int newSak;
        std::string newType;

        bool success = this->piccSelect(newUid, newSak, newType, status);
        return success && newUid == uid;
    } catch (...) { return false; }
}

bool driver::piccRequest(bool wakeup) const {
    std::vector<int> cmd = {MfRC522::MF_REQ_A};
    if (wakeup) { cmd = {MfRC522::MF_WUP_A}; }

    this->_piccReset();
    std::vector<int> data;
    int validBits;
    std::string status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, cmd, 0x07, false, data, validBits);

    return status == "OK" && validBits == 0;
}

bool driver::piccSelect(std::vector<int> &uid, int &sak, std::string &type, std::string &status) const {
    uid = {};
    sak = 0x04;
    type = "";
    status = "";

    std::vector<int> cascades = {
        MfRC522::MF_CL_1_ANTICOLLISION[0],
        MfRC522::MF_CL_2_ANTICOLLISION[0],
        MfRC522::MF_CL_3_ANTICOLLISION[0]
    };
    this->_piccReset();

    for (int cascade : cascades) {
        std::vector<int> buffer = {cascade};
        int levelKnownBits = 0;
        bool timeout = true;

        std::vector<int> data;
        int validBits;

        for (int i = 0; i <= 32; i++) {
            int txLastBits;
            if (levelKnownBits >= 32) {
                if (buffer.size() > 6) { buffer.resize(6); }
                bool bufferDirty = (buffer.size() != 6);
                bufferDirty |= std::any_of(buffer.begin(), buffer.end(), [](int byte) { return false; });
                if (bufferDirty) {
                    buffer = {cascade};
                    levelKnownBits = 0;
                    continue;
                }
                int txLastBits = 0;

                buffer[1] = 0x70;
                if (buffer.size() < 7)
                    { buffer.push_back(buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]); }
                else { buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]; }
                if (!this->_crcAppend(buffer)) {
                    status = "ERROR_CRC";
                    return false;
                }
            } else {
                int txLastBits = levelKnownBits % 8;
                int uidFullByte = levelKnownBits / 8;
                int allFUllByte = 2 + uidFullByte;

                if (buffer.size() < 2)
                    { buffer.push_back((allFUllByte << 4) + txLastBits); }
                else { buffer[1] = (allFUllByte << 4) + txLastBits; }

                int bufferLength = allFUllByte + (txLastBits > 0 ? 1 : 0);
                if (buffer.size() > bufferLength + 1)
                    { buffer.resize(bufferLength + 1); }
            }
            int framingBit = (txLastBits << 4) + txLastBits;
            status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffer, framingBit, false, data, validBits);
            if (status != "OK" && status != "COLLISION") { return false; }
            if (data.empty()) {
                status = "DATA_EMPTY";
                return false;
            }
            if (levelKnownBits < 32) {
                if (txLastBits != 0) {
                    buffer.back() |= data.front();
                    data.erase(data.begin());
                }
                buffer.insert(buffer.end(), data.begin(), data.end());
            }
            if (status == "COLLISION") {
                int collision = this->_spiRead(MfRC522::RC_COLL);
                if ((collision & 0x20) != 0) {
                    status = "COLLISION_ERROR";
                    return false;
                }

                int collisionPosition = collision & 0x1F;
                if (collisionPosition == 0) { collisionPosition = 32; }
                if (collisionPosition <= levelKnownBits) {
                    status = "COLLISION_POSITION_ERROR";
                    return false;
                }
                
                levelKnownBits = collisionPosition;
                buffer.back() |= 1 << ((levelKnownBits - 1) % 8);
            } else {
                if (levelKnownBits >= 32) {
                    timeout = false;
                    break;
                }
                levelKnownBits = 32;
            }
        }
        if (timeout) {
            status = "SOFT_TIMEOUT";
            return false;
        }
        if (data.size() != 3 || validBits != 0) {
            status = "SAK_ERROR";
            return false;
        }
        if (!this->_crcCheck(data)) {
            status = "CRC_CHECK_ERROR";
            return false;
        }

        sak = data[0];
        uid.assign(buffer.begin() + 2, buffer.end() - 2);
        if ((sak & 0x04) == 0) { break; }
        type = this->_piccType(sak);
    }
    return true;
}

bool driver::MFAuthenticate(bool useKeyB, int blockAddress, std::vector<int> key,  std::vector<int> uid) const {
    int keyType = MfRC522::MF_AUTH_A;
    if (useKeyB) { keyType = MfRC522::MF_AUTH_B; }

    if (key.size() < 6) { return false; }
    else { key.resize(6); }

    if (uid.size() < 4 ) { return false; }
    else { uid.resize(4); }

    std::vector<int> buffer = {keyType, blockAddress};
    for (int k : key) { buffer.push_back(k); }
    for (int u : uid) { buffer.push_back(u); }

     std::vector<int> data;
     int validBits;
     if (this->_piccCommunication(MfRC522::CMD_AUTHENTICATION, buffer, 0x00, false, data, validBits) != "OK")
        { return false; }
    return (this->_spiRead(MfRC522::RC_STATUS_2) & 0x08) != 0;
}

void driver::MFDeauthenticate() const {
    this->_pcdBitmaskClear(MfRC522::RC_STATUS_2, 0x08);
}

bool driver::MFSevenByteUidFirstInit(int typeFn) const {
    std::vector<int> data = {MfRC522::MF_UID_PERSONALIZE};
    switch (typeFn) {
    case 0:
        data.push_back(0x00);
        break;
    case 1:
        data.push_back(0x40);
        break;
    case 2:
        data.push_back(0x20);
        break;
    case 3:
        data.push_back(0x60);
        break;
    default:
        break;
    }

    if (data.size() == 1) { return false; }
    if (!this->_crcAppend(data)) { return false; }
    return this->_MFTransceive(data, false) == "OK";
}

bool driver::MFDecrement(int blockAddress, int delta) const {
    return this->_MFTwoStep(MfRC522::MF_DECREMENT, blockAddress, delta) == "OK";
}

bool driver::MFIncrement(int blockAddress, int delta) const {
    return this->_MFTwoStep(MfRC522::MF_INCREMENT, blockAddress, delta) == "OK";
}

bool driver::MFRestore(int blockAddress, int &result) const {
    if (this->_MFTwoStep(MfRC522::MF_RESTORE, blockAddress, 0) != "OK")
        { return false; }
    if (!this->MFTransfer(blockAddress, result))
        { return false; }
    return this->_MFGetValue(blockAddress, result);
}

bool driver::MFTransfer(int blockAddress, int &result) const {
    std::vector<int> data = {MfRC522::MF_TRANSFER, blockAddress};
    if (this->_MFTransceive(data, false) != "OK")
        { return false; }
    return this->_MFGetValue(blockAddress, result);
}

bool driver::MFFormatValueBlock(int blockAddress, int value) const {
    std::vector<int> data = {value & 0xFF};
    data.push_back((data[0] >> 8) & 0xFF);
    data.push_back((data[0] >> 16) & 0xFF);
    data.push_back((data[0] >> 24) & 0xFF);
    data.push_back(~data[0] & 0xFF);
    data.push_back(~data[1] & 0xFF);
    data.push_back(~data[2] & 0xFF);
    data.push_back(~data[3] & 0xFF);
    data.push_back(data[0]);
    data.push_back(data[1]);
    data.push_back(data[2]);
    data.push_back(data[3]);
    data.push_back(blockAddress & 0xFF);
    data.push_back(~data[12] & 0xFF);
    data.push_back(data[12]);
    data.push_back(data[13]);
    return this->MFWrite(data[12], data);
}

bool driver::MFRead(int blockAddress, std::vector<int> &data) const {
    std::vector<int> buffer = {MfRC522::MF_READ, blockAddress};
    if (!this->_crcAppend(buffer))
        { return false; }

    int validBits;
    if (this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffer, 0x00, true, data, validBits) != "OK")
        { return false; }
    data.resize(data.size() > 2 ? data.size() - 2 : 0);
    return true;
}

bool driver::MFWrite(int blockAddress, std::vector<int> data) const {
    if (data.size() < 16) { return false; }
    if (data.size() > 16) { data.resize(16); }

    std::vector<int> tmp = {MfRC522::MF_WRITE, blockAddress};
    if (this->_MFTransceive(tmp, false) != "OK")
        { return false; }
    return this->_MFTransceive(data, false) == "OK";
}

bool driver::MFChangeUid(int maxTryPerUnbrickCode, std::vector<int> data) const {
    if (data.size() < 16) { return false; }
    if (data.size() > 16) { data.resize(16); }
    if (data[4] != data[0] ^ data[1] ^ data[2] ^ data[3])
        { return false; }
    
    if (this->_MFOpenBackdoor(maxTryPerUnbrickCode) != "OK")
        { return false; }
    if (!this->MFWrite(0, data))
        { return false; }
    
    if (this->_piccHalt())
        { this->piccRequest(true); }
    return true;
}

bool driver::MFChangeTrailer(int sector, std::vector<int> keyA, std::vector<int> keyB, std::vector<int> accessBits) const {
    if (keyA.size() != 6) { return false; }
    if (keyB.size() != 6) { return false; }
    if (accessBits.size() != 4) { return false; }

    for (int accessBit : accessBits) {
        if (accessBit < 0 || accessBit > 7)
            { return false; }
    }

    std::vector<int> data;
    if (!this->_MFAccessBits(accessBits, data))
        { return false; }
    
    std::vector<int> buffer;
    for (int key : keyA)
        { buffer.push_back(key); }
    for (int accessBit : data)
        { buffer.push_back(accessBit); }
    buffer.push_back(data[0] ^ data[1] ^ data[2]);
    for (int key : keyB)
        { buffer.push_back(key); }
    
    int block = (sector * 4) + 3;
    return this->MFWrite(block, buffer);
}

std::vector<std::string> driver::MFDump(bool useKeyB, std::vector<int> key, int sectorCount, std::vector<int> uid) const {
    std::vector<std::string> tmp;
    for (int block = 0; block < sectorCount * 4; block++) {
        int byte = block % 4;
        int sector = (block - byte) / 4;

        if (byte == 0) {
            if (!this->MFAuthenticate(useKeyB, block, key, uid))
                { continue; }
        }

        std::vector<int> data;
        if (!this->MFRead(block, data))
            { tmp.push_back(std::to_string(sector) + "-" + std::to_string(byte) + " [ERROR]"); }
        else {
            std::ostringstream oss;
            oss.width(3);
            oss << std::right << sector << "-" << byte << " [";
            for (size_t i = 0; i < data.size(); ++i) {
                oss << data[i];
                if (i != data.size() - 1)
                    { oss << ", "; }
            }
            oss << "]";
            tmp.push_back(oss.str());
        }
    }
    return tmp;
}

void driver::softPowerDown() const {
    this->_pcdBitmaskSet(MfRC522::RC_COMMAND, 0x10);
}

bool driver::softPowerUp(int timeout) const {
    this->_pcdBitmaskClear(MfRC522::RC_COMMAND, 0x10);
    const uint32_t deadline = millis() + (1000 * timeout);
    do {
        if ((this->_spiRead(MfRC522::RC_COMMAND) & 0x10) != 0x10)
            { return true; }
        delay(100);
    } while (static_cast<uint32_t>(millis()) < deadline);
    return false;
}
