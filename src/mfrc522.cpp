#include "../include/mfrc522.h"
using namespace MfRC522;

driver::driver(int pinSDA, SPIClass &SPI) {
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
    this->_spiTransferBegin();
    this->_SPI->transfer((((address << 1) & 0x7E) | 0x80));
    int data = this->_SPI->transfer(0);
    this->_spiTransferEnd();
    return data;
}

void driver::_spiWrite(int address, int data) const {
    std::vector<int> datas = {data};
    return this->_spiWrite(address, datas);
}

void driver::_spiWrite(int address, std::vector<int> datas) const {
    this->_spiTransferBegin();
    this->_SPI->transfer((address << 1) & 0x7E);
    for (int data : datas)
        { this->_SPI->transfer(data); }
    this->_spiTransferEnd();
}

void driver::_pcdInitialize(int &antennaLevel) const {
    this->_softReset();

    this->_spiWrite(MfRC522::RC_TX_MODE, 0x00);
    this->_spiWrite(MfRC522::RC_RX_MODE, 0x00);
    this->_spiWrite(MfRC522::RC_MOD_WIDTH, 0x26);
    this->_spiWrite(MfRC522::RC_T_MOD, 0x80); // 0x8D | 0x80
    this->_spiWrite(MfRC522::RC_T_PRESCALER, 0xA9); // 0x3E | 0xA9
    this->_spiWrite(MfRC522::RC_T_RELOAD_HIGH, 0x03);
    this->_spiWrite(MfRC522::RC_T_RELOAD_LOW, 0xE8);
    this->_spiWrite(MfRC522::RC_TX_ASK, 0x40);
    this->_spiWrite(MfRC522::RC_MODE, 0x3D);

    this->pcdAntennaEnable();
    this->pcdAntennaLevel(antennaLevel);
}

void driver::_pcdBitmaskSet(int address, int mask) const {
    int current = this->_spiRead(address);
    this->_spiWrite(address, current | mask);
}

void driver::_pcdBitmaskClear(int address, int mask) const {
    int current = this->_spiRead(address);
    this->_spiWrite(address, current & (~mask));
}

bool driver::_crcAppend(int &data) const {
    std::vector<int> datas = {data};
    return this->_crcAppend(datas);
}

bool driver::_crcAppend(std::vector<int> &datas) const {
    std::vector<int> crc;
    if (!this->_crcCalculate(datas, crc))
        { return false; }
    datas.insert(datas.end(), crc.begin(), crc.end());
    return true;
}

bool driver::_crcCalculate(int data, std::vector<int> &crc) const {
    std::vector<int> datas = {data};
    return this->_crcCalculate(datas, crc);
}

bool driver::_crcCalculate(std::vector<int> datas, std::vector<int> &crc) const {
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
    this->_spiWrite(MfRC522::RC_DIV_IRQ, 0x04);
    this->_pcdBitmaskSet(MfRC522::RC_FIFO_LEVEL, 0x80);
    this->_spiWrite(MfRC522::RC_FIFO_DATA, datas);
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

bool driver::_crcCheck(std::vector<int> datas) const {
    std::vector<int> crc;
    std::vector<int> buffers = datas;
    buffers.resize(buffers.size() - 2);

    if (!this->_crcCalculate(buffers, crc))
        { return false; }
    if (datas[datas.size() - 2] != crc.front())
        { return false; }
    if (datas.back() != crc.back())
        { return false; }
    return true;
}

std::string driver::_piccCommunication(int cmd, int buffer, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const {
    std::vector<int> buffers = {buffer};
    return this->_piccCommunication(cmd, buffers, framingBit, checkCrc, datas, validBits);
}

std::string driver::_piccCommunication(int cmd, std::vector<int> buffers, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const {
    datas.clear();
    validBits = 0;
    int irqWait = 0x00;

    if (cmd == MfRC522::CMD_AUTHENTICATION)
        { irqWait = 0x10; }
    else if (cmd == MfRC522::CMD_TRANSCEIVE)
        { irqWait = 0x30; }

    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
    this->_spiWrite(MfRC522::RC_COM_IRQ, 0x7F);
    this->_pcdBitmaskSet(MfRC522::RC_FIFO_LEVEL, 0x80);
    this->_spiWrite(MfRC522::RC_BIT_FRAMING, framingBit);
    this->_spiWrite(MfRC522::RC_FIFO_DATA, buffers);
    this->_spiWrite(MfRC522::RC_COMMAND, cmd);

    if (cmd == MfRC522::CMD_TRANSCEIVE)
        { this->_pcdBitmaskSet(MfRC522::RC_BIT_FRAMING, 0x80); }

    bool completed = false;
    const uint32_t deadline = millis() + 36;
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

    int datasLength = this->_spiRead(MfRC522::RC_FIFO_LEVEL);
    while (datasLength > 0) {
        datas.push_back(this->_spiRead(MfRC522::RC_FIFO_DATA));
        datasLength--;
    }

    validBits = this->_spiRead(MfRC522::RC_CONTROL) & 0x07;
    if (datas.size() > 0 && checkCrc) {
        if (datas.size() == 1 && validBits == 4) { return "MF_NACK"; }
        if (datas.size() < 2 || validBits != 0) { return "CRC_ERROR"; }
        if (!this->_crcCheck(datas)) { return "CRC_CHECK_ERROR"; }
    }
    if ((error & 0x08) != 0)
        { return "COLLISION"; }
    return "OK";
}

bool driver::_piccHalt(std::string &status, bool withUnselect) const {
    status.clear();

    std::vector<int> buffers = MfRC522::MF_HALT;
    if (!this->_crcAppend(buffers)) {
        status = "ERROR_CRC";
        return false;
    }

    int validBits;
    std::vector<int> datas;
    status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffers, 0x00, false, datas, validBits);

    if (withUnselect)
        { this->piccUnselect(); }
    return status == "PICC_TIMEOUT" || status == "SOFT_TIMEOUT";
}

void driver::_piccReset() const {
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
    this->piccUnselect();
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

std::string driver::_MFTransceive(int buffer, bool acceptTimeout, bool ignoreCrc) const {
    std::vector<int> buffers = {buffer};
    return this->_MFTransceive(buffers, acceptTimeout, ignoreCrc);
}

std::string driver::_MFTransceive(std::vector<int> buffers, bool acceptTimeout, bool ignoreCrc) const {
    if (!ignoreCrc) {
        if (!this->_crcAppend(buffers))
            { return "ERROR_CRC"; }
    }
    int validBits;
    std::vector<int> datas;

    std::string status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffers, 0x00, false, datas, validBits);
    if (acceptTimeout && (status == "PICC_TIMEOUT" || status == "SOFT_TIMEOUT"))
        { return "OK"; }
    if (status != "OK")
        { return status; }

    if (datas.size() != 1)
        { return "DATA_ERROR"; }
    if (validBits != 4)
        { return "VALID_BITS_ERROR"; }
    if (datas.front() != 0x0A)
        { return "ACK_ERROR"; }
    return "OK";
}

bool driver::_MFGetValue(int address, int &value, std::string &status) const {
    value = 0;
    status.clear();

    std::vector<int> datas;
    if (!this->MFRead(address, datas, status))
        { return false; }
    value = (datas[3] << 24) + (datas[2] << 16) + (datas[1] << 8) + datas.front();
    return true;
}

std::string driver::_MFTwoStep(int cmd, int address, int value) const {
    std::vector<int> buffers = {cmd, address};
    std::string status = this->_MFTransceive(buffers);
    if (status != "OK")
        { return status; }

    buffers = {
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF
    };
    return this->_MFTransceive(buffers, true);
}

bool driver::_MFAccessBits(std::vector<int> accessBits, std::vector<int> &datas, std::string &status) const {
    datas.clear();
    status.clear();

    if (accessBits.size() != 4) {
        status = "ERROR_ACCESS_BITS_SIZE";
        return false;
    }
    for (int accessBit : accessBits) {
        if(accessBit < 0 || accessBit > 7) {
            status = "ERROR_ACCESS_BIT_SIZE";
            return false;
        }
    }

    int c1 = (accessBits.back() & 4) << 1
        | (accessBits[2] & 4) << 0
        | (accessBits[1] & 4) >> 1
        | (accessBits.front() & 4) >> 2;
    int c2 = (accessBits.back() & 2) << 2
        | (accessBits[2] & 2) << 1
        | (accessBits[1] & 2) << 0
        | (accessBits.front() & 2) >> 1;
    int c3 = (accessBits.back() & 1) << 3
        | (accessBits[2] & 1) << 2
        | (accessBits[1] & 1) << 1
        | (accessBits.front() & 1) << 0;

    std::vector<int> buffers = {
        (~c2 & 0xF) << 4 | (~c1 & 0xF), // byte 6
        c1 << 4 | (~c3 & 0xF), // byte 7
        c3 << 4 | c2 // byte 8
    };

    if (buffers.size() != 3) {
        status = "ERROR_INTERNAL_MF_ACCESS_BITS";
        return false;
    }
    datas = buffers;
    status = "OK";
    return true;
}

bool driver::_MFWriteTrailer(int sector, std::vector<int> datas, std::string &status) const {
    status.clear();

    if (datas.size() < 16) {
        status = "ERROR_DATAS";
        return false;
    } else { datas.resize(16); }

    int address = this->_getBlockAddress(sector, 3);
    std::vector<int> buffers = {MfRC522::MF_WRITE, address};
    status = this->_MFTransceive(buffers);
    if (status != "OK")
        { return false; }

    status = this->_MFTransceive(datas);
    return status == "OK";
}

bool driver::_MFOpenBackdoor(std::string &status, bool withUnselect) const {
    if (withUnselect)
        { this->piccUnselect(); }
    bool dontCare = this->_piccHalt(status, false);

    status = this->_MFTransceive(MfRC522::MF_UID_PERSONALIZE, false, true);
    if (status != "OK")
        { return false; }

    status = this->_MFTransceive(MfRC522::MF_SET_MOD_TYPE, false, true);
    return status == "OK";
}

void driver::_softReset() const {
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_SOFT_RESET);
    delay(10);
    this->_spiWrite(MfRC522::RC_T_MOD, 0x87);
    this->_spiWrite(MfRC522::RC_T_PRESCALER, 0xFF);
    this->_spiWrite(MfRC522::RC_TX_ASK, 0x40);
    this->_spiWrite(MfRC522::RC_MODE, 0x3D);
}

int driver::_getBlockAddress(int sector, int block) const {
    return (sector * 4) + block;
}

void driver::_getSectorBlock(int blockAddress, int &sector, int &block) const {
    block = blockAddress % 4;
    sector = (blockAddress - block) / 4;
}

void driver::pcdAntennaLevel(int &level) const {
    if (level < 0)
        { level = 1; }
    if (level > 7)
        { level = 7; }
    this->_spiWrite(MfRC522::RC_RFC_FG, level << 4);
    level = (this->_spiRead(MfRC522::RC_RFC_FG) & 0x70) >> 4;
}

void driver::pcdAntennaEnable(bool enable) const {
    if (enable)
        { this->_pcdBitmaskSet(MfRC522::RC_TX_CONTROL, 0x03); }
    else { this->_pcdBitmaskClear(MfRC522::RC_TX_CONTROL, 0x03); }
}

bool driver::piccWaitTag(int timeout, int &antennaLevel, std::string &status) const {
    status.clear();
    bool newCardPresent = false;

    const uint32_t deadline = millis() + (1000 * timeout);
    do {
        this->_pcdInitialize(antennaLevel);
        this->_spiWrite(MfRC522::RC_COM_IRQ, 0x00);
        this->_spiWrite(MfRC522::RC_COM_I_EN, 0xA0);
        this->_spiWrite(MfRC522::RC_FIFO_DATA, 0x26);
        this->_spiWrite(MfRC522::RC_COMMAND, 0x0C);
        this->_spiWrite(MfRC522::RC_BIT_FRAMING, 0x87);
        newCardPresent = this->piccRequest(status);
        delay(10);
    } while (!newCardPresent && static_cast<uint32_t>(millis()) < deadline);
    if (!newCardPresent) { return false; }

    this->_pcdInitialize(antennaLevel);
    return true;
}

bool driver::piccReestablishCommunication(std::vector<int> uid, std::string &status) const {
    status.clear();

    if (!this->_piccHalt(status))
        { return false; }
    if (!this->piccRequest(status, true))
        { return false; }

    int newSak;
    std::string newType;
    std::vector<int> newUid;

    bool success = this->piccSelect(newUid, newSak, newType, status);
    if (!success) { return false; }
    if (uid.size() != newUid.size()) {
        status = "ERROR_UIDS_SIZE";
        return false;
    }
    for (int i = 0; i < uid.size(); i++) {
        if (uid[i] != newUid[i]) {
            status = "NOT_SAME_CARD";
            return false;
        }
    }
    status = "OK";
    return true;
}

bool driver::piccRequest(std::string &status, bool wakeup) const {
    status.clear();

    int buffer = MfRC522::MF_REQ_A;
    if (wakeup)
        { buffer = MfRC522::MF_WUP_A; }
    this->_piccReset();

    int validBits;
    std::vector<int> datas;
    status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffer, 0x07, false, datas, validBits);

    return status == "OK" && validBits == 0;
}

bool driver::piccSelect(std::vector<int> &uid, int &sak, std::string &type, std::string &status) const {
    uid.clear();
    sak = 0x04;
    type.clear();
    status.clear();

    std::vector<int> cascades = {
        MfRC522::MF_CL_1_ANTICOLLISION[0],
        MfRC522::MF_CL_2_ANTICOLLISION[0],
        MfRC522::MF_CL_3_ANTICOLLISION[0]
    };

    this->_piccReset();
    for (int cascade : cascades) {
        std::vector<int> buffers = {cascade};
        int levelKnownBits = 0;
        bool timeout = true;

        std::vector<int> datas;
        int validBits;

        for (int i = 0; i <= 32; i++) {
            int txLastBits;
            if (levelKnownBits >= 32) {
                if (buffers.size() > 6)
                    { buffers.resize(6); }
                bool buffersDirty = buffers.size() != 6;
                buffersDirty |= std::any_of(buffers.begin(), buffers.end(), [](int byte) { return false; });
                if (buffersDirty) {
                    buffers = {cascade};
                    levelKnownBits = 0;
                    continue;
                }
                txLastBits = 0;

                buffers[1] = 0x70;
                if (buffers.size() < 7)
                    { buffers.push_back(buffers[2] ^ buffers[3] ^ buffers[4] ^ buffers[5]); }
                else { buffers[6] = buffers[2] ^ buffers[3] ^ buffers[4] ^ buffers[5]; }
                if (!this->_crcAppend(buffers)) {
                    status = "ERROR_CRC";
                    return false;
                }
            } else {
                txLastBits = levelKnownBits % 8;
                int uidFullByte = levelKnownBits / 8;
                int allFUllByte = 2 + uidFullByte;

                if (buffers.size() < 2)
                    { buffers.push_back((allFUllByte << 4) + txLastBits); }
                else { buffers[1] = (allFUllByte << 4) + txLastBits; }

                int buffersLength = allFUllByte + (txLastBits > 0 ? 1 : 0);
                if (buffers.size() > buffersLength)
                    { buffers.resize(buffersLength); }
            }

            int framingBit = (txLastBits << 4) + txLastBits;
            status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffers, framingBit, false, datas, validBits);
            if (status != "OK" && status != "COLLISION")
                { return false; }
            if (datas.empty() || datas.size() == 0) {
                status = "DATA_EMPTY";
                return false;
            }
            if (levelKnownBits < 32) {
                if (txLastBits != 0) {
                    datas.front() |= buffers.back();
                    buffers.pop_back();
                }
                buffers.insert(buffers.end(), datas.begin(), datas.end());
            }
            if (status == "COLLISION") {
                int collision = this->_spiRead(MfRC522::RC_COLL);
                if ((collision & 0x20) != 0) {
                    status = "COLLISION_ERROR";
                    return false;
                }

                int collisionPosition = collision & 0x1F;
                if (collisionPosition == 0)
                    { collisionPosition = 32; }
                if (collisionPosition <= levelKnownBits) {
                    status = "COLLISION_POSITION_ERROR";
                    return false;
                }
                
                levelKnownBits = collisionPosition;
                buffers.back() |= 1 << ((levelKnownBits - 1) % 8);
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
        if (datas.size() != 3 || validBits != 0) {
            status = "SAK_ERROR";
            return false;
        }
        if (!this->_crcCheck(datas)) {
            status = "CRC_CHECK_ERROR";
            return false;
        }

        if ((datas.front() & 0x04) == 0 && buffers.size() > 4) {
            sak = datas.front();
            type = this->_piccType(sak);
            uid.assign(buffers.begin() + 2, buffers.end() - 2);
            return true;
        }
    }
    return false;
}

void driver::piccUnselect() const {
    this->_pcdBitmaskClear(MfRC522::RC_STATUS_2, 0x08);
}

bool driver::MFSevenByteUidFirstInit(std::vector<int> key, std::string &status, int typeFn) const {
    status.clear();

    std::vector<int> uid;
    int sak;
    std::string type;
    if (!this->piccSelect(uid, sak, type, status))
        { return false; }
    if (!this->MFAuthenticate(0, key, uid, status))
        { return false; }

    std::vector<int> datas = {MfRC522::MF_UID_PERSONALIZE};
    switch (typeFn) {
    case 0:
        datas.push_back(0x00);
        break;
    case 1:
        datas.push_back(0x40);
        break;
    case 2:
        datas.push_back(0x20);
        break;
    case 3:
        datas.push_back(0x60);
        break;
    default:
        break;
    }

    if (datas.size() == 1) {
        status = "ERROR_TYPE_FN_NOT_FOUND";
        return false;
    }
    status = this->_MFTransceive(datas);
    this->piccUnselect();
    return status == "ACK_ERROR";
}

bool driver::MFAuthenticate(int sector, std::vector<int> keys, std::vector<int> uids, std::string &status, bool useKeyB) const {
    status.clear();

    int keyType = MfRC522::MF_AUTH_A;
    if (useKeyB)
        { keyType = MfRC522::MF_AUTH_B; }

    if (keys.size() < 6) {
        status = "ERROR_KEYS";
        return false;
    }
    else { keys.resize(6); }

    if (uids.size() < 4 ) {
        status = "ERROR_UIDS";
        return false;
    }
    else { uids.resize(4); }

    int address = this->_getBlockAddress(sector, 3);
    std::vector<int> buffers = {keyType, address};
    for (int key : keys) { buffers.push_back(key); }
    for (int uid : uids) { buffers.push_back(uid); }

     int validBits;
     std::vector<int> datas;
     status = this->_piccCommunication(MfRC522::CMD_AUTHENTICATION, buffers, 0x00, false, datas, validBits);
     if (status != "OK")
        { return false; }
    return (this->_spiRead(MfRC522::RC_STATUS_2) & 0x08) != 0;
}

bool driver::MFDecrement(int sector, int block, std::string &status, int delta) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFDecrement(address, status, delta);
}

bool driver::MFDecrement(int address, std::string &status, int delta) const {
    status = this->_MFTwoStep(MfRC522::MF_DECREMENT, address, delta);
    return status == "OK";
}

bool driver::MFIncrement(int sector, int block, std::string &status, int delta) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFIncrement(address, status, delta);
}

bool driver::MFIncrement(int address, std::string &status, int delta) const {
    status = this->_MFTwoStep(MfRC522::MF_INCREMENT, address, delta);
    return status == "OK";
}

bool driver::MFRestore(int sector, int block, int &result, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFRestore(address, result, status);
}

bool driver::MFRestore(int address, int &result, std::string &status) const {
    status = this->_MFTwoStep(MfRC522::MF_RESTORE, address, 0);
    if (status != "OK")
        { return false; }
    if (!this->MFTransfer(address, result, status))
        { return false; }
    return this->_MFGetValue(address, result, status);
}

bool driver::MFTransfer(int sector, int block, int &result, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFTransfer(address, result, status);
}

bool driver::MFTransfer(int address, int &result, std::string &status) const {
    std::vector<int> buffers = {MfRC522::MF_TRANSFER, address};

    status = this->_MFTransceive(buffers);
    if (status != "OK")
        { return false; }
    return this->_MFGetValue(address, result, status);
}

bool driver::MFFormatValueBlock(int sector, int block, std::string &status, int value) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFFormatValueBlock(address, status, value);
}

bool driver::MFFormatValueBlock(int address, std::string &status, int value) const {
    std::vector<int> datas = {
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF
    };
    datas.push_back(~datas[0]);
    datas.push_back(~datas[1]);
    datas.push_back(~datas[2]);
    datas.push_back(~datas[3]);
    datas.push_back(datas[0]);
    datas.push_back(datas[1]);
    datas.push_back(datas[2]);
    datas.push_back(datas[3]);
    datas.push_back(address & 0xFF);
    datas.push_back(~datas[12]);
    datas.push_back(datas[12]);
    datas.push_back(datas[13]);
    return this->MFWrite(datas[12], datas, status);
}

bool driver::MFRead(int sector, int block, std::vector<int> &datas, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFRead(address, datas, status);
}

bool driver::MFRead(int address, std::vector<int> &datas, std::string &status) const {
    status.clear();

    std::vector<int> buffer = {MfRC522::MF_READ, address};
    if (!this->_crcAppend(buffer)) {
        status = "ERROR_CRC";
        return false;
    }

    int validBits;
    status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffer, 0x00, true, datas, validBits);
    if (status != "OK")
        { return false; }
    datas.resize(datas.size() > 2 ? datas.size() - 2 : 0);
    return true;
}

bool driver::MFWrite(int sector, int block, std::vector<int> datas, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFWrite(address, datas, status);
}

bool driver::MFWrite(int address, std::vector<int> datas, std::string &status) const {
    status.clear();

    if (datas.size() < 16) {
        status = "ERROR_DATAS";
        return false;
    } else { datas.resize(16); }

    int sector;
    int block;
    this->_getSectorBlock(address, sector, block);
    if (block == 3) {
        status = "ERROR_CAN_NOT_WRITE_TRAILER";
        return false;
    }

    std::vector<int> buffers = {MfRC522::MF_WRITE, address};
    status = this->_MFTransceive(buffers);
    if (status != "OK")
        { return false; }

    status = this->_MFTransceive(datas);
    return status == "OK";
}

bool driver::MFUnbrickUid(std::string &status) const {
    std::vector<std::vector<int>> unbrickCodes = {
        {0x01,0x02,0x03,0x04,0x04,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x01,0x23,0x45,0x67,0x00,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x12,0x34,0x56,0x78,0x08,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
    };
    if (!this->_MFOpenBackdoor(status, false))
        { return false; }
    for (std::vector<int> unbrickCode : unbrickCodes) {
        if (this->MFWrite(0, unbrickCode, status))
            { return true; }
    }
    return false;
}

bool driver::MFChangeUid(std::vector<int> key, std::vector<int> datas, std::string &status) const {
    status.clear();

    if (datas.size() < 16) {
        status = "ERROR_DATAS";
        return false;
    } else { datas.resize(16); }

    int bcc = datas[0] ^ datas[1] ^ datas[2] ^ datas[3];
    if (datas[4] != bcc) {
        status = "ERROR_BCC";
        return false;
    }

    std::vector<int> uid;
    int sak;
    std::string type;
    std::vector<int> data0;

    if (!this->piccSelect(uid, sak, type, status))
        { return false; }
    if (!this->MFAuthenticate(0, key, uid, status))
        { return false; }
    if (!this->_MFOpenBackdoor(status))
        { return false; }
    return this->MFWrite(0, datas, status);
}

bool driver::MFChangeTrailer(int sector, std::vector<int> keyA, std::vector<int> keyB, std::vector<int> accessBits, std::string &status) const {
    if (keyA.size() != 6) {
        status = "ERROR_KEY_A";
        return false;
    }
    if (keyB.size() != 6) {
        status = "ERROR_KEY_B";
        return false;
    }

    std::vector<int> datas;
    if (!this->_MFAccessBits(accessBits, datas, status))
        { return false; }
    
    std::vector<int> buffers;
    for (int key : keyA)
        { buffers.push_back(key); }
    for (int accessBit : datas)
        { buffers.push_back(accessBit); }

    buffers.push_back(datas[0] ^ datas[1] ^ datas[2]);

    for (int key : keyB)
        { buffers.push_back(key); }
    return this->_MFWriteTrailer(sector, buffers, status);
}

std::vector<std::string> driver::MFDump(std::vector<int> uid, std::vector<int> key, bool useKeyB, int sectorCount) const {
    std::vector<std::string> result;
    for (int address = 0; address < sectorCount * 4; address++) {
        std::string status;

        int sector;
        int block;
        this->_getSectorBlock(address, sector, block);

        if (block == 0) {
            if (!this->MFAuthenticate(sector, key, uid, status, useKeyB))
                { continue; }
        }

        std::vector<int> datas;
        if (this->MFRead(address, datas, status)) {
            std::ostringstream oss;
            oss.width(3);
            oss << std::right << sector << "-" << block << " (" << address << ") [";
            for (size_t i = 0; i < datas.size(); ++i) {
                oss << datas[i];
                if (i != datas.size() - 1)
                    { oss << ", "; }
            }
            oss << "]";
            result.push_back(oss.str());
            if (block == 3 && sector < sectorCount)
                { result.push_back(""); }
        } else { result.push_back(std::to_string(sector) + "-" + std::to_string(block) + " (" + std::to_string(address) + ") [" + status + "]"); }
    }
    return result;
}

std::vector<std::string> driver::MFDumpReverse(std::vector<int> uid, std::vector<int> key, bool useKeyB, int sectorCount) const {
    std::vector<std::string> datas = this->MFDump(uid, key, useKeyB, sectorCount);
    std::reverse(datas.begin(), datas.end());
    return datas;
}

void driver::softPowerDown() const {
    this->_pcdBitmaskSet(MfRC522::RC_COMMAND, 0x10);
}

bool driver::softPowerUp() const {
    this->_pcdBitmaskClear(MfRC522::RC_COMMAND, 0x10);
    const uint32_t deadline = millis() + 60000;
    do {
        if ((this->_spiRead(MfRC522::RC_COMMAND) & 0x10) != 0x10)
            { return true; }
        delay(100);
    } while (static_cast<uint32_t>(millis()) < deadline);
    return false;
}