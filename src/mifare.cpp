/*
 * src/mifare.cpp
 *
 * Copyright (c) 2026 DeathManOne
 * https://github.com/DeathManOne
 *
 * This file is part of the MFRC522 library.
 *
 * MFRC522 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MFRC522 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MFRC522.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include "../include/MfRC522.h"
#include <algorithm>
#include <cstdint>
#include <sstream>

using namespace MfRC522;

namespace {
    bool isOutsideByteRange(int value) {
        return value < 0 || value > 255;
    }

    void _getSectorBlock(int blockAddress, int &sector, int &block) {
        if (blockAddress < 128) {
            sector = blockAddress / 4;
            block  = blockAddress % 4;
            return;
        }

        const int offset = blockAddress - 128;
        sector           = 32 + offset / 16;
        block            = offset % 16;
    }
}

std::string driver::_MFTransceive(std::vector<int> buffers, bool acceptTimeout, bool ignoreCrc) const {
    if (!ignoreCrc) {
        if (!this->_crcAppend(buffers))
            { return "ERROR_CRC"; }
    }

    int validBits;
    std::vector<int> datas;

    std::string status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffers, 0x00, false, datas, validBits);

    if (acceptTimeout && status == "PICC_TIMEOUT") { return "OK"; }
    if (status        != "OK")                     { return status; }
    if (datas.size()  != 1)                        { return "DATA_ERROR"; }
    if (validBits     != 4)                        { return "VALID_BITS_ERROR"; }
    if (datas.front() != 0x0A)                     { return "ACK_ERROR"; }

    return "OK";
}

bool driver::_MFGetValue(int address, int &value, std::string &status) const {
    value = 0;
    status.clear();

    std::vector<int> datas;
    if (!this->MFRead(address, datas, status)) { return false; }

    for (size_t i = 0; i < 4; ++i) {
        if (datas[i] != datas[i + 8] || (datas[i] ^ datas[i + 4]) != 0xFF) {
            status = "VALUE_BLOCK_ERROR";
            return false;
        }
    }

    if (datas[12] != datas[14] || datas[13] != datas[15] || (datas[12] ^ datas[13]) != 0xFF) {
        status = "VALUE_BLOCK_ERROR";
        return false;
    }

    const uint32_t rawValue =
        static_cast<uint32_t>(datas[0])         |
        (static_cast<uint32_t>(datas[1]) << 8)  |
        (static_cast<uint32_t>(datas[2]) << 16) |
        (static_cast<uint32_t>(datas[3]) << 24);

    const int64_t signedValue = (rawValue & 0x80000000u) != 0
        ? static_cast<int64_t>(rawValue) - 0x100000000LL
        : static_cast<int64_t>(rawValue);

    value = static_cast<int>(signedValue);
    return true;
}

std::string driver::_MFTwoStep(int cmd, int address, int value) const {
    if (address < 0 || address > 255) { return "ERROR_ADDRESS"; }

    std::vector<int> buffers = {cmd, address};
    std::string status       = this->_MFTransceive(buffers);

    if (status != "OK") { return status; }

    const uint32_t rawValue = static_cast<uint32_t>(value);
    buffers = {
        static_cast<int>(rawValue         & 0xFFu),
        static_cast<int>((rawValue >> 8)  & 0xFFu),
        static_cast<int>((rawValue >> 16) & 0xFFu),
        static_cast<int>(rawValue  >> 24)
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

    int c1 = (accessBits.back() & 4) << 1 | (accessBits[2] & 4) << 0 | (accessBits[1] & 4) >> 1 | (accessBits.front() & 4) >> 2;
    int c2 = (accessBits.back() & 2) << 2 | (accessBits[2] & 2) << 1 | (accessBits[1] & 2) << 0 | (accessBits.front() & 2) >> 1;
    int c3 = (accessBits.back() & 1) << 3 | (accessBits[2] & 1) << 2 | (accessBits[1] & 1) << 1 | (accessBits.front() & 1) << 0;

    std::vector<int> buffers = {
        (~c2 & 0xF) << 4 | (~c1 & 0xF), // byte 6
        c1          << 4 | (~c3 & 0xF), // byte 7
        c3          << 4 | c2           // byte 8
    };

    if (buffers.size() != 3) {
        status = "ERROR_INTERNAL_MF_ACCESS_BITS";
        return false;
    }

    datas  = buffers;
    status = "OK";
    return true;
}

bool driver::_MFWriteTrailer(int sector, std::vector<int> datas, std::string &status) const {
    status.clear();

    if (datas.size() < 16) {
        status = "ERROR_DATAS";
        return false;
    } else { datas.resize(16); }

    const int trailerBlock = sector < 32 ? 3 : 15;
    const int address      = this->_getBlockAddress(sector, trailerBlock);

    if (address < 0) {
        status = "ERROR_ADDRESS";
        return false;
    }

    std::vector<int> buffers = {MfRC522::MF_WRITE, address};
    status                   = this->_MFTransceive(buffers);

    if (status != "OK") { return false; }

    status = this->_MFTransceive(datas);
    return status == "OK";
}

bool driver::MFSevenByteUidFirstInit(std::vector<int> key, std::string &status, int typeFn) const {
    status.clear();

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

    std::vector<int> uid;
    int sak;
    std::string type;

    if (!this->piccSelect(uid, sak, type, status))  { return false; }
    if (!this->MFAuthenticate(0, key, uid, status)) { return false; }

    status = this->_MFTransceive(datas);
    this->piccUnselect();

    return status == "OK";
}

bool driver::MFAuthenticate(int sector, std::vector<int> keys, std::vector<int> uids, std::string &status, bool useKeyB) const {
    status.clear();

    int keyType = MfRC522::MF_AUTH_A;
    if (useKeyB) { keyType = MfRC522::MF_AUTH_B; }

    if (keys.size() < 6 || std::any_of(keys.begin(), keys.begin() + 6, isOutsideByteRange)) {
        status = "ERROR_KEYS";
        return false;
    }
    keys.resize(6);

    if (uids.size() < 4 || std::any_of(uids.end() - 4, uids.end(), isOutsideByteRange)) {
        status = "ERROR_UIDS";
        return false;
    }

    const int trailerBlock = sector < 32 ? 3 : 15;
    const int address      = this->_getBlockAddress(sector, trailerBlock);

    if (address < 0) {
        status = "ERROR_ADDRESS";
        return false;
    }

    std::vector<int> buffers = {keyType, address};
    for (int key : keys)
        { buffers.push_back(key); }
    buffers.insert(buffers.end(), uids.end() - 4, uids.end());

    int validBits;
    std::vector<int> datas;

    status = this->_piccCommunication(MfRC522::CMD_AUTHENTICATION, buffers, 0x00, false, datas, validBits);
    if (status != "OK") { return false; }

    if ((this->_spiRead(MfRC522::RC_STATUS_2) & 0x08) == 0) {
        status = "AUTHENTICATION_ERROR";
        return false;
    }
    return true;
}

bool driver::MFDecrement(int address, std::string &status, int delta) const {
    status = this->_MFTwoStep(MfRC522::MF_DECREMENT, address, delta);
    return status == "OK";
}

bool driver::MFIncrement(int address, std::string &status, int delta) const {
    status = this->_MFTwoStep(MfRC522::MF_INCREMENT, address, delta);
    return status == "OK";
}

bool driver::MFRestore(int address, int &result, std::string &status) const {
    status = this->_MFTwoStep(MfRC522::MF_RESTORE, address, 0);

    if (status != "OK")
        { return false; }
    return this->MFTransfer(address, result, status);
}

bool driver::MFTransfer(int address, int &result, std::string &status) const {
    if (address < 0 || address > 255) {
        status = "ERROR_ADDRESS";
        return false;
    }

    std::vector<int> buffers = {MfRC522::MF_TRANSFER, address};
    status                   = this->_MFTransceive(buffers);

    if (status != "OK")
        { return false; }
    return this->_MFGetValue(address, result, status);
}

bool driver::MFFormatValueBlock(int address, std::string &status, int value) const {
    const uint32_t rawValue = static_cast<uint32_t>(value);
    std::vector<int> datas = {
        static_cast<int>(rawValue         & 0xFFu),
        static_cast<int>((rawValue >> 8)  & 0xFFu),
        static_cast<int>((rawValue >> 16) & 0xFFu),
        static_cast<int>(rawValue  >> 24)
    };

    datas.push_back(datas[0] ^ 0xFF);
    datas.push_back(datas[1] ^ 0xFF);
    datas.push_back(datas[2] ^ 0xFF);
    datas.push_back(datas[3] ^ 0xFF);

    datas.push_back(datas[0]);
    datas.push_back(datas[1]);
    datas.push_back(datas[2]);
    datas.push_back(datas[3]);

    datas.push_back(address   & 0xFF);
    datas.push_back(datas[12] ^ 0xFF);
    datas.push_back(datas[12]);
    datas.push_back(datas[13]);

    return this->MFWrite(address, datas, status);
}

bool driver::MFRead(int address, std::vector<int> &datas, std::string &status) const {
    status.clear();
    datas.clear();

    if (address < 0 || address > 255) {
        status = "ERROR_ADDRESS";
        return false;
    }

    std::vector<int> buffer = {MfRC522::MF_READ, address};
    if (!this->_crcAppend(buffer)) {
        status = "ERROR_CRC";
        return false;
    }

    int validBits;
    status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffer, 0x00, true, datas, validBits);

    if (status != "OK") {
        datas.clear();
        return false;
    }

    if (datas.size() != 18) {
        datas.clear();
        status = "DATA_ERROR";
        return false;
    }

    datas.resize(16);
    return true;
}

bool driver::MFWrite(int address, std::vector<int> datas, std::string &status) const {
    status.clear();
    if (address < 0 || address > 255) {
        status = "ERROR_ADDRESS";
        return false;
    }

    if (datas.size() < 16 ||
        std::any_of(datas.begin(), datas.begin() + 16, isOutsideByteRange)) {
        status = "ERROR_DATAS";
        return false;
    }
    datas.resize(16);

    int sector;
    int block;
    _getSectorBlock(address, sector, block);

    const int trailerBlock = sector < 32 ? 3 : 15;
    if (block == trailerBlock) {
        status = "ERROR_CAN_NOT_WRITE_TRAILER";
        return false;
    }

    std::vector<int> buffers = {MfRC522::MF_WRITE, address};
    status                   = this->_MFTransceive(buffers);

    if (status != "OK") { return false; }

    status = this->_MFTransceive(datas);
    return status == "OK";
}

bool driver::MFChangeTrailer(int sector, std::vector<int> keyA, std::vector<int> keyB, std::vector<int> accessBits, std::string &status) const {
    if (keyA.size() != 6 || std::any_of(keyA.begin(), keyA.end(), isOutsideByteRange)) {
        status = "ERROR_KEY_A";
        return false;
    }

    if (keyB.size() != 6 || std::any_of(keyB.begin(), keyB.end(), isOutsideByteRange)) {
        status = "ERROR_KEY_B";
        return false;
    }

    std::vector<int> datas;
    if (!this->_MFAccessBits(accessBits, datas, status)) { return false; }

    const int trailerBlock = sector < 32 ? 3 : 15;
    const int address      = this->_getBlockAddress(sector, trailerBlock);

    std::vector<int> currentTrailer;
    if (!this->MFRead(address, currentTrailer, status)) { return false; }

    std::vector<int> buffers;
    for (int key : keyA)        { buffers.push_back(key); }
    for (int accessBit : datas) { buffers.push_back(accessBit); }

    buffers.push_back(currentTrailer[9]);
    for (int key : keyB)        { buffers.push_back(key); }

    return this->_MFWriteTrailer(sector, buffers, status);
}

std::vector<std::string> driver::MFDump(std::vector<int> uid, std::vector<int> key, bool useKeyB, int sectorCount) const {
    std::vector<std::string> result;
    bool sectorAuthenticated = false;

    if (sectorCount <= 0 || sectorCount > 40) { return result; }

    const int lastSector  = sectorCount - 1;
    const int lastBlock   = lastSector < 32 ? 3 : 15;
    const int lastAddress = this->_getBlockAddress(lastSector, lastBlock);

    for (int address = 0; address <= lastAddress; address++) {
        std::string status;

        int sector;
        int block;
        _getSectorBlock(address, sector, block);

        if (block == 0) {
            sectorAuthenticated = this->MFAuthenticate(sector, key, uid, status, useKeyB);
            if (!sectorAuthenticated)
                { result.push_back(std::to_string(sector) + "-0 (" + std::to_string(address) + ") [" + status + "]"); }
        }

        if (!sectorAuthenticated) { continue; }
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

            const int trailerBlock = sector < 32 ? 3 : 15;
            if (block == trailerBlock)
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
