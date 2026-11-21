/*
 * src/gateway.cpp
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
#include <utility>

using namespace MfRC522;

std::string driver::_piccCommunication(int cmd, int buffer, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const {
    std::vector<int> buffers = {buffer};
    return this->_piccCommunication(cmd, buffers, framingBit, checkCrc, datas, validBits);
}

std::string driver::_MFTransceive(int buffer, bool acceptTimeout, bool ignoreCrc) const {
    std::vector<int> buffers = {buffer};
    return this->_MFTransceive(buffers, acceptTimeout, ignoreCrc);
}

bool driver::MFDecrement(int sector, int block, std::string &status, int delta) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFDecrement(address, status, delta);
}

bool driver::MFIncrement(int sector, int block, std::string &status, int delta) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFIncrement(address, status, delta);
}

bool driver::MFRestore(int sector, int block, int &result, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFRestore(address, result, status);
}

bool driver::MFTransfer(int sector, int block, int &result, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFTransfer(address, result, status);
}

bool driver::MFFormatValueBlock(int sector, int block, std::string &status, int value) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFFormatValueBlock(address, status, value);
}

bool driver::MFRead(int sector, int block, std::vector<int> &datas, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFRead(address, datas, status);
}

bool driver::MFWrite(int sector, int block, std::vector<int> datas, std::string &status) const {
    int address = this->_getBlockAddress(sector, block);
    return this->MFWrite(address, std::move(datas), status);
}
