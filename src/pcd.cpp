/*
 * src/pcd.cpp
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

using namespace MfRC522;

void driver::_pcdInitialize(int &antennaLevel) const {
    this->_softReset();

    this->_spiWrite(MfRC522::RC_TX_MODE,       0x00);
    this->_spiWrite(MfRC522::RC_RX_MODE,       0x00);
    this->_spiWrite(MfRC522::RC_MOD_WIDTH,     0x26);
    this->_spiWrite(MfRC522::RC_T_MOD,         0x80);
    this->_spiWrite(MfRC522::RC_T_PRESCALER,   0xA9);
    this->_spiWrite(MfRC522::RC_T_RELOAD_HIGH, 0x03);
    this->_spiWrite(MfRC522::RC_T_RELOAD_LOW,  0xE8);
    this->_spiWrite(MfRC522::RC_TX_ASK,        0x40);
    this->_spiWrite(MfRC522::RC_MODE,          0x3D);

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

void driver::pcdAntennaLevel(int &level) const {
    if (level < 0) { level = 1; }
    if (level > 7) { level = 7; }

    const int gainMask = 0x70;
    const int config   = this->_spiRead(MfRC522::RC_RFC_FG);

    this->_spiWrite(MfRC522::RC_RFC_FG, (config & ~gainMask) | (level << 4));
    level = (this->_spiRead(MfRC522::RC_RFC_FG) & gainMask) >> 4;
}

void driver::pcdAntennaEnable(bool enable) const {
    if (enable) { this->_pcdBitmaskSet(MfRC522::RC_TX_CONTROL,   0x03); }
    else        { this->_pcdBitmaskClear(MfRC522::RC_TX_CONTROL, 0x03); }
}
