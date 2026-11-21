/*
 * src/soft.cpp
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
#include <cstdint>
#include <esp32-hal.h>

using namespace MfRC522;

namespace {
    constexpr int noCmdChange    = 0x07;
    constexpr int powerDownBit   = 0x10;
    constexpr int receiverOffBit = 0x20;
}

void driver::_softReset() const {
    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_SOFT_RESET);
    delay(10);
}

void driver::softPowerDown() const {
    const int receiverState = this->_spiRead(MfRC522::RC_COMMAND) & receiverOffBit;
    this->_spiWrite(MfRC522::RC_COMMAND, receiverState | powerDownBit | noCmdChange);
}

bool driver::softPowerUp() const {
    const int receiverState = this->_spiRead(MfRC522::RC_COMMAND) & receiverOffBit;
    this->_spiWrite(MfRC522::RC_COMMAND, receiverState | noCmdChange);

    const uint32_t startedAt = millis();
    do {
        if ((this->_spiRead(MfRC522::RC_COMMAND) & powerDownBit) == 0)
            { return true; }
        delay(100);
    } while (static_cast<uint32_t>(millis() - startedAt) < 60000u);

    return false;
}
