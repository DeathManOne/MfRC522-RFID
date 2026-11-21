/*
 * src/crc.cpp
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

bool driver::_crcAppend(std::vector<int> &datas) const {
    std::vector<int> crc;

    if (!this->_crcCalculate(datas, crc))
        { return false; }
    datas.insert(datas.end(), crc.begin(), crc.end());

    return true;
}

bool driver::_crcCalculate(const std::vector<int> &datas, std::vector<int> &crc) const {
    if (datas.empty() || datas.size() > 64) { return false; }

    this->_spiWrite(MfRC522::RC_COMMAND,    MfRC522::CMD_IDLE);
    this->_spiWrite(MfRC522::RC_DIV_IRQ,    0x04);
    this->_spiWrite(MfRC522::RC_FIFO_LEVEL, 0x80);
    this->_spiWrite(MfRC522::RC_FIFO_DATA,  datas);
    this->_spiWrite(MfRC522::RC_COMMAND,    MfRC522::CMD_CALCULATE_CRC);
    
    const uint32_t startedAt = millis();
    do {
		int irq = this->_spiRead(MfRC522::RC_DIV_IRQ);
		if ((irq & 0x04) != 0) {
			this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
			crc.push_back(this->_spiRead(MfRC522::RC_CRC_RESULT_LOW));
			crc.push_back(this->_spiRead(MfRC522::RC_CRC_RESULT_HIGHT));
            if (crc.size() != 2)
                { return false; }
			return true;
		}
    } while (static_cast<uint32_t>(millis() - startedAt) < 89u);

    this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE);
    return false;
}

bool driver::_crcCheck(const std::vector<int> &datas) const {
    if (datas.size() < 2) { return false; }

    std::vector<int> crc;
    std::vector<int> buffers(datas.begin(), datas.end() - 2);

    if (!this->_crcCalculate(buffers, crc))     { return false; }
    if (datas[datas.size() - 2] != crc.front()) { return false; }
    if (datas.back() != crc.back())             { return false; }
    return true;
}
