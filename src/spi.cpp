/*
 * src/spi.cpp
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
#include <esp32-hal-gpio.h>

using namespace MfRC522;

void driver::_spiTransferBegin() const {
    this->_SPI->beginTransaction(this->_SETTINGS);
    digitalWrite(this->_PIN_SDA, LOW);
}

void driver::_spiTransferEnd() const {
    digitalWrite(this->_PIN_SDA, HIGH);
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
    this->_spiTransferBegin();
    this->_SPI->transfer((address << 1) & 0x7E);
    this->_SPI->transfer(data);
    this->_spiTransferEnd();
}

void driver::_spiWrite(int address, const std::vector<int> &datas) const {
    this->_spiTransferBegin();
    this->_SPI->transfer((address << 1) & 0x7E);
    for (int data : datas)
        { this->_SPI->transfer(data); }
    this->_spiTransferEnd();
}
