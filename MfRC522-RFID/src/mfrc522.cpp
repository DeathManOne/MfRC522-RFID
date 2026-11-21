/*
 * src/mfrc522.cpp
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
#include <esp32-hal-gpio.h>

#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
// Provided by Arduino's board variant. The GPIO HAL expects a raw GPIO number.
std::int8_t digitalPinToGPIONumber(std::int8_t pin);
#endif

using namespace MfRC522;

namespace {
    int chipSelectGpio(int pin) {
#if defined(BOARD_HAS_PIN_REMAP) && !defined(BOARD_USES_HW_GPIO_NUMBERS)
        if (pin < 0 || pin > INT8_MAX) { return -1; }
        return digitalPinToGPIONumber(static_cast<std::int8_t>(pin));
#else
        return pin;
#endif
    }
}

driver::driver(int pinSDA, SPIClass &SPI) : _PIN_SDA(chipSelectGpio(pinSDA)), _SPI(&SPI), _SETTINGS(4000000u, SPI_MSBFIRST, SPI_MODE0) {
    pinMode(this->_PIN_SDA, OUTPUT);
    digitalWrite(this->_PIN_SDA, HIGH);
}

int driver::_getBlockAddress(int sector, int block) const {
    if (sector < 0  || sector >= 40)            { return -1; }

    const int blocksInSector = sector < 32 ? 4 : 16;
    if (block  < 0  || block >= blocksInSector) { return -1; }
    if (sector < 32)                            { return sector * 4 + block; }

    return 128 + (sector - 32) * 16 + block;
}
