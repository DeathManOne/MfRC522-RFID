/*
 * src/hack.cpp
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

bool driver::_MFOpenBackdoor(std::string &status, bool withUnselect) const {
    if (withUnselect) { this->piccUnselect(); }

    static_cast<void>(this->_piccHalt(status, false));
    status = this->_MFTransceive(MfRC522::MF_UID_PERSONALIZE, false, true);

    if (status != "OK") { return false; }

    status = this->_MFTransceive(MfRC522::MF_SET_MOD_TYPE, false, true);
    return status == "OK";
}

bool driver::MFUnbrickUid(std::string &status) const {
    std::vector<std::vector<int>> unbrickCodes = {
        {0x01,0x02,0x03,0x04,0x04,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x01,0x23,0x45,0x67,0x00,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x12,0x34,0x56,0x78,0x08,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
    };

    if (!this->_MFOpenBackdoor(status, false)) { return false; }

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

    if (!this->piccSelect(uid, sak, type, status))  { return false; }
    if (!this->MFAuthenticate(0, key, uid, status)) { return false; }
    if (!this->_MFOpenBackdoor(status))             { return false; }

    return this->MFWrite(0, datas, status);
}
