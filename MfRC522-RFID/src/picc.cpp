/*
 * src/picc.cpp
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
#include <esp_timer.h>

using namespace MfRC522;

#ifdef MFRC522_TEST_TRACE_COLLISION
void mfrc522TestRecordCollision(int position);
#endif

std::string driver::_piccCommunication(int cmd, const std::vector<int> &buffers, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const {
    datas.clear();
    validBits = 0;
    if (buffers.size() > 64) { return "BUFFER_OVERFLOW"; }

    int irqWait = 0x00;

    if (cmd == MfRC522::CMD_AUTHENTICATION)  { irqWait = 0x10; }
    else if (cmd == MfRC522::CMD_TRANSCEIVE) { irqWait = 0x30; }

    this->_spiWrite(MfRC522::RC_COMMAND,     MfRC522::CMD_IDLE);
    this->_spiWrite(MfRC522::RC_COM_IRQ,     0x7F);
    this->_spiWrite(MfRC522::RC_FIFO_LEVEL,  0x80);
    this->_spiWrite(MfRC522::RC_BIT_FRAMING, framingBit);
    this->_spiWrite(MfRC522::RC_FIFO_DATA,   buffers);
    this->_spiWrite(MfRC522::RC_COMMAND,     cmd);

    if (cmd == MfRC522::CMD_TRANSCEIVE)
        { this->_pcdBitmaskSet(MfRC522::RC_BIT_FRAMING, 0x80); }

    bool completed           = false;
    bool timerExpired        = false;
    const uint32_t startedAt = millis();

    do {
        int irq = this->_spiRead(MfRC522::RC_COM_IRQ);
        if ((irq & irqWait) != 0) {
            completed = true;
            break;
        }
        if ((irq & 0x01) != 0) {
            timerExpired = true;
            break;
        }
    } while (static_cast<uint32_t>(millis() - startedAt) < 36u);

    const int error = this->_spiRead(MfRC522::RC_ERROR);

    if (!completed) { this->_spiWrite(MfRC522::RC_COMMAND, MfRC522::CMD_IDLE); }
    this->_pcdBitmaskClear(MfRC522::RC_BIT_FRAMING, 0x80);

    if ((error & 0x40) != 0) { return "TEMPERATURE_ERROR"; }
    if ((error & 0x80) != 0) { return "FIFO_WRITE_ERROR"; }
    if (!completed)          { return timerExpired ? "PICC_TIMEOUT" : "SOFT_TIMEOUT"; }

    if ((error & 0x01) != 0) { return "PROTOCOL_ERROR"; }
    if ((error & 0x02) != 0) { return "PARITY_ERROR"; }
    if ((error & 0x04) != 0) { return "CRC_ERROR"; }
    if ((error & 0x10) != 0) { return "BUFFER_OVERFLOW"; }

    const int datasLength = this->_spiRead(MfRC522::RC_FIFO_LEVEL) & 0x7F;
    if (datasLength > 64) { return "FIFO_LEVEL_ERROR"; }

    for (int i = 0; i < datasLength; ++i)
        { datas.push_back(this->_spiRead(MfRC522::RC_FIFO_DATA)); }

    validBits = this->_spiRead(MfRC522::RC_CONTROL) & 0x07;
    if ((error & 0x08) != 0) { return "COLLISION"; }

    if (checkCrc) {
        if (datas.size() == 1 && validBits == 4) { return "MF_NACK"; }
        if (datas.size() < 2  || validBits != 0) { return "CRC_ERROR"; }
        if (!this->_crcCheck(datas))             { return "CRC_CHECK_ERROR"; }
    }
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

    if (withUnselect)   { this->piccUnselect(); }
    if (status == "OK") { status = "UNEXPECTED_HALT_RESPONSE"; }

    return status == "PICC_TIMEOUT";
}

void driver::_piccReset() const {
    this->_spiWrite(MfRC522::RC_COMMAND,     MfRC522::CMD_IDLE);
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

bool driver::piccWaitTag(int timeout, int &antennaLevel, std::string &status) const {
    status.clear();

    bool newCardPresent     = false;
    const int64_t startedAt = esp_timer_get_time();
    const int64_t timeoutUs = timeout > 0 ? static_cast<int64_t>(timeout) * 1000000LL : 0;

    do {
        this->_pcdInitialize(antennaLevel);
        newCardPresent = this->piccRequest(status);
        if (status == "TEMPERATURE_ERROR" || status == "FIFO_WRITE_ERROR")
            { return false; }
        delay(10);
    } while (!newCardPresent && esp_timer_get_time() - startedAt < timeoutUs);

    if (!newCardPresent)
        { return false; }
    this->_pcdInitialize(antennaLevel);
    return true;
}

bool driver::piccReestablishCommunication(std::vector<int> uid, std::string &status) const {
    status.clear();

    if (!this->_piccHalt(status))         { return false; }
    if (!this->piccRequest(status, true)) { return false; }

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
    if (wakeup) { buffer = MfRC522::MF_WUP_A; }
    this->_piccReset();

    int validBits;
    std::vector<int> datas;

    status = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffer, 0x07, false, datas, validBits);
    if (status != "OK") { return false; }

    if (datas.size() != 2 || validBits != 0) {
        status = "ATQA_ERROR";
        return false;
    }
    return true;
}

bool driver::piccSelect(std::vector<int> &uid, int &sak, std::string &type, std::string &status) const {
    uid.clear();
    sak = 0x04;
    type.clear();
    status.clear();

    std::vector<int> cascades = {MfRC522::MF_CL_1_ANTICOLLISION[0], MfRC522::MF_CL_2_ANTICOLLISION[0], MfRC522::MF_CL_3_ANTICOLLISION[0]};
    this->_piccReset();
    std::vector<int> selectedUid;

    for (int cascade : cascades) {
        std::vector<int> buffers = {cascade};
        int levelKnownBits       = 0;
        bool timeout             = true;

        std::vector<int> datas;
        int validBits;

        for (int i = 0; i <= 32; i++) {
            int txLastBits;
            if (levelKnownBits >= 32) {
                if (buffers.size() != 6) {
                    status = "ANTICOLLISION_LENGTH_ERROR";
                    return false;
                }

                txLastBits = 0;
                buffers[1] = 0x70;
                buffers.push_back(buffers[2] ^ buffers[3] ^ buffers[4] ^ buffers[5]);

                if (!this->_crcAppend(buffers)) {
                    status = "ERROR_CRC";
                    return false;
                }
            } else {
                txLastBits      = levelKnownBits % 8;
                int uidFullByte = levelKnownBits / 8;
                int allFUllByte = 2 + uidFullByte;

                if (buffers.size() < 2) { buffers.push_back((allFUllByte << 4) + txLastBits); }
                else                    { buffers[1] = (allFUllByte << 4) + txLastBits; }

                int buffersLength = allFUllByte + (txLastBits > 0 ? 1 : 0);
                if (buffers.size() > buffersLength)
                    { buffers.resize(buffersLength); }
            }

            int framingBit = (txLastBits << 4) + txLastBits;
            status         = this->_piccCommunication(MfRC522::CMD_TRANSCEIVE, buffers, framingBit, false, datas, validBits);
            if (status != "OK" && status != "COLLISION") { return false; }

            if (datas.empty() && status != "COLLISION") {
                status = "DATA_EMPTY";
                return false;
            }

            if (levelKnownBits < 32 && !datas.empty()) {
                if (txLastBits != 0) {
                    const int knownMask = (1 << txLastBits) - 1;
                    datas.front()       = (datas.front() & ~knownMask) | (buffers.back() & knownMask);
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
                if (collisionPosition == 0) { collisionPosition = 32; }

                if (collisionPosition <= levelKnownBits) {
                    status = "COLLISION_POSITION_ERROR";
                    return false;
                }
                
                const int bitsBeforeCollision     = collisionPosition - 1;
                const size_t collisionByte        = 2 + bitsBeforeCollision / 8;
                const int collisionBit            = 1 << (bitsBeforeCollision % 8);
                const size_t bytesBeforeCollision = 2 + (bitsBeforeCollision + 7) / 8;

                if (buffers.size() < bytesBeforeCollision) {
                    status = "COLLISION_DATA_ERROR";
                    return false;
                }

                buffers.resize(collisionByte + 1, 0);
                buffers[collisionByte] = (buffers[collisionByte] & (collisionBit - 1)) | collisionBit;
                levelKnownBits         = collisionPosition;
#ifdef MFRC522_TEST_TRACE_COLLISION
                mfrc522TestRecordCollision(collisionPosition);
#endif
            } else {
                if (levelKnownBits >= 32) {
                    timeout = false;
                    break;
                }

                if (buffers.size() != 7) {
                    status = "ANTICOLLISION_LENGTH_ERROR";
                    return false;
                }

                const int bcc = buffers[2] ^ buffers[3] ^ buffers[4] ^ buffers[5];
                if (buffers[6] != bcc) {
                    status = "BCC_ERROR";
                    return false;
                }

                buffers.pop_back();
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

        const bool hasNextCascade = (datas.front() & 0x04) != 0;
        if (hasNextCascade) {
            if (buffers[2] != 0x88) {
                status = "CASCADE_TAG_ERROR";
                return false;
            }
            selectedUid.insert(selectedUid.end(), buffers.begin() + 3, buffers.begin() + 6);
            continue;
        }

        selectedUid.insert(selectedUid.end(), buffers.begin() + 2, buffers.begin() + 6);
        sak  = datas.front();
        type = this->_piccType(sak);
        uid.swap(selectedUid);
        return true;
    }
    status = "UID_INCOMPLETE";
    return false;
}

void driver::piccUnselect() const {
    this->_pcdBitmaskClear(MfRC522::RC_STATUS_2, 0x08);
}
