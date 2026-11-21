/*
 * include/MfRC522.h
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

#pragma once

#include <SPI.h>
#include <vector>
#include <string>

namespace MfRC522 {
    const int CMD_IDLE              = 0x00;
    const int CMD_MEM               = 0x01;
    const int CMD_RANDOM_ID         = 0x02;
    const int CMD_CALCULATE_CRC     = 0x03;
    const int CMD_TRANSMIT          = 0x04;
    const int CMD_NO_CHANGE         = 0x07;
    const int CMD_RECEIVE           = 0x08;
    const int CMD_TRANSCEIVE        = 0x0C;
    const int CMD_AUTHENTICATION    = 0x0E;
    const int CMD_SOFT_RESET        = 0x0F;

    const int MF_REQ_A              = 0x26;
    const int MF_WUP_A              = 0x52;

    const std::vector<int> MF_CL_1_ANTICOLLISION = {0x93, 0x20};
    const std::vector<int> MF_CL_1_SELECT        = {0x93, 0x70};
    const std::vector<int> MF_CL_2_ANTICOLLISION = {0x95, 0x20};
    const std::vector<int> MF_CL_2_SELECT        = {0x95, 0x70};
    const std::vector<int> MF_CL_3_ANTICOLLISION = {0x97, 0x20};
    const std::vector<int> MF_CL_3_SELECT        = {0x97, 0x70};
    const std::vector<int> MF_HALT               = {0x50, 0x00};

    const int MF_AUTH_A             = 0x60;
    const int MF_AUTH_B             = 0x61;
    const int MF_UID_PERSONALIZE    = 0x40;
    const int MF_SET_MOD_TYPE       = 0x43;
    const int MF_READ               = 0x30;
    const int MF_WRITE              = 0xA0;
    const int MF_DECREMENT          = 0xC0;
    const int MF_INCREMENT          = 0xC1;
    const int MF_RESTORE            = 0xC2;
    const int MF_TRANSFER           = 0xB0;
    const int MF_UL_WRITE           = 0xA2;

    const int RC_COMMAND            = 0x01;
    const int RC_COM_I_EN           = 0x02;
    const int RC_DIV_I_EN           = 0x03;
    const int RC_COM_IRQ            = 0x04;
    const int RC_DIV_IRQ            = 0x05;
    const int RC_ERROR              = 0x06;
    const int RC_STATUS_1           = 0x07;
    const int RC_STATUS_2           = 0x08;
    const int RC_FIFO_DATA          = 0x09;
    const int RC_FIFO_LEVEL         = 0x0A;
    const int RC_WATER_LEVEL        = 0x0B;
    const int RC_CONTROL            = 0x0C;
    const int RC_BIT_FRAMING        = 0x0D;
    const int RC_COLL               = 0x0E;

    const int RC_MODE               = 0x11;
    const int RC_TX_MODE            = 0x12;
    const int RC_RX_MODE            = 0x13;
    const int RC_TX_CONTROL         = 0x14;
    const int RC_TX_ASK             = 0x15;
    const int RC_TX_SEL             = 0x16;
    const int RC_RX_SEL             = 0x17;
    const int RC_RX_THRESHOLD       = 0x18;
    const int RC_DEMOD              = 0x19;
    const int RC_MF_TX              = 0x1C;
    const int RC_MF_RX              = 0x1D;
    const int RC_SERIAL_SPEED       = 0x1F;
    
    const int RC_CRC_RESULT_HIGHT   = 0x21;
    const int RC_CRC_RESULT_LOW     = 0x22;
    const int RC_MOD_WIDTH          = 0x24;
    const int RC_RFC_FG             = 0x26;
    const int RC_GS_N               = 0x27;
    const int RC_CW_GS_P            = 0x28;
    const int RC_MOD_GS_P           = 0x29;
    const int RC_T_MOD              = 0x2A;
    const int RC_T_PRESCALER        = 0x2B;
    const int RC_T_RELOAD_HIGH      = 0x2C;
    const int RC_T_RELOAD_LOW       = 0x2D;
    const int RC_T_COUNTER_VAL_HIGH = 0x2E;
    const int RC_T_COUNTER_VAL_LOW  = 0x2F;
    
    const int RC_TEST_SEL_1         = 0x31;
    const int RC_TEST_SEL_2         = 0x32;
    const int RC_TEST_PIN_EN        = 0x33;
    const int RC_TEST_PIN_VALUE     = 0x34;
    const int RC_TEST_BUS           = 0x35;
    const int RC_AUTO_TEST          = 0x36;
    const int RC_VERSION            = 0x37;
    const int RC_ANALOG_TEST        = 0x38;
    const int RC_TEST_DAC_1         = 0x39;
    const int RC_TEST_DAC_2         = 0x3A;
    const int RC_TEST_ADC           = 0x3B;

    class driver {
        private:
            int _PIN_SDA;
            SPIClass *_SPI;
            SPISettings _SETTINGS;

            void _spiTransferBegin() const;
            void _spiTransferEnd() const;
            int _spiRead(int address) const;
            void _spiWrite(int address, int data) const;
            void _spiWrite(int address, const std::vector<int> &datas) const;
            void _pcdInitialize(int &antennaLevel) const;
            void _pcdBitmaskSet(int address, int mask) const;
            void _pcdBitmaskClear(int address, int mask) const;
            bool _crcAppend(std::vector<int> &datas) const;
            bool _crcCalculate(const std::vector<int> &datas, std::vector<int> &crc) const;
            bool _crcCheck(const std::vector<int> &datas) const;
            std::string _piccCommunication(int cmd, int buffer, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const;
            std::string _piccCommunication(int cmd, const std::vector<int> &buffers, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const;
            bool _piccHalt(std::string &status, bool withUnselect = true) const;
            void _piccReset() const;
            std::string _piccType(int sak) const;
            std::string _MFTransceive(int buffer, bool acceptTimeout = false, bool ignoreCrc = false) const;
            std::string _MFTransceive(std::vector<int> buffers, bool acceptTimeout = false, bool ignoreCrc = false) const;
            bool _MFGetValue(int address, int &value, std::string &status) const;
            std::string _MFTwoStep(int cmd, int address, int value) const;
            bool _MFAccessBits(std::vector<int> accessBits, std::vector<int> &datas, std::string &status) const;
            bool _MFWriteTrailer(int sector, std::vector<int> datas, std::string &status) const;
            bool _MFOpenBackdoor(std::string &status, bool withUnselect = true) const;
            void _softReset() const;
            int _getBlockAddress(int sector, int block) const;
        public:
            /**
             * @brief Initialize the driver and configure the chip-select pin.
             * @param[in] pinSDA GPIO connected to the reader chip-select pin.
             * @param[in] SPI Initialized SPI bus. The caller owns it and must keep it alive for the lifetime of the driver.
             */
            driver(int pinSDA, SPIClass &SPI);

            /**
             * @brief Destroy the driver.
             */
            ~driver() = default;

            /**
             * @brief Set the reader's RF receiver gain.
             * @param[in,out] level Requested gain code (0 to 7), replaced with the code read back from the reader.
             */
            void pcdAntennaLevel(int &level) const;

            /**
             * @brief Enable or disable the reader's antenna drivers.
             * @param[in] enable Enable the antenna drivers when true. Defaults to true.
             */
            void pcdAntennaEnable(bool enable = true) const;

            /**
             * @brief Poll for an ISO/IEC 14443 Type A tag.
             * @param[in] timeout Retry window in seconds. A value of zero or less performs a single detection attempt.
             * @param[in,out] antennaLevel Requested gain code (0 to 7), replaced with the code read back from the reader.
             * @param[out] status Operation status code. TEMPERATURE_ERROR and FIFO_WRITE_ERROR stop polling immediately.
             * @return True if a tag responds with a valid ATQA.
             */
            bool piccWaitTag(int timeout, int &antennaLevel, std::string &status) const;

            /**
             * @brief Halt the tag, wake it, and verify that the reselected tag has the expected UID.
             * @param[in] uid Expected UID of the tag to reselect.
             * @param[out] status Operation status code. UNEXPECTED_HALT_RESPONSE indicates that the tag replied to HALT.
             * @return True if the tag is reselected and its UID matches.
             */
            bool piccReestablishCommunication(std::vector<int> uid, std::string &status) const;

            /**
             * @brief Send REQA or WUPA and validate the ATQA response.
             * @param[out] status Operation status code.
             * @param[in] wakeup Use WUPA when true or REQA when false. Defaults to false.
             * @return True if a valid ATQA response is received.
             */
            bool piccRequest(std::string &status, bool wakeup = false) const;

            /**
             * @brief Select a Type A tag using anticollision.
             * @param[out] uid Complete 4-, 7-, or 10-byte UID on success. Empty on failure.
             * @param[out] sak Final Select Acknowledge (SAK) byte on success.
             * @param[out] type Tag type derived from the SAK on success.
             * @param[out] status Operation status code.
             * @return True if selection completes and the SAK passes validation.
             */
            bool piccSelect(std::vector<int> &uid, int &sak, std::string &type, std::string &status) const;

            /**
             * @brief Disable Crypto1 on the reader.
             */
            void piccUnselect() const;

            /**
             * @brief Configure the UID usage mode of a compatible seven-byte MIFARE Classic tag.
             * @param[in] key Key A for sector 0.
             * @param[out] status Operation status code. ERROR_TYPE_FN_NOT_FOUND indicates an invalid UID usage mode.
             * @param[in] typeFn UID usage mode (0 to 3). Defaults to 1.
             * @return True if the card acknowledges the UID usage command.
             * @warning A compatible MIFARE Classic EV1 card locks the UID usage mode after accepting this command.
             */
            bool MFSevenByteUidFirstInit(std::vector<int> key, std::string &status, int typeFn = 1) const;

            /**
             * @brief Authenticate a MIFARE Classic sector and enable Crypto1.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] keys Sector key. The first six bytes are used.
             * @param[in] uids UID returned by piccSelect(), or four authentication bytes. The last four bytes are sent to the reader.
             * @param[out] status Operation status code.
             * @param[in] useKeyB Use Key B when true or Key A when false. Defaults to false.
             * @return True if the reader reports Crypto1 enabled.
             */
            bool MFAuthenticate(int sector, std::vector<int> keys, std::vector<int> uids, std::string &status, bool useKeyB = false) const;
            
            /**
             * @brief Subtract an amount using a formatted value block. Call MFTransfer() to store the result.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] block Block index within the sector (0 to 3, or 0 to 15 in Classic 4K sectors 32 to 39).
             * @param[out] status Operation status code.
             * @param[in] delta Signed amount to subtract. Defaults to 1.
             * @return True if the decrement exchange succeeds.
             */
            bool MFDecrement(int sector, int block, std::string &status, int delta = 1) const;

            /**
             * @brief Subtract an amount using a formatted value block. Call MFTransfer() to store the result.
             * @param[in] address Absolute block address (0 to 255, subject to the card capacity).
             * @param[out] status Operation status code.
             * @param[in] delta Signed amount to subtract. Defaults to 1.
             * @return True if the decrement exchange succeeds.
             */
            bool MFDecrement(int address, std::string &status, int delta = 1) const;

            /**
             * @brief Add an amount using a formatted value block. Call MFTransfer() to store the result.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] block Block index within the sector (0 to 3, or 0 to 15 in Classic 4K sectors 32 to 39).
             * @param[out] status Operation status code.
             * @param[in] delta Signed amount to add. Defaults to 1.
             * @return True if the increment exchange succeeds.
             */
            bool MFIncrement(int sector, int block, std::string &status, int delta = 1) const;

            /**
             * @brief Add an amount using a formatted value block. Call MFTransfer() to store the result.
             * @param[in] address Absolute block address (0 to 255, subject to the card capacity).
             * @param[out] status Operation status code.
             * @param[in] delta Signed amount to add. Defaults to 1.
             * @return True if the increment exchange succeeds.
             */
            bool MFIncrement(int address, std::string &status, int delta = 1) const;

            /**
             * @brief Restore the value from a formatted value block, transfer it back to the same block, and read it back.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] block Block index within the sector (0 to 3, or 0 to 15 in Classic 4K sectors 32 to 39).
             * @param[out] result Signed value read back from the block on success.
             * @param[out] status Operation status code.
             * @return True if the restore, transfer, and readback succeed.
             */
            bool MFRestore(int sector, int block, int &result, std::string &status) const;

            /**
             * @brief Restore the value from a formatted value block, transfer it back to the same block, and read it back.
             * @param[in] address Absolute block address (0 to 255, subject to the card capacity).
             * @param[out] result Signed value read back from the block on success.
             * @param[out] status Operation status code.
             * @return True if the restore, transfer, and readback succeed.
             */
            bool MFRestore(int address, int &result, std::string &status) const;

            /**
             * @brief Write the transfer buffer to a value block and read back the stored value.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] block Block index within the sector (0 to 3, or 0 to 15 in Classic 4K sectors 32 to 39).
             * @param[out] result Signed value read back from the block on success.
             * @param[out] status Operation status code.
             * @return True if the transfer and readback succeed.
             */
            bool MFTransfer(int sector, int block, int &result, std::string &status) const;

            /**
             * @brief Write the transfer buffer to a value block and read back the stored value.
             * @param[in] address Absolute block address (0 to 255, subject to the card capacity).
             * @param[out] result Signed value read back from the block on success.
             * @param[out] status Operation status code.
             * @return True if the transfer and readback succeed.
             */
            bool MFTransfer(int address, int &result, std::string &status) const;

            /**
             * @brief Overwrite a data block with a formatted MIFARE Classic value block.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] block Block index within the sector (0 to 3, or 0 to 15 in Classic 4K sectors 32 to 39).
             * @param[out] status Operation status code.
             * @param[in] value Initial signed value stored in the block. Defaults to 0.
             * @return True if the formatted block is written.
             */
            bool MFFormatValueBlock(int sector, int block, std::string &status, int value = 0) const;

            /**
             * @brief Overwrite a data block with a formatted MIFARE Classic value block.
             * @param[in] address Absolute block address (0 to 255, subject to the card capacity).
             * @param[out] status Operation status code.
             * @param[in] value Initial signed value stored in the block. Defaults to 0.
             * @return True if the formatted block is written.
             */
            bool MFFormatValueBlock(int address, std::string &status, int value = 0) const;

            /**
             * @brief Read and validate a 16-byte MIFARE Classic block.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] block Block index within the sector (0 to 3, or 0 to 15 in Classic 4K sectors 32 to 39).
             * @param[out] datas The 16 data bytes of the block on success. Empty on failure.
             * @param[out] status Operation status code.
             * @return True if a complete block is received and its CRC is valid.
             */
            bool MFRead(int sector, int block, std::vector<int> &datas, std::string &status) const;

            /**
             * @brief Read and validate a 16-byte MIFARE Classic block.
             * @param[in] address Absolute block address (0 to 255, subject to the card capacity).
             * @param[out] datas The 16 data bytes of the block on success. Empty on failure.
             * @param[out] status Operation status code.
             * @return True if a complete block is received and its CRC is valid.
             */
            bool MFRead(int address, std::vector<int> &datas, std::string &status) const;

            /**
             * @brief Write a MIFARE Classic data block.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] block Block index within the sector (0 to 3, or 0 to 15 in Classic 4K sectors 32 to 39).
             * @param[in] datas Data to write. Provide at least 16 elements. The first 16 must be in the range 0 to 255 and are written to the card.
             * @param[out] status Operation status code.
             * @return True if the card acknowledges the write.
             */
            bool MFWrite(int sector, int block, std::vector<int> datas, std::string &status) const;

            /**
             * @brief Write a MIFARE Classic data block.
             * @param[in] address Absolute block address (0 to 255, subject to the card capacity).
             * @param[in] datas Data to write. Provide at least 16 elements. The first 16 must be in the range 0 to 255 and are written to the card.
             * @param[out] status Operation status code.
             * @return True if the card acknowledges the write.
             */
            bool MFWrite(int address, std::vector<int> datas, std::string &status) const;

            /**
             * @brief Attempt to recover a compatible rewritable tag.
             * @param[out] status Operation status code.
             * @return True if the recovery write succeeds.
             */
            bool MFUnbrickUid(std::string &status) const;

            /**
             * @brief Change the UID on a compatible rewritable tag.
             * @param[in] key Key A for sector 0.
             * @param[in] datas Replacement manufacturer-block data.
             * @param[out] status Operation status code.
             * @return True if the manufacturer-block write succeeds.
             */
            bool MFChangeUid(std::vector<int> key, std::vector<int> datas, std::string &status) const;

            /**
             * @brief Write the keys and access conditions while preserving the trailer's user byte.
             * @pre Authenticate the sector with a key that permits reading and writing its trailer.
             * @param[in] sector Sector index. The valid range depends on the card capacity.
             * @param[in] keyA Six-byte Key A.
             * @param[in] keyB Six-byte Key B.
             * @param[in] accessBits Four access-condition codes (0 to 7), in order for data groups 0 to 2 and the sector trailer.
             * @param[out] status Operation status code.
             * @return True if the card acknowledges the trailer write.
             */
            bool MFChangeTrailer(int sector, std::vector<int> keyA, std::vector<int> keyB, std::vector<int> accessBits, std::string &status) const;

            /**
             * @brief Read and format the requested MIFARE Classic sectors in block order.
             * @param[in] uid Tag UID or four authentication bytes, as accepted by MFAuthenticate().
             * @param[in] key Sector key to use for each authentication.
             * @param[in] useKeyB Use Key B when true or Key A when false. Defaults to false.
             * @param[in] sectorCount Number of sectors to read from sector 0. Valid range is 1 to 40. Defaults to 16.
             * @return Formatted lines in ascending block order. An authentication failure produces one error line and skips the sector. An invalid sector count returns an empty vector.
             */
            std::vector<std::string> MFDump(std::vector<int> uid, std::vector<int> key, bool useKeyB = false, int sectorCount = 16) const;

            /**
             * @brief Read and format the requested MIFARE Classic sectors in reverse block order.
             * @param[in] uid Tag UID or four authentication bytes, as accepted by MFAuthenticate().
             * @param[in] key Sector key to use for each authentication.
             * @param[in] useKeyB Use Key B when true or Key A when false. Defaults to false.
             * @param[in] sectorCount Number of sectors to read from sector 0. Valid range is 1 to 40. Defaults to 16.
             * @return The lines returned by MFDump() in reverse order.
             */
            std::vector<std::string> MFDumpReverse(std::vector<int> uid, std::vector<int> key, bool useKeyB = false, int sectorCount = 16) const;

            /**
             * @brief Put the MFRC522 into soft power-down mode.
             */
            void softPowerDown() const;

            /**
             * @brief Wake the MFRC522 from soft power-down mode.
             * @return True if the reader exits soft power-down before the timeout.
             */
            bool softPowerUp() const;
            
    };
}
