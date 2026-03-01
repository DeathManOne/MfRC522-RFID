#ifndef _MFRC522_
#define _MFRC522_

#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <sstream>

namespace MfRC522 {
    const int CMD_IDLE = 0x00;
    const int CMD_MEM = 0x01;
    const int CMD_RANDOM_ID = 0x02;
    const int CMD_CALCULATE_CRC = 0x03;
    const int CMD_TRANSMIT = 0x04;
    const int CMD_NO_CHANGE = 0x07;
    const int CMD_RECEIVE = 0x08;
    const int CMD_TRANSCEIVE = 0x0C;
    const int CMD_AUTHENTICATION = 0x0E;
    const int CMD_SOFT_RESET = 0x0F;

    const int MF_REQ_A = 0x26;
    const int MF_WUP_A = 0x52;
    const std::vector<int> MF_CL_1_ANTICOLLISION = {0x93, 0x20};
    const std::vector<int> MF_CL_1_SELECT = {0x93, 0x70};
    const std::vector<int> MF_CL_2_ANTICOLLISION = {0x95, 0x20};
    const std::vector<int> MF_CL_2_SELECT = {0x95, 0x70};
    const std::vector<int> MF_CL_3_ANTICOLLISION = {0x97, 0x20};
    const std::vector<int> MF_CL_3_SELECT = {0x97, 0x70};
    const std::vector<int> MF_HALT = {0x50, 0x00};
    const int MF_AUTH_A = 0x60;
    const int MF_AUTH_B = 0x61;
    const int MF_UID_PERSONALIZE = 0x40;
    const int MF_SET_MOD_TYPE = 0x43;
    const int MF_READ = 0x30;
    const int MF_WRITE = 0xA0;
    const int MF_DECREMENT = 0xC0;
    const int MF_INCREMENT = 0xC1;
    const int MF_RESTORE = 0xC2;
    const int MF_TRANSFER = 0xB0;
    const int MF_UL_WRITE = 0xA2;

    const int RC_COMMAND = 0x01;
    const int RC_COM_I_EN = 0x02;
    const int RC_DIV_I_EN = 0x03;
    const int RC_COM_IRQ = 0x04;
    const int RC_DIV_IRQ = 0x05;
    const int RC_ERROR = 0x06;
    const int RC_STATUS_1 = 0x07;
    const int RC_STATUS_2 = 0x08;
    const int RC_FIFO_DATA = 0x09;
    const int RC_FIFO_LEVEL = 0x0A;
    const int RC_WATER_LEVEL = 0x0B;
    const int RC_CONTROL = 0x0C;
    const int RC_BIT_FRAMING = 0x0D;
    const int RC_COLL = 0x0E;

    const int RC_MODE = 0x11;
    const int RC_TX_MODE = 0x12;
    const int RC_RX_MODE = 0x13;
    const int RC_TX_CONTROL = 0x14;
    const int RC_TX_ASK = 0x15;
    const int RC_TX_SEL = 0x16;
    const int RC_RX_SEL = 0x17;
    const int RC_RX_THRESHOLD = 0x18;
    const int RC_DEMOD = 0x19;
    const int RC_MF_TX = 0x1C;
    const int RC_MF_RX = 0x1D;
    const int RC_SERIAL_SPEED = 0x1F;
    
    const int RC_CRC_RESULT_HIGHT = 0x21;
    const int RC_CRC_RESULT_LOW = 0x22;
    const int RC_MOD_WIDTH = 0x24;
    const int RC_RFC_FG = 0x26;
    const int RC_GS_N = 0x27;
    const int RC_CW_GS_P = 0x28;
    const int RC_MOD_GS_P = 0x29;
    const int RC_T_MOD = 0x2A;
    const int RC_T_PRESCALER = 0x2B;
    const int RC_T_RELOAD_HIGH = 0x2C;
    const int RC_T_RELOAD_LOW = 0x2D;
    const int RC_T_COUNTER_VAL_HIGH = 0x2E;
    const int RC_T_COUNTER_VAL_LOW = 0x2F;
    
    const int RC_TEST_SEL_1 = 0x31;
    const int RC_TEST_SEL_2 = 0x32;
    const int RC_TEST_PIN_EN = 0x33;
    const int RC_TEST_PIN_VALUE = 0x34;
    const int RC_TEST_BUS = 0x35;
    const int RC_AUTO_TEST = 0x36;
    const int RC_VERSION = 0x37;
    const int RC_ANALOG_TEST = 0x38;
    const int RC_TEST_DAC_1 = 0x39;
    const int RC_TEST_DAC_2 = 0x3A;
    const int RC_TEST_ADC = 0x3B;

    class driver {
        private:
            int *_PIN_SDA;
            SPIClass *_SPI;
            SPISettings *_SETTINGS;

            void _spiTransferBegin() const;
            void _spiTransferEnd() const;
            int _spiRead(int address) const;
            void _spiWrite(int address, int data) const;
            void _spiWrite(int address, std::vector<int> datas) const;
            void _pcdInitialize(int &antennaLevel) const;
            void _pcdBitmaskSet(int address, int mask) const;
            void _pcdBitmaskClear(int address, int mask) const;
            bool _crcAppend(int &data) const;
            bool _crcAppend(std::vector<int> &datas) const;
            bool _crcCalculate(int data, std::vector<int> &crc) const;
            bool _crcCalculate(std::vector<int> datas, std::vector<int> &crc) const;
            bool _crcCheck(std::vector<int> datas) const;
            std::string _piccCommunication(int cmd, int buffer, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const;
            std::string _piccCommunication(int cmd, std::vector<int> buffers, int framingBit, bool checkCrc, std::vector<int> &datas, int &validBits) const;
            bool _piccHalt(std::string &status) const;
            void _piccReset() const;
            std::string _piccType(int sak) const;
            std::string _MFTransceive(int buffer, bool acceptTimeout = false) const;
            std::string _MFTransceive(std::vector<int> buffers, bool acceptTimeout = false) const;
            bool _MFGetValue(int address, int &value, std::string &status) const;
            std::string _MFTwoStep(int cmd, int address, int value) const;
            bool _MFAccessBits(std::vector<int> accessBits, std::vector<int> &datas, std::string &status) const;
            bool _MFWriteTrailer(int sector, std::vector<int> datas, std::string &status) const;
            /* /!\ NOT TESTED /!\ */
            std::string _MFOpenBackdoor(bool maxTryPerUnbrickCode) const;
            void _softReset() const;
            int _getBlockAddress(int sector, int block) const;
            void _getSectorBlock(int blockAddress, int &sector, int &block) const;
        public:
            /**
             * @brief constructor
             * @param pinSDA for chip select (CS/SS/SDA)
             * @param SPI class of SPI for HSPI/VSPI
             */
            driver(int pinSDA, SPIClass &SPI);

            /**
             * @brief deconstructor
             */
            ~driver();

            /**
             * @brief set antenna gain
             * @param level [in/out] set level and return/change value with current level used
             */
            void pcdAntennaLevel(int &level) const;

            /**
             * @brief enable or disable the antenna
             * @param enable (default: true) 
             */
            void pcdAntennaEnable(bool enable = true) const;

            /**
             * @brief wait a tag to be close to the receiver
             * @param timeout max of time to wait an approach of a tag
             * @param antennaLevel [in/out] set level and return/change value with current level used
             * @param status [out] log
             * @return true if a tag is close, otherwise false
             */
            bool piccWaitTag(int timeout, int &antennaLevel, std::string &status) const;

            /**
             * @brief use it after an unselect
             * @param uid id of the card
             * @param status [out] log
             * @return true if the connection is reestablished, otherwise false
             */
            bool piccReestablishCommunication(std::vector<int> uid, std::string &status) const;

            /**
             * @brief request after before a select
             * @param status [out] log
             * @param wakeup (default: false) true for a reestablished, false for a select
             * @return true if success, otherwise false
             */
            bool piccRequest(std::string &status, bool wakeup = false) const;

            /**
             * @brief select your tag close to the receiver
             * @param uid [out] id of the tag
             * @param sak [out] sak of the tag
             * @param type [out] type of the tag
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool piccSelect(std::vector<int> &uid, int &sak, std::string &type, std::string &status) const;

            /**
             * @brief unselect the picc
             */
            void piccUnselect() const;

            /**
             * @brief /!\ NOT TESTED /!\ first ignition for tag with seven byte uid
             * @param status [out] log
             * @param typeFn (default: 1) FN type of the tag - datasheet - default option is 1 - between 0 and 3
             * @return true if success, otherwise false
             */
            bool MFSevenByteUidFirstInit(std::string &status, int typeFn = 1) const;

            /**
             * @brief authenticate the sector you want to use
             * @param sector number of the sector
             * @param keys password of the sector
             * @param uids id of the tag
             * @param status [out] log
             * @param useKeyB (default: false) true for keyA | false for keyB
             * @return true if success, otherwise false
             */
            bool MFAuthenticate(int sector, std::vector<int> keys, std::vector<int> uids, std::string &status, bool useKeyB = false) const;
            
            /**
             * @brief decrement the value from a sector/block - have to format (MFFormatValueBlock) before and transfer (MFTransfer) after
             * @param sector number of the sector
             * @param block number of the block in sector
             * @param status [out] log
             * @param delta (default: 1) decrement the value with this delta
             * @return true if success, otherwise false
             */
            bool MFDecrement(int sector, int block, std::string &status, int delta = 1) const;

            /**
             * @brief decrement the value from an address - have to format (MFFormatValueBlock) before and transfer (MFTransfer) after
             * @param address number of the address
             * @param status [out] log
             * @param delta (default: 1) decrement the value with this delta
             * @return true if success, otherwise false
             */
            bool MFDecrement(int address, std::string &status, int delta = 1) const;

            /**
             * @brief increment the value from a sector/block - have to format (MFFormatValueBlock) before and transfer (MFTransfer) after
             * @param sector number of the sector
             * @param block number of the block in sector
             * @param status [out] log
             * @param delta (default: 1) increment the value with this delta
             * @return true if success, otherwise false
             */
            bool MFIncrement(int sector, int block, std::string &status, int delta = 1) const;

            /**
             * @brief increment the value from a address - have to format (MFFormatValueBlock) before and transfer (MFTransfer) after
             * @param address number of the address
             * @param status [out] log
             * @param delta (default: 1) increment the value with this delta
             * @return true if success, otherwise false
             */
            bool MFIncrement(int address, std::string &status, int delta = 1) const;

            /**
             * @brief retrieve the value from a sector/block - have to format (MFFormatValueBlock) before
             * @param sector number of the sector
             * @param block number of the block in sector
             * @param result [out] value stored in sector/block
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFRestore(int sector, int block, int &result, std::string &status) const;

            /**
             * @brief retrieve the value from a address - have to format (MFFormatValueBlock) before
             * @param address number of the address
             * @param result [out] value stored in sector/block
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFRestore(int address, int &result, std::string &status) const;

            /**
             * @brief transfer a value you want after increment (MFIncrement) or decrement (MFDecrement)
             * @param sector number of the sector
             * @param block number of the block in sector
             * @param result [out] value stored in sector/block
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFTransfer(int sector, int block, int &result, std::string &status) const;

            /**
             * @brief transfer a value you want after increment (MFIncrement) or decrement (MFDecrement)
             * @param address number of the address
             * @param result [out] value stored in sector/block
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFTransfer(int address, int &result, std::string &status) const;

            /**
             * @brief format a sector/block to be a numeric data before increment (MFIncrement), decrement (MFDecrement) and restore (MFRestore)
             * @param sector number of the sector
             * @param block number of the block in sector
             * @param status [out] log
             * @param value (default: 0) ignition value
             * @return true if success, otherwise false
             */
            bool MFFormatValueBlock(int sector, int block, std::string &status, int value = 0) const;

            /**
             * @brief format a address to be a numeric data before increment (MFIncrement), decrement (MFDecrement) and restore (MFRestore)
             * @param address number of the address
             * @param status [out] log
             * @param value (default: 0) ignition value
             * @return true if success, otherwise false
             */
            bool MFFormatValueBlock(int address, std::string &status, int value = 0) const;

            /**
             * @brief read datas from sector/block
             * @param sector number of the sector
             * @param block number of the block in sector
             * @param datas [out] datas stored in sector/block
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFRead(int sector, int block, std::vector<int> &datas, std::string &status) const;

            /**
             * @brief read datas from address
             * @param address number of the address
             * @param datas [out] datas stored in address
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFRead(int address, std::vector<int> &datas, std::string &status) const;

            /**
             * @brief write datas for sector/block
             * @param sector number of the sector
             * @param block number of the block in sector
             * @param datas write datas in sector/block
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFWrite(int sector, int block, std::vector<int> datas, std::string &status) const;

            /**
             * @brief write datas for the address
             * @param address number of the address
             * @param datas write datas in address
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFWrite(int address, std::vector<int> datas, std::string &status) const;

            /**
             * @brief /!\ NOT TESTED /!\ change the uid of the tag
             * @param maxTryPerUnbrickCode max try per code known
             * @param data all data to store in uid address
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFChangeUid(int maxTryPerUnbrickCode, std::vector<int> datas, std::string &status) const;

            /**
             * @brief change the trailer of the sector
             * @param sector number of the sector to change
             * @param keyA list of keyA
             * @param keyB list of keyB
             * @param accessBits format values for block 0, 1, 2, 3 excepted for uid address
             * @param status [out] log
             * @return true if success, otherwise false
             */
            bool MFChangeTrailer(int sector, std::vector<int> keyA, std::vector<int> keyB, std::vector<int> accessBits, std::string &status) const;

            /**
             * @brief get all datas of the tag
             * @param uid id of the tag
             * @param key list of keyA or keyB
             * @param useKeyB (default: false) false for keyA | true for keyB
             * @param sectorCount (default: 16) number of count sector (usually 16)
             * @return list of string
             */
            std::vector<std::string> MFDump(std::vector<int> uid, std::vector<int> key, bool useKeyB = false, int sectorCount = 16) const;

            /**
             * @brief shut down the UC
             */
            void softPowerDown() const;

            /**
             * @brief power up the UC
             * @return true if success, otherwise false
             */
            bool softPowerUp() const;
            
    };
}
#endif