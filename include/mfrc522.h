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
            void _spiWrite(int address, std::vector<int> data) const;
            int _pcdInitialize(int antennaLevel) const;
            void _pcdBitmaskSet(int address, int mask) const;
            void _pcdBitmaskClear(int address, int mask) const;
            bool _crcAppend(std::vector<int> &data) const;
            bool _crcCalculate(std::vector<int> data, std::vector<int> &crc) const;
            bool _crcCheck(std::vector<int> data) const;
            std::string _piccCommunication(int cmd, std::vector<int> data, int framingBit, bool checkCrc, std::vector<int> &newData, int &validBits) const;
            bool _piccHalt() const;
            void _piccReset() const;
            std::string _piccType(int sak) const;
            std::string _MFTransceive(int data, bool acceptTimeout) const;
            std::string _MFTransceive(std::vector<int> data, bool acceptTimeout) const;
            bool _MFGetValue(int blockAddress, int &value) const;
            std::string _MFTwoStep(int cmd, int blockAddress, int value) const;
            bool _MFAccessBits(std::vector<int> accessbits, std::vector<int> &data) const;
            std::string _MFOpenBackdoor(bool maxTryPerUnbrickCode) const;
            void _softReset() const;
        public:
            driver(int pinSDA, SPIClass &SPI);
            ~driver();
            int pcdAntennaLevel(int level) const;
            void pcdAntennaEnable(bool enable) const;
            bool piccWaitTag(int &antennaLevel, int timeout) const;
            bool piccReestablishCommunication(std::vector<int> uid, std::string &status) const;
            bool piccRequest(bool wakeup) const;
            bool piccSelect(std::vector<int> &uid, int &sak, std::string &type, std::string &status) const;
            bool MFAuthenticate(bool useKeyB, int blockAddress, std::vector<int> key, std::vector<int> uid) const;
            void MFDeauthenticate() const;
            bool MFSevenByteUidFirstInit(int typeFn) const;
            bool MFDecrement(int blockAddress, int delta) const;
            bool MFIncrement(int blockAddress, int delta) const;
            bool MFRestore(int blockAddress, int &result) const;
            bool MFTransfer(int blockAddress, int &result) const;
            bool MFFormatValueBlock(int blockAddress, int value) const;
            bool MFRead(int blockAddress, std::vector<int> &data) const;
            bool MFWrite(int blockAddress, std::vector<int> data) const;
            bool MFChangeUid(int maxTryPerUnbrickCode, std::vector<int> data) const;
            bool MFChangeTrailer(int sector, std::vector<int> keyA, std::vector<int> keyB, std::vector<int> accessBits) const;
            std::vector<std::string> MFDump(bool useKeyB, std::vector<int> key, int sectorCount, std::vector<int> uid) const;
            void softPowerDown() const;
            bool softPowerUp(int timeout = 60) const;
            
    };
}
#endif