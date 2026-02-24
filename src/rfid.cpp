#include "../include/rfid.h"
using namespace MfRC522;

RFID::RFID(int pinIRQ, int pinRST, int pinSDA, SPIClass &SPI, int antennaLevel) {
    //this->_DRIVER = new driver(pinIRQ, pinRST, pinSDA, SPI);
    //this->_DRIVER->initialize(antennaLevel);
}

RFID::~RFID() {
    delete this->_DRIVER;
}

int RFID::getBlockAddress(int sector, int block) const {
    return (sector * 4) + block;
}

void RFID::getSectorBlock(int blockAddress, int &sector, int &block) const {
    block = blockAddress % 4;
    sector = (blockAddress - block) / 4;
}
