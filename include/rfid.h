#ifndef _RFID_
#define _RFID_

#include <Arduino.h>
#include <SPI.h>
#include "mfrc522.h"

namespace MfRC522 {
    class RFID {
        private:
            driver *_DRIVER;
        public:
            RFID(int pinIRQ, int pinRST, int pinSDA, SPIClass &SPI, int antennaLevel = 4);
            ~RFID();
            int getBlockAddress(int sector, int block) const;
            void getSectorBlock(int blockAddress, int &sector, int &block) const;
            
    };
}
#endif