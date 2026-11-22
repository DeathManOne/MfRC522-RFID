/*
 * examples/ReadBlock.ino
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

#include <HardwareSerial.h>
#include <MfRC522.h>
#include <SPI.h>
#include <esp32-hal.h>

#include <string>
#include <vector>

namespace {
    constexpr int chipSelect       = D5;
    constexpr int detectionTimeout = 2;
    constexpr int testSector       = 1;
    constexpr int testBlock        = 0;
    const std::vector<int> sectorKey(6, 0xFF);

    struct Tag {
        std::vector<int> uid;
        int sak = 0;
        std::string type;
    };

    void report(const char *test, bool passed, const std::string &status) {
        Serial.printf("%s %s [%s]\n", passed ? "PASS" : "FAIL", test, status.c_str());
    }

    void printBytes(const std::vector<int> &bytes) {
        for (int byte : bytes)
            { Serial.printf(" %02X", static_cast<unsigned int>(byte)); }
        Serial.println();
    }

    bool selectTag(MfRC522::driver &reader, Tag &tag) {
        int antennaGain = 4;
        std::string status;
    
        const bool found = reader.piccWaitTag(detectionTimeout, antennaGain, status);
        report("piccWaitTag", found, status);
        if (!found) { return false; }

        const bool requested = reader.piccRequest(status);
        report("piccRequest", requested, status);
        if (!requested) { return false; }

        const bool selected = reader.piccSelect(tag.uid, tag.sak, tag.type, status);
        report("piccSelect", selected, status);
        if (selected) {
            Serial.printf("Type=%s SAK=%02X UID (%u bytes):", tag.type.c_str(),
                static_cast<unsigned int>(tag.sak),
                static_cast<unsigned int>(tag.uid.size())
            );
            printBytes(tag.uid);
        }
        return selected;
    }

    void readBlock(MfRC522::driver &reader, int sector, int block) {
        const int blocksInSector = sector < 32 ? 4 : 16;

        if (sector < 0 || sector >= 40 || block < 0 || block >= blocksInSector) {
            report("read arguments", false, "INVALID_SECTOR_OR_BLOCK");
            return;
        }

        Tag tag;
        if (!selectTag(reader, tag)) { return; }

        std::string status;
        const bool authenticated = reader.MFAuthenticate(sector, sectorKey, tag.uid, status);
        report("MFAuthenticate", authenticated, status);

        if (authenticated) {
            std::vector<int> data;
            const bool read = reader.MFRead(sector, block, data, status);
            report("MFRead", read && data.size() == 16, status);

            if (read)
                { printBytes(data); }
        }
        reader.piccUnselect();
    }

    void runTest(MfRC522::driver &reader) {
        readBlock(reader, testSector, testBlock);
    }
}

void setup() {
    Serial.begin(115200);
    SPI.begin();
}

void loop() {
    static MfRC522::driver reader(chipSelect, SPI);
    static bool prompted = false;

    if (!prompted && Serial) {
        Serial.println("Place a Classic tag with default Key A on the reader, then press Enter.");
        prompted = true;
    }

    if (!Serial.available()) {
        delay(10);
        return;
    }

    if (Serial.read() != '\n')
        { return; }
    runTest(reader);
}
