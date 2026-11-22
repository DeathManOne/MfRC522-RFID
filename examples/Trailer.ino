/*
 * examples/Trailer.ino
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

#include <algorithm>
#include <string>
#include <vector>

namespace {
    constexpr int chipSelect       = D5;
    constexpr int detectionTimeout = 2;
    constexpr int testSector       = 1;
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
            {  Serial.printf(" %02X", static_cast<unsigned int>(byte)); }
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

    bool verifyBlock(MfRC522::driver &reader, int sector, int block, const std::vector<int> &expected, const char *test) {
        std::vector<int> actual;
        std::string status;

        const bool read  = reader.MFRead(sector, block, actual, status);
        const bool match = read && actual == expected;

        report(test, match, read && !match ? "DATA_MISMATCH" : status);
        return match;
    }

    void testTrailer(MfRC522::driver &reader, int sector) {
        if (sector <= 0 || sector >= 40) {
            report("trailer-test", false, "CHOOSE_SECTOR_1_TO_39");
            return;
        }

        Tag tag;
        if (!selectTag(reader, tag)) { return; }
        if (tag.type != "MIFARE_4K" && (tag.type != "MIFARE_1K" || sector >= 16)) {
            report("trailer-test", false, "CARD_TYPE_OR_CAPACITY_MISMATCH");
            return;
        }

        std::string status;
        const bool authenticated = reader.MFAuthenticate(sector, sectorKey, tag.uid, status);
        report("MFAuthenticate", authenticated, status);
        if (!authenticated) {
            reader.piccUnselect();
            return;
        }

        const int trailer = sector < 32 ? 3 : 15;
        std::vector<int> before;

        const bool read = reader.MFRead(sector, trailer, before, status);
        report("read trailer", read, status);
        if (!read || before.size() != 16) {
            reader.piccUnselect();
            return;
        }

        const bool factorySettings = before[6] == 0xFF && before[7] == 0x07 && before[8] == 0x80 && std::equal(sectorKey.begin(), sectorKey.end(), before.begin() + 10);
        if (!factorySettings) {
            report("trailer-test", false, "EXPECTED_FACTORY_ACCESS_BITS_AND_READABLE_DEFAULT_KEY_B");
            reader.piccUnselect();
            return;
        }

        std::vector<int> sameTrailer = before;
        std::copy(sectorKey.begin(), sectorKey.end(), sameTrailer.begin());

        const bool written          = reader.MFWrite(sector, trailer, sameTrailer, status);
        const bool protectedTrailer = !written && status == "ERROR_CAN_NOT_WRITE_TRAILER";

        report("MFWrite trailer protection", protectedTrailer, status);
        if (protectedTrailer) {
            const bool rewritten = reader.MFChangeTrailer(sector, sectorKey, sectorKey, {0, 0, 0, 1}, status);
            report("MFChangeTrailer", rewritten, status);
            if (rewritten)
                { verifyBlock(reader, sector, trailer, before, "trailer readback and user byte"); }
        }
        reader.piccUnselect();
    }

    void runTest(MfRC522::driver &reader) {
        testTrailer(reader, testSector);
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
        Serial.println("Use a Classic test card with factory keys and access bits. Press Enter to run the trailer test.");
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
