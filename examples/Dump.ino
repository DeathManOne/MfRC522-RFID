/*
 * examples/Dump.ino
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

    void testDump(MfRC522::driver &reader) {
        Tag tag;
        if (!selectTag(reader, tag)) { return; }

        const bool classic4K = tag.type == "MIFARE_4K";
        if (!classic4K && tag.type != "MIFARE_1K") {
            report("dump-test", false, "EXPECTED_CLASSIC_1K_OR_4K");
            return;
        }

        const int sectorCount = classic4K ? 40 : 16;
        const int blockCount  = classic4K ? 256 : 64;
        const auto forward    = reader.MFDump(tag.uid, sectorKey, false, sectorCount);
        reader.piccUnselect();

        int address = 0;
        for (const auto &line : forward) {
            if (line.empty()) { continue; }

            const std::string marker  = " (" + std::to_string(address) + ") [";
            const auto markerPosition = line.find(marker);

            if (markerPosition == std::string::npos ||
                line.find_first_of("0123456789", markerPosition + marker.size()) != markerPosition + marker.size()) {
                Serial.printf("Expected block %d, got: %s\n", address, line.c_str());
                break;
            }
            ++address;
        }
        report("MFDump", address == blockCount, std::to_string(address) + "/" + std::to_string(blockCount) + " blocks");

        Tag reverseTag;
        if (!selectTag(reader, reverseTag) || reverseTag.uid != tag.uid) {
            report("MFDumpReverse", false, "ORIGINAL_CARD_NOT_SELECTED");
            return;
        }
        const auto reverse   = reader.MFDumpReverse(reverseTag.uid, sectorKey, false, sectorCount);
        reader.piccUnselect();
        const bool isReverse = reverse.size() == forward.size() && std::equal(forward.begin(), forward.end(), reverse.rbegin());
        report("MFDumpReverse", address == blockCount && isReverse, isReverse ? (address == blockCount ? "OK" : "INCOMPLETE_DUMP") : "ORDER_OR_DATA_MISMATCH");
    }

    void runTest(MfRC522::driver &reader) {
        testDump(reader);
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
        Serial.println("Place one Classic 1K or 4K tag with default sector keys on the reader, then press Enter.");
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
