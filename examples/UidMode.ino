/*
 * examples/UidMode.ino
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
    constexpr int chipSelect           = D5;
    constexpr int detectionTimeout     = 2;
    constexpr bool enableUidModeTest   = false;
    const std::vector<int> expectedUid = {};
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

    void testUidMode(MfRC522::driver &reader, const std::vector<int> &expectedUid) {
        if (expectedUid.size() != 7) {
            report("uid-mode-test", false, "EXPECTED_7_BYTE_HEX_UID");
            return;
        }

        Tag tag;
        if (!selectTag(reader, tag)) { return; }
        if (tag.uid != expectedUid || (tag.type != "MIFARE_1K" && tag.type != "MIFARE_4K")) {
            report("uid-mode-test", false, "CARD_UID_OR_TYPE_MISMATCH");
            return;
        }

        reader.pcdAntennaEnable(false);
        delay(20);
        reader.pcdAntennaEnable();
        delay(20);

        std::string status;
        if (!reader.piccRequest(status, true)) {
            report("uid-mode-test WUPA", false, status);
            return;
        }
        const bool accepted = reader.MFSevenByteUidFirstInit(sectorKey, status, 0);
        report("MFSevenByteUidFirstInit", accepted, status);
        if (accepted) {
            Tag after;
            const bool selected = selectTag(reader, after);
            report("7-byte UID after personalization", selected && after.uid == expectedUid, selected ? (after.uid == expectedUid ? "OK" : "UID_CHANGED") : "SELECTION_FAILED");
        }
    }

    void runTest(MfRC522::driver &reader) {
        if (!enableUidModeTest) {
            report("uid-mode-test", false, "UID_MODE_TEST_DISABLED");
            return;
        }
        testUidMode(reader, expectedUid);
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
        Serial.println("On acceptance, this command permanently locks UID mode. Use only a dedicated compatible card.");
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
