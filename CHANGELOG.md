# Changelog

## 3.0.1

### Added

- Add 13 standalone sketches for testing the reader and MIFARE Classic operations on Arduino ESP32.

### Removed

- Remove the test-only anticollision trace and its callback.

## 3.0.0

### Changed

- Split the driver into focused source files for SPI, CRC, reader configuration, tag communication, MIFARE operations, and power management.
- Removed the direct dependency on `Arduino.h`. The library still targets the Arduino framework on ESP32.
- Updated the README examples and documented the public API's parameters, return values, and error statuses.
- Flush the FIFO with a direct register write before CRC calculations and PICC exchanges.

### Fixed

- Map the chip-select pin to its GPIO number on Arduino ESP32 boards that remap digital pins, including the Nano ESP32.
- Return complete 4-, 7-, and 10-byte UIDs after selection, validate the anticollision BCC, and resume at the reported collision bit.
- Use the last four UID bytes for MIFARE Classic authentication and report failure when Crypto1 stays disabled.
- Correct MIFARE Classic 4K sector addressing, trailer protection, and dump ranges. Skip sectors whose authentication fails.
- Validate block addresses, payload bytes, and value-block redundancy. Return an empty buffer on failed reads and preserve the trailer's user byte when changing keys or access conditions.
- Distinguish hardware faults from timeouts and report unexpected HALT responses. Use a 64-bit timer for tag polling.
- Reject PICC requests that exceed the 64-byte FIFO and invalid receive counts reported by the reader.
- Return success from `MFSevenByteUidFirstInit()` only when the card acknowledges the command.
- Change soft power state without reissuing the current reader command.
