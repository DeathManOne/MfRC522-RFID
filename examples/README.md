# MfRC522 examples

Use one sketch at a time as the entry point in a PlatformIO project with
MfRC522-RFID installed. The examples target Arduino Nano ESP32 with CS on
D5, SCK on D13, MISO on D12, MOSI on D11, and a 3.3 V reader supply.
Set the serial monitor to 115200 baud and LF (New Line), then press Enter
to run a test.

| Sketch | Test |
| --- | --- |
| Scan | Detect and select a tag, then display its UID and SAK |
| NoTagTimeout | Verify the expected timeout with an empty RF field |
| ReadBlock | Authenticate with the default Key A and read a block |
| InvalidAddress | Verify that an out-of-range read clears its output |
| Reselect | Halt, wake, and reselect the same tag |
| SoftPower | Sleep, wake, and select the same tag after wake-up |
| Antenna | Check gain codes and disable and re-enable the field |
| Dump | Compare forward and reverse dumps on Classic 1K or 4K |
| WriteBlock | Write a pattern, read it back, and restore the original block |
| ValueBlock | Format, increment, decrement, and restore a value block, then restore the original data |
| Trailer | Check trailer protection and preserve factory settings and the user byte |
| TwoTags | Select one of two tags with configured UIDs |
| UidMode | Check the official UID usage personalization command on a compatible seven-byte Classic card |

Use a scratch data block for WriteBlock and ValueBlock. Keep the same tag
in the field until backup restoration is verified. Trailer requires a
sector with factory keys and access bits.

For TwoTags, copy each UID from Scan into the sketch before uploading.

UidMode is disabled by default. On an accepting card, personalization
permanently locks its UID usage mode. Use a dedicated card for this example.

Hardware tests were completed on Classic 1K. Classic 4K and UidMode are
pending hardware verification.
