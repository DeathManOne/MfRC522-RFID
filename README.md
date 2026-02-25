same code than [PiRC522](https://github.com/DeathManOne/PiRC522) but, don't know why, piccSelect = PICC_TIMEOUT (in _piccCommunication)
## WORK
Detection (piccWaitTag)

## DOES NOT WORK
Select (piccSelect)

if you want help to me ... it will be with pleasure


## Initialize
```c++
MfRC522::driver *_RFID;
int RFID_PIN_SDA = 5;
int RFID_ANTENNA_LEVEL = 4;
int RFID_WAIT_TAG_TIMEOUT = 15;

void setup {
  SPI.begin();
  _RFID = new MfRC522::driver(RFID_PIN_SDA, SPI);
}

void loop {
  if (!_RFID->piccWaitTag(RFID_ANTENNA_LEVEL, RFID_WAIT_TAG_TIMEOUT)) {
    Serial.printf("TIMEOUT with antennaLevel: %d\n", RFID_ANTENNA_LEVEL);
    return;
  } else { Serial.println("NEW CARD...");}

  if (!_RFID->piccRequest(false)) {
    Serial.println("REQUEST FAILED");
    return;
  } else { Serial.println("REQUEST OK"); }

  std::vector<int> uid;
  int sak;
  std::string type;
  std::string status;
  if (!_RFID->piccSelect(uid, sak, type, status))
    { Serial.printf("NOT SELECTED with status: %s\n", status.c_str()); }
  else { Serial.println("SELECTED"); }
}
```
