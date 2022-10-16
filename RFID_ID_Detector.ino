#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#define PN532_IRQ   (2)
#define PN532_RESET (3)
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);
  // configure board to read RFID tags
  nfc.SAMConfig();
  // Serial.println("Show us your card!");
}
void loop() {
  // RFID
  Serial.println("Hello");
  boolean detector;                            //The value that is going to be used to detect whether a tag is found
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };     //Buffer to store the returned UID from tag. If there is an issue where 
                                               //the reader seems to only read a certain number of times, take out 3 0s
                                               //to match with the byte size of the tags.
  uint8_t obj1[] = {0xF9, 0x3E, 0x4, 0xF4};    //Yellow Tower's id
  uint8_t obj2[] = {0xC9, 0x12, 0xD, 0xF4};    //Blue Tower's id
  uint8_t obj3[] = {0x59, 0xE3, 0xB, 0xF4};    //Red Tower's id
  uint8_t obj4[] = {0x59, 0xC8, 0x6, 0xF4};    //Orange Tower's id
  uint8_t obj5[] = {0x69, 0xDB, 0x6, 0xF4};    //Purple Tower's id
  uint8_t uidLength;
  detector = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
  Serial.println(detector);
  if(detector){
      Serial.println("Found a tag!");
      Serial.println("This is .....");
      if(memcmp(obj1, uid, 4) == 0){
        Serial.println("Yellow Block");
        delay(1000);
      }
      else if (memcmp(obj2, uid, 4) == 0){
        Serial.println("Blue Block");
        delay(1000);
      }
      else if(memcmp(obj3, uid, 4) == 0){
        Serial.println("Red Block");
        delay(1000);
      }
      else{
        Serial.println("Not in database");
        delay(1000);
      }
      delay(500);
    }
    else{
      Serial.println("No Objects in Range");
    }
  }
