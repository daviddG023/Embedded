#include "rfid.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

const uint8_t authorized_uid[] = {0x07, 0x20, 0x01, 0x26};
const uint8_t authorized_uid_length = sizeof(authorized_uid);

// Initializes the RFID reader
void rfid_init(MFRC522Ptr_t *mfrc) {
    // Initialize the RFID reader structure
    *mfrc = MFRC522_Init();    
    // Initialize the reader
    PCD_Init(*mfrc, spi0);
}

// Checks if a card is present
bool card_present(MFRC522Ptr_t mfrc) {
    return PICC_IsNewCardPresent(mfrc);
}

// Reads the UID of the card
bool read_card_uid(MFRC522Ptr_t mfrc, uint8_t *uid, uint8_t *uid_length) {
    if (PICC_ReadCardSerial(mfrc)) {
        // Copy the UID and its length from the MFRC522 structure
        for (uint8_t i = 0; i < mfrc->uid.size; i++) {
            uid[i] = mfrc->uid.uidByte[i];
        }
        *uid_length = mfrc->uid.size;
        return true;
    }
    return false;
}

// Halts the card to prepare for the next one
void halt_card(MFRC522Ptr_t mfrc) {
    PICC_HaltA(mfrc);
}
bool is_authorized_uid(uint8_t *uid, uint8_t uid_length) {
    // Compare the scanned UID with the authorized UID
    if (uid_length == authorized_uid_length &&
        memcmp(uid, authorized_uid, authorized_uid_length) == 0) {
        return true; // UID matches
    }
    return false; // UID doesn't match
}
