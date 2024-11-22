#ifndef RFID_H
#define RFID_H

#include <stdbool.h>
#include <stdint.h>
#include "mfrc522.h"
#include <string.h> // For memcmp

// Initializes the RFID reader
void rfid_init(MFRC522Ptr_t *mfrc);

// Checks if a card is present
bool card_present(MFRC522Ptr_t mfrc);

// Reads the UID of the card
bool read_card_uid(MFRC522Ptr_t mfrc, uint8_t *uid, uint8_t *uid_length);

// Halts the card to prepare for the next one
void halt_card(MFRC522Ptr_t mfrc);

// Checks if the scanned UID is authorized
bool is_authorized_uid(uint8_t *uid, uint8_t uid_length);

#endif // RFID_READER_H
