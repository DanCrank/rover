// example encrptyion key - put your real key in encryption_key.h
#include <Arduino.h>

#ifndef ROVER_ENCRYPTION_KEY_H
#define ROVER_ENCRYPTION_KEY_H

uint8_t encryptionKey[] = { 0, 1, 2, 3, 4, 5, 6, 7,
                             8, 9, 10, 11, 12, 13, 14, 15 };

uint8_t syncWords[] = { 0, 1 };

#endif //ROVER_ENCRYPTION_KEY_H