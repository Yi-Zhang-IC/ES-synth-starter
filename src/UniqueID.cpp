#include "UniqueID.hpp"
#include <Arduino.h>
#include <cstdlib>
#include <CRC.h>

uint32_t UniqueID::calculate(void)
{
    uint8_t mcu_uid[12];
    memcpy(mcu_uid, (uint8_t*)UID_BASE, sizeof(mcu_uid));

    for (size_t i = 0; i < sizeof(mcu_uid); i++) {
        Serial.printf("%02x ", mcu_uid[i]);
    }
    Serial.println();

    return calcCRC32(mcu_uid, sizeof(mcu_uid));
}
