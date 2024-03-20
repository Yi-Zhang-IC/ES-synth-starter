#include "UniqueID.hpp"
#include <Arduino.h>
#include <cstdlib>
#include <CRC.h>

uint32_t UniqueID::calculate(void)
{
    uint8_t mcu_uid[12];
    memcpy(mcu_uid, (uint8_t*)UID_BASE, sizeof(mcu_uid));
    return calcCRC32(mcu_uid, sizeof(mcu_uid));
}
