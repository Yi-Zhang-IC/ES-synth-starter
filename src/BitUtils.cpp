#include "BitUtils.hpp"

int8_t BitUtils::highestBitSet(uint32_t val)
{
    if (val == 0) {
        return -1;
    }

    int pos = 0;
    while ((val >>= 1) != 0) {
        pos++;
    }
    return pos;
}

uint32_t BitUtils::extractBitField(uint32_t val, uint8_t base_bitpos, uint8_t length)
{
    return (val >> base_bitpos) & ((1u << length) - 1);
}
