#include <cstdint>

namespace BitUtils
{
int8_t highestBitSet(uint32_t val);
uint32_t extractBitField(uint32_t val, uint8_t base_bitpos, uint8_t length);
bool bitIsSet(uint32_t val, uint8_t bitpos);
}; // namespace BitUtils
