#include <cstdint>

namespace BitUtils
{
/// @brief Finds the position of the highest 1 bit in `val`.
/// @param val
/// @return The position of the highest 1 bit in `val`, or -1 if `val` is zero
int8_t highestBitSet(uint32_t val);

/// @brief Extract a sequence of bits at a given position in `val`.
/// @param val The source variable
/// @param base_bitpos The position of the LSB of the desired field of bits
/// @param length The length of the field of bits
/// @return The field of bits with its LSB at bit 0
uint32_t extractBitField(uint32_t val, uint8_t base_bitpos, uint8_t length);

/// @brief Test a specified bit in `val`.
/// @param val The source variable
/// @param bitpos The position of the bit to test
/// @return The value of the bit at `bitpos`
bool bitIsSet(uint32_t val, uint8_t bitpos);
}; // namespace BitUtils
