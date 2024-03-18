#include <cstdint>

namespace UniqueID
{
/// @brief Generate a 32-bit hash of the MCU's 96-bit hardware UID.
uint32_t calculate(void);

}; // namespace UniqueID
