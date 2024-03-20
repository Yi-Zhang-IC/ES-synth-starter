#include <cstdint>
#include <array>

namespace MessageFormatter {
    std::array<uint8_t, 1> keyEvent(bool keyDown, uint8_t keyIndex);
    std::array<uint8_t, 2> optionUpdate(uint8_t optionID, uint8_t value);
    std::array<uint8_t, 5> enumerationPositionAndID(uint8_t position, uint32_t id);
    std::array<uint8_t, 1> enumerationDone();
};
