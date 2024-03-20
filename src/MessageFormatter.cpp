#include "MessageFormatter.hpp"

std::array<uint8_t, 1> MessageFormatter::keyEvent(bool keyDown, uint8_t keyIndex)
{
    return { (keyDown ? 0x40 : 0x00) | (keyIndex & 0x3F) };
}

std::array<uint8_t, 2> MessageFormatter::optionUpdate(uint8_t optionID, uint8_t value)
{
    return { 0x80 | (optionID & 0x3F), value };
}

std::array<uint8_t, 5> MessageFormatter::enumerationPositionAndID(uint8_t position, uint32_t id)
{
    return { 0xC0 | (position & 0x1F), (id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF };
}

std::array<uint8_t, 1> MessageFormatter::enumerationDone()
{
    return { 0xE0 };
}
