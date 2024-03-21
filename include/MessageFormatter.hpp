#include <cstdint>
#include <array>

/// @brief Generators for raw CAN message contents of various types given their field values.
namespace MessageFormatter {
    /// @brief Generated a note start/stop message.
    /// @param keyDown `true` if the key has been pressed, i.e. the note should start playing
    /// @param keyIndex The index of the note to play, starting from 0 for C4
    std::array<uint8_t, 1> keyEvent(bool keyDown, uint8_t keyIndex);

    /// @brief Generates a settings update message.
    /// @param optionID A byte indicating the targeted setting, e.g. volume, waveform
    /// @param value The new value for that setting
    std::array<uint8_t, 2> settingUpdate(uint8_t optionID, uint8_t value);

    /// @brief Generates an enumeration position broadcast message.
    /// @param position The physical position of the current keyboard, start at 0 on the leftmost keyboard
    /// @param id A 32-bit unique ID of the current keyboard.
    std::array<uint8_t, 5> enumerationPositionAndID(uint8_t position, uint32_t id);

    /// @brief Generates an enumeration complete message.
    /// @note This message should only be sent by the rightmost keyboard.
    std::array<uint8_t, 1> enumerationDone();
};
