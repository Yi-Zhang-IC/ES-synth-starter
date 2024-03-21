#include <cstdint>

enum class EventType {
    SETTINGS_UPDATE,
    NOTE_CHANGE
};

enum class SettingName {
    VOLUME,
    WAVEFORM
};

enum class NoteChange {
    START,
    STOP
};

struct GenericEvent {
    EventType type;
    uint8_t subtype;
    uint8_t value;
};
