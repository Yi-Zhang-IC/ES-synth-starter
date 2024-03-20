#include "Phasor.hpp"
#include <cstdint>
#include <string>

struct Note {
    uint8_t idx;
    std::string name;
    Phasor phase;
};
