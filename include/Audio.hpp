#include <array>
#include <string>
#include <cstdint>
#include <unordered_map>
#include <functional>

enum class WaveformName { SAWTOOTH, TRIANGLE, SQUARE, SINE };

extern const std::array<float, 60> noteFreqs;
extern const std::array<std::string, 12> noteNames;
extern const std::array<int16_t, 256> sineTable;
extern const std::unordered_map<WaveformName, std::function<int16_t(uint32_t)>> WaveformGenerators;
