#include <array>
#include <string>
#include <cstdint>
#include <unordered_map>
#include <functional>

enum class WaveformName { SAWTOOTH, TRIANGLE, SQUARE, SINE };

/// @brief List of note frequencies. Starts at C4.
extern const std::array<float, 60> noteFreqs;

/// @brief List of note names as strings. Black keys are denoted as ♯ rather than ♭.
extern const std::array<std::string, 12> noteNames;

/// @brief Samples of one cycle of sine at the full range of int16_t.
extern const std::array<int16_t, 256> sineTable;

/// @brief Dictionary of functions mapping `uint32_t phase` to amplitude for different waveforms.
extern const std::unordered_map<WaveformName, std::function<int16_t(uint32_t)>> WaveformGenerators;
