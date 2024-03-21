#include <cstdint>

/// @brief A phase accumulator.
class Phasor
{
public:
    Phasor() = delete;

    /// @brief Construct a `Phasor` given the oscillation frequency and sample rate, so that the phase wraps around at the specified frequency when `.next()` is called at the specified sample rate.
    /// @param freqHz
    /// @param sampleRateHz
    Phasor(float freqHz, uint32_t sampleRateHz);

    /// @brief Increment the phase and return its value.
    /// @return Incremented phase
    uint32_t next();

private:
    const uint32_t phaseIncStep;
    uint32_t phase;
};
