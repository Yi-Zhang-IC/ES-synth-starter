#include <cstdint>

class Phasor
{
public:
    Phasor() = delete;
    Phasor(float freqHz, uint32_t sampleRateHz);
    uint32_t next();
    uint32_t getFreqHz() const;

private:
    const float freqHz;
    const uint32_t phaseIncStep;
    uint32_t phase;
};
