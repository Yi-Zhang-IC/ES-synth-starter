#include "Phasor.hpp"

Phasor::Phasor(float freqHz, uint32_t sampleRateHz): phase(0), phaseIncStep(UINT32_MAX / (sampleRateHz / freqHz)) {}

uint32_t Phasor::next()
{
    phase += phaseIncStep;
    return phase;
}
