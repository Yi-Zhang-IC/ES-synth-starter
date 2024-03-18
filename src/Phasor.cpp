#include "Phasor.hpp"

Phasor::Phasor(uint32_t freqHz, uint32_t sampleRateHz): phase(0), freqHz(freqHz), phaseIncStep(UINT32_MAX / (sampleRateHz / freqHz)) {}

uint32_t Phasor::next()
{
    phase += phaseIncStep;
    return phase;
}

uint32_t Phasor::getFreqHz() const
{
    return freqHz;
}
