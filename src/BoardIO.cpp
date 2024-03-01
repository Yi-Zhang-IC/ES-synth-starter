#include "BoardIO.hpp"

void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
    digitalWrite(WEN_PIN, LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN, value);
    digitalWrite(WEN_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(WEN_PIN, LOW);
}

std::bitset<4> readRow()
{
    std::bitset<4> result;

    result[0] = digitalRead(C0_PIN);
    result[1] = digitalRead(C1_PIN);
    result[2] = digitalRead(C2_PIN);
    result[3] = digitalRead(C3_PIN);

    return result;
}

void selectRow(uint8_t rowIdx)
{
    digitalWrite(WEN_PIN, LOW);

    digitalWrite(RA0_PIN, rowIdx & 0x01);
    digitalWrite(RA1_PIN, rowIdx & 0x02);
    digitalWrite(RA2_PIN, rowIdx & 0x04);

    digitalWrite(WEN_PIN, HIGH);

    delayMicroseconds(3); // Wait for row voltage to settle
}
