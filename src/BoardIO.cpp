#include "BoardIO.hpp"

void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
    digitalWrite(WEN_PIN, LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN, value);
    digitalWrite(WEN_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(WEN_PIN, LOW);
}

uint8_t readRow()
{
    std::bitset<4> result;

    result[0] = digitalRead(C0_PIN);
    result[1] = digitalRead(C1_PIN);
    result[2] = digitalRead(C2_PIN);
    result[3] = digitalRead(C3_PIN);

    return static_cast<uint8_t>(result.to_ulong());
}

void selectRow(uint8_t rowIdx)
{
    digitalWrite(WEN_PIN, LOW);

    digitalWrite(RA0_PIN, rowIdx & 0x01);
    digitalWrite(RA1_PIN, rowIdx & 0x02);
    digitalWrite(RA2_PIN, rowIdx & 0x04);

    digitalWrite(WEN_PIN, HIGH);
    delayMicroseconds(1);
}

uint8_t selectAndReadRow(uint8_t rowIdx)
{
    selectRow(rowIdx);
    uint8_t result = readRow();
    digitalWrite(WEN_PIN, LOW);
    return result;
}
