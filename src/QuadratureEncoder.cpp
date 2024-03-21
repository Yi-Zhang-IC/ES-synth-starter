#include "QuadratureEncoder.hpp"
#include <cmath>

QuadratureEncoder::QuadratureEncoder(): oldPosition(0), position(0), prevState(0), lastDirection(Direction::Unknown) {}

void QuadratureEncoder::update(uint8_t currentState)
{
    uint8_t state = currentState & 0x03; // Only interested in the two least significant bits
    if (state == prevState) {
        return; // No change
    }

    switch (prevState << 2 | state) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        position++; // CW
        lastDirection = Direction::Clockwise; // Update last known direction to CW
        break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
        position--; // CCW
        lastDirection = Direction::Counterclockwise; // Update last known direction to CCW
        break;
    // Incorrect transitions, assume the direction based on lastDirection
    case 0b0011: // 00 -> 11
    case 0b0110: // 01 -> 10
    case 0b1100: // 11 -> 00
    case 0b1001: // 10 -> 01
        if (lastDirection == Direction::Clockwise) {
            position += 2; // Assume CW if last direction was CW
        } else if (lastDirection == Direction::Counterclockwise) {
            position -= 2; // Assume CCW if last direction was CCW
        }
        // If lastDirection is Unknown, do nothing
        break;
    default:
        break;
    }

    prevState = state; // Update the previous state for the next call
}

// Get the current position
int QuadratureEncoder::getChange()
{
    auto change = position - oldPosition;
    if (std::abs(change) >= 2) {
        oldPosition = position;
        return change / 2;
    } else {
        return 0;
    }
}
