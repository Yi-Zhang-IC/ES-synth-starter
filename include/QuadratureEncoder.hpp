#include <cstdint>

/// @brief A decoder for standard quadrature rotary encoders with two outputs.
class QuadratureEncoder
{
public:
    enum class Direction { Unknown, Clockwise, Counterclockwise };

    QuadratureEncoder();

    /// @brief Update the decoder with the outputs of the rotary encoder in order to detect rotation.
    /// @param currentState The two outputs of the rotary encoder in the two LSBs.
    void update(uint8_t currentState);

    /// @brief Get the number of steps rotated since the last call to `.getChange()`.
    /// @return The number of steps newly rotated, positive if clockwise and negative if anticlockwise.
    int getChange();

private:
    int oldPosition, position; // Encoder position
    uint8_t prevState; // Previous state of the encoder signals
    Direction lastDirection;
};
