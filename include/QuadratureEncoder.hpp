#include <cstdint>

class QuadratureEncoder
{
public:
    enum class Direction { Unknown, Clockwise, Counterclockwise };

    QuadratureEncoder();
    void update(uint8_t currentState);
    int getChange();

private:
    int oldPosition, position; // Encoder position
    uint8_t prevState; // Previous state of the encoder signals
    Direction lastDirection;
};
