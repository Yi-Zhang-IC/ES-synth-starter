#include <cstdint>

struct Bitmap {
    uint32_t width, height;
    uint8_t *bits;
};

extern const Bitmap squareIcon;
extern const Bitmap sawtoothIcon;
extern const Bitmap triangleIcon;
extern const Bitmap sineIcon;
