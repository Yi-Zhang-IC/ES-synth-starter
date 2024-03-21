#include <cstdint>

/// @brief A struct storing a monochrome bitmap image in [XBM](https://en.wikipedia.org/wiki/X_BitMap) format.
struct XBMBitmap {
    uint32_t width, height;
    uint8_t *bits;
};

extern const XBMBitmap squareIcon;
extern const XBMBitmap sawtoothIcon;
extern const XBMBitmap triangleIcon;
extern const XBMBitmap sineIcon;
