#include "Bitmaps.hpp"

static unsigned char square_bits[] = { 0x07, 0x07, 0x07, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
    0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x70, 0x70, 0x70 };
const Bitmap squareIcon = { 24, 8, square_bits };

static unsigned char sawtooth_bits[] = { 0x7f, 0x7f, 0x7f, 0x3f, 0x3f, 0x3f, 0x5f, 0x5f, 0x5f, 0x6f, 0x6f, 0x6f, 0x77,
    0x77, 0x77, 0x7b, 0x7b, 0x7b, 0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7e };
const Bitmap sawtoothIcon = { 24, 8, sawtooth_bits };

static unsigned char triangle_bits[] = { 0xef, 0xef, 0xef, 0xd7, 0xd7, 0xd7, 0xd7, 0xd7, 0xd7, 0xbb, 0xbb, 0xbb, 0xbb,
    0xbb, 0xbb, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0xfe, 0xfe, 0xfe };
const Bitmap triangleIcon = { 24, 8, triangle_bits };

static unsigned char sine_bits[] = { 0xe7, 0xe7, 0xe7, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xbd, 0xbd,
    0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0x7e, 0x7e, 0x7e };
const Bitmap sineIcon = { 24, 8, sine_bits };