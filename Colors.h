#ifndef Colors_h
#define Colors_h

#include <NeoPixelBus.h>

#define colorSaturation 4
const RgbColor red(colorSaturation, 0, 0);
const RgbColor green(0, colorSaturation, 0);
const RgbColor blue(0, 0, colorSaturation);
const RgbColor white(colorSaturation);
const RgbColor black(0);

#endif