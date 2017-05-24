#include "color_tools/color_tools.h"

namespace color_tools
{

/**
 * From: https://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
 */
void convert(const HSV & hsv, std_msgs::ColorRGBA & rgba)
{
    double h_ = hsv.h / 60.0;   // [0, 6]
    double c = hsv.v * hsv.s;
    double x = c * (1 - fabs(fmod(h_, 2) - 1));
    double r_ = 0.0;
    double g_ = 0.0;
    double b_ = 0.0;
    if(h_ < 1) {
        r_ = c;
        g_ = x;
    } else if(h_ < 2) {
        r_ = x;
        g_ = c;
    } else if(h_ < 3) {
        g_ = c;
        b_ = x;
    } else if(h_ < 4) {
        g_ = x;
        b_ = c;
    } else if(h_ < 5) {
        r_ = x;
        b_ = c;
    } else if(h_ < 6) {
        r_ = c;
        b_ = x;
    }

    double m = hsv.v - c;
    rgba.r = r_ + m;
    rgba.g = g_ + m;
    rgba.b = b_ + m;
    rgba.a = 1.0;
}

}
