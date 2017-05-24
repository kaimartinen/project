#ifndef COLOR_TOOLS_H
#define COLOR_TOOLS_H

#include <std_msgs/ColorRGBA.h>

namespace color_tools
{
    struct HSV
    {
        double h;   ///< [0, 360]
        double s;
        double v;
    };

    void convert(const HSV & hsv, std_msgs::ColorRGBA & rgba);
}

#endif

