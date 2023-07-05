#ifndef MATH_H
#define MATH_H

#include <math.h>

inline double radians_to_degrees(const double &radians)
{
    return radians * 180 / M_PI;
}

inline double degrees_to_radians(const double &degrees)
{
    return (degrees * (M_PI / 180));
}

#endif