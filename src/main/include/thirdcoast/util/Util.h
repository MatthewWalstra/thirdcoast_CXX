#pragma once

#include <string>
#if defined(_USE_MATH_DEFINES) && !defined(_MATH_DEFINES_DEFINED)
    #define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <iomanip>

#include "Constants.h"


class Util {
    static constexpr double kEpsilon = 1E-6;
  public:
    Util();
    
    /**
     * Calculates whether two doubles are equal
     * 
     * @param x
     * @param y
     * @param epsilon error amount, + or -
     * 
     * @return bool whether two doubles are equal
     */
    static bool epsilonEquals(double x, double y, double epsilon)
    {
        return ((x - epsilon <= y) && (x + epsilon >= y));
    }
    /**
     * Calculates whether two doubles are equal with standard kEpsilon = 1E-6
     * 
     * @param x
     * @param y
     * 
     * @return bool whether two doubles are equal
     */
    static bool epsilonEquals(double x, double y)
    {
        return epsilonEquals(x, y, kEpsilon);
    }

    template < class T >
    static std::string sstr( T args )
    {
        std::ostringstream sstr;
        // fold expression
        ( sstr << std::setprecision(2) << std::fixed << args );
        return sstr.str();
    }

    static double degToRads(double degrees)
    {
        return degrees * Constants::PI / 180.0;
    }

    static double radsToDeg(double radians)
    {
        return radians * 180.0 / Constants::PI;
    }

    static double limit(double v, double min, double max)
    {
        return std::fmin(max, std::fmax(min, v));
    }

    static double limit(double v, double lim)
    {
        return limit(v, -lim, lim);
    }
};