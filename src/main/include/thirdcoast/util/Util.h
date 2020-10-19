#pragma once

#include <string>
#include <math.h>
#include <iomanip>


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
        return ((x - kEpsilon <= y) && (x + kEpsilon >= y));
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
        return degrees * M_PI / 180.0;
    }

    static double radsToDeg(double radians)
    {
        return radians * 180.0 / M_PI;
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