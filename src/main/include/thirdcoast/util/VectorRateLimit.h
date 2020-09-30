#pragma once

#include <vector>
#include <cmath>
#include <memory>


class VectorRateLimit {
    double rateLimit;
    double kEpsilon = 0.0001;

    double maxFwdStp;
    double maxStrStp;

    double curVel;

    std::array<double, 2> output = {0.0, 0.0};

  public:
    VectorRateLimit(double rateLimit) : rateLimit(rateLimit) {}

    std::array<double, 2> apply(double forward, double strafe)
    {
        curVel = std::hypot(forward, strafe);

        //Calculate Steps
        maxFwdStp = std::sin(std::atan2(forward, strafe));
        maxStrStp = std::cos(std::atan2(forward, strafe));

        if (std::fabs(maxFwdStp) < kEpsilon)
        {
            maxFwdStp = rateLimit * (maxFwdStp - std::sin(std::atan2(output.at(0), output.at(1))));
        } else
        {
            maxFwdStp = rateLimit * maxFwdStp;
        }
        
        if (std::fabs(maxStrStp) < kEpsilon)
        {
            maxStrStp = rateLimit * (maxStrStp - std::sin(std::atan2(output.at(0), output.at(1))));
        } else
        {
            maxStrStp = rateLimit * maxStrStp;
        }

        if (std::fabs(forward - output.at(0)) > std::fabs(maxFwdStp))
        {
            output[0] = output.at(0) + std::copysign(maxFwdStp, forward - output.at(0));
        } else
        {
            output[0] = forward;
        }
        
        if (std::fabs(strafe - output.at(1)) > std::fabs(maxStrStp))
        {
            output[1] = output.at(1) + std::copysign(maxStrStp, strafe - output.at(1));
        } else
        {
            output[1] = strafe;
        }

        return output;
    }
    
};
