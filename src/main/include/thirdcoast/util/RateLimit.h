/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>


class RateLimit {
  double rateLimit = 0.0;
  double lastLimit = 0.0;
 public:
  RateLimit(double rate_limit){rateLimit = rate_limit;}


  /**
   * @param joystick_input: joystick axis position
   * @return the joystick axis position after rate limiting
   */
  double apply(double joystick_input)
  {
    double y;
    if (std::fabs(joystick_input - lastLimit) > rateLimit)
    {
      y = lastLimit + std::copysign(rateLimit, joystick_input - lastLimit);
    } else 
    {
      y = joystick_input;
    }

    lastLimit = y;
    return y;
  }
};
