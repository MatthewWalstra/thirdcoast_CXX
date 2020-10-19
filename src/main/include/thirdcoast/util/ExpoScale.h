/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>


class ExpoScale {
  double deadband = 0.05;
  double scale = 0.0;
  double offset = 0.0;
 public:
  
  /* Applies exponential scaling and deadband to joystick inputs */
  ExpoScale(double deadband, double scale)
  {
    this->deadband = deadband;
    this->scale = scale;
    offset = 1.0 / (scale * std::pow(1 - deadband, 3.0) + (1.0 - scale) * (1.0 - deadband));
  }

  /**
   * Return the joystick axis position input adjusted on an exponential scale with deadband
   * adjustment.
   *
   * @param input the joystick axis position
   * @return the adjusted input value, range is -1.0 to 1.0
   */
  double apply(double input)
  {
    

    if (std::fabs(input) < deadband)
    {
      return 0.0;
    }
    
    double y = input > 0.0 ? input - deadband : input + deadband;
    return (scale * std::pow(y, 3.0) + (1 - scale) * y) * offset;
  }
};
