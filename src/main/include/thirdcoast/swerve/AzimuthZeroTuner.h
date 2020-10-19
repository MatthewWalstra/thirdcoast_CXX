/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <functional>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <thirdcoast/swerve/SwerveDrive.h>
namespace Thirdcoast {
class AzimuthZeroTuner {
  int i = 0;
  enum Wheels
  {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    BACK_LEFT = 2,
    BACK_RIGHT = 3
  };
 public:
  AzimuthZeroTuner();
  void update(std::shared_ptr<Thirdcoast::SwerveDrive> drive);
  frc::SendableChooser<Wheels> mWheelChooser;
  
};

}