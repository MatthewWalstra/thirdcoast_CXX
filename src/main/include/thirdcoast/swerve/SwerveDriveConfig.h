/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <AHRS.h>
#include <frc/TimedRobot.h>

#include <vector>
#include <memory>

#include "thirdcoast/swerve/Wheel.h"

namespace Thirdcoast {

class SwerveDriveConfig {
 public:
  SwerveDriveConfig(){}

  static const int WHEEL_COUNT = 4;
  /**
   * NavX gyro connected to MXP SPI port, used for field-oriented driving. If null, field-oriented
   * driving is disabled.
   */
  std::shared_ptr<AHRS> gyro;

  /** Initialize with four initialized wheels, in order from wheel 0 to wheel 3. */
  std::array<std::shared_ptr<Wheel>, WHEEL_COUNT> wheels;

  /** Wheel base length from front to rear of robot. */
  double length = 1.0;

  /** Wheel base width from left to right of robot. */
  double width = 1.0;

  /**
   * Robot period is the {@code TimedRobot} period in seconds, defaults to {@code
   * TimedRobot.kDefaultPeriod}.
   */
  double robotPeriod = frc::TimedRobot::kDefaultPeriod.value();

  /** Factor to correct gyro lag when simultaneously applying azimuth and drive. */
  double gyroRateCoeff = 0.0;

  bool enableSmartDashboardOutput = true;

};

}