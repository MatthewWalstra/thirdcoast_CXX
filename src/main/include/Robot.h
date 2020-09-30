/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//#include "thirdcoast/util/Util.h"

#include "thirdcoast/swerve/SwerveDrive.h"
#include "XBoxController.h"
#include "thirdcoast/util/ExpoScale.h"
#include "thirdcoast/util/VectorRateLimit.h"

#include "Constants.h"

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
  ControlBoard::XBoxController controller{0};
  std::shared_ptr<Thirdcoast::SwerveDrive> drive;
  ExpoScale yawExpo{Constants::DEADBAND, Constants::YAW_EXPO};
  ExpoScale driveExpo{Constants::DEADBAND, Constants::DRIVE_EXPO};
  VectorRateLimit vectorLimit{Constants::VECTOR_LIMIT};
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  double getForward();
  double getStrafe();
  double getYaw();

  std::shared_ptr<Thirdcoast::SwerveDrive> configSwerve();
  std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> getWheels();
};
