/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <iomanip>
#include <thread>
#include <iostream>
#include "thirdcoast/nanoLog/NanoLog.hpp"

//#include "thirdcoast/util/Util.h"

#include "thirdcoast/swerve/SwerveDrive.h"
#include "XBoxController.h"
#include "thirdcoast/util/ExpoScale.h"
#include "thirdcoast/util/VectorRateLimit.h"
#include "thirdcoast/swerve/AzimuthZeroTuner.h"

#include "Constants.h"

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <frc/controller/PIDController.h>



class Robot : public frc::TimedRobot {
  //declare controller and drive
  ControlBoard::XBoxController controller{0};
  std::shared_ptr<Thirdcoast::SwerveDrive> drive;

  Thirdcoast::AzimuthZeroTuner azimuthTuner{};

  bool prev_save = false;
  bool prev_reset = false;
  double setpoint = 0.0;
  double prev_setpoint = 0.0;
  double prev_timestamp = 0.0;
  int i = 0;
  double j = 0.0;
  double scalar = 2.0 * Constants::PI * frc::TimedRobot::kDefaultPeriod.value();
  double circleStepsize = scalar / 3.0;
  double forwardStepsize = scalar / .75;
  bool finished_circle = false;
  bool finished_forward = false;

  std::fstream mWheelOutput;

  //declare rate limiters
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

  /**
   * @return xbox controller joysticks values
   * Forward = Left Y
   * Strafe = Left X
   * Yaw = Right X
   **/
  double getForward();
  double getStrafe();
  double getYaw();

  /**
   * @return true, if A && B && X && Y are pressed
   **/
  bool getAzimuthSaveTrigger();
  
   /**
   * @return true, if start is pressed
   **/
  bool getAzimuthResetTrigger();

  std::shared_ptr<Thirdcoast::SwerveDrive> configSwerve();
  std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> getWheels();
};
