/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "thirdcoast/swerve/SwerveDriveConfig.h"
#include "thirdcoast/swerve/Wheel.h"
#include "thirdcoast/util/Util.h"

#include <AHRS.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace Thirdcoast {

/**
 * Control a Third Coast swerve drive.
 *
 * <p>Wheels are a array numbered 0-3 from front to back, with even numbers on the left side when
 * facing forward.
 * 
 * Forward: Azimuth controller CAN IDs (+10 for drive motor controller)
 * 
 *  0 * * 1
 *  *  *  * 
 *  *  *  *
 *  2 * * 3 
 *
 * <p>Derivation of inverse kinematic equations are from Ether's <a
 * href="https://www.chiefdelphi.com/media/papers/2426">Swerve Kinematics and Programming</a>.
 *
 * @see Wheel
 */

class SwerveDrive {
  double DEFAULT_ABSOLUTE_AZIMUTH_OFFSET = 200;

  std::shared_ptr<AHRS> gyro;
  double kLengthComponent;
  double kWidthComponent;
  double kGyroRateCorrection;  
  std::array<std::shared_ptr<Wheel>, SwerveDriveConfig::WHEEL_COUNT> wheels;
  
  std::array<double, SwerveDriveConfig::WHEEL_COUNT> ws = {0.0, 0.0, 0.0, 0.0};
  std::array<double, SwerveDriveConfig::WHEEL_COUNT> wa  = {0.0, 0.0, 0.0, 0.0};

  std::array<int, SwerveDriveConfig::WHEEL_COUNT> ZeroPositions  = {13236 % 4096, -4385 % 4096, 48112 % 4096, -47876 % 4096};
  bool fieldOriented;
  bool enableTuning = false;
  bool enableSmartDashboardOutput = true;


 public:

  SwerveDrive(SwerveDriveConfig config);

  /**
   * Return key that wheel zero information is stored under in WPI preferences.
   *
   * @param wheel the wheel number
   * @return the String key
   */
  std::string getPreferenceKeyForWheel(int wheel);

  /**
   * Set the drive mode.
   *
   * @param driveMode the drive mode
   */
  void setDriveMode(Wheel::DriveMode driveMode);

  /**
   * Set all four wheels to specified values.
   *
   * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the robot
   *     straight-ahead position
   * @param drive 0 to 1 in the direction of the wheel azimuth
   */
  void set(double azimuth, double drive);

  /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  void drive(double forward, double strafe, double azimuth);

  /**
   * Stops all wheels' azimuth and drive movement. Calling this in the robots {@code teleopInit} and
   * {@code autonomousInit} will reset wheel azimuth relative encoders to the current position and
   * thereby prevent wheel rotation if the wheels were moved manually while the robot was disabled.
   */
  void stop();
  /**
   * Outputs setpoints to smartdashboard
   */
  void outputToSmartDashboard();

  /**
   * Save the wheels' azimuth current position as read by absolute encoder. These values are saved
   * persistently on the roboRIO and are normally used to calculate the relative encoder offset
   * during wheel initialization.
   *
   * <p>The wheel alignment data is saved in the WPI preferences data store and may be viewed using
   * a network tables viewer.
   *
   * @see #zeroAzimuthEncoders()
   */
  void saveAzimuthPositions();

  void saveAzimuthPositions(frc::Preferences *prefs);

  /**
   * Set wheels' azimuth relative offset from zero based on the current absolute position. This uses
   * the physical zero position as read by the absolute encoder and saved during the wheel alignment
   * process.
   *
   * @see #saveAzimuthPositions()
   */
  void zeroAzimuthEncoders();

  void zeroAzimuthEncoders(frc::Preferences *prefs);

  /**
   * Returns the four wheels of the swerve drive.
   *
   * @return the Wheel array.
   */
  std::array<std::shared_ptr<Wheel>, SwerveDriveConfig::WHEEL_COUNT> getWheels()
  {
    return wheels;
  }

  /**
   * Get the gyro instance being used by the drive.
   *
   * @return the gyro instance
   */
  std::shared_ptr<AHRS> getGyro()
  {
    return gyro;
  }

  /**
   * Reset Gyro to zero
   * 
   */

  void resetGyro()
  {
    gyro->Reset();
  }

  /**
   * Get status of field-oriented driving.
   *
   * @return status of field-oriented driving.
   */
  bool isFieldOriented()
  {
    return fieldOriented;
  }

  /**
   * Enable or disable field-oriented driving. Enabled by default if connected gyro is passed in via
   * {@code SwerveDriveConfig} during construction.
   *
   * @param enabled true to enable field-oriented driving.
   */
  void setFieldOriented(bool enabled)
  {
    fieldOriented = enabled;
  }

  /**
   * Get status of whether azimuth tuning is enabled
   */
  bool isEnableTuning()
  {
    return enableTuning;
  }

  /**
   * Enable or disable azimuth tuning
   **/
  void setEnableTuning(bool enable)
  {
    enableTuning = enable;
  }

  /**
   * Unit testing
   *
   * @return length
   */
  double getLengthComponent()
  {
    return kLengthComponent;
  }

  /**
   * Unit testing
   *
   * @return width
   */
  double getWidthComponent()
  {
    return kWidthComponent;
  }

  std::array<double, SwerveDriveConfig::WHEEL_COUNT> getWS()
  {
    return ws;
  }

  std::array<double, SwerveDriveConfig::WHEEL_COUNT> getWA()
  {
    return wa;
  }

  void outputSmartDashboard();

};

}