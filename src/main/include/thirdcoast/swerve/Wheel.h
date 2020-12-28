/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <iomanip>

#include "thirdcoast/swerve/MotorControllerWrapper.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "thirdcoast/util/Util.h"
#include "thirdcoast/swerve/MotorControllerWrapper.h"

/**
 * Controls a swerve drive wheel azimuth and drive motors.
 *
 * <p>The swerve-drive inverse kinematics algorithm will always calculate individual wheel angles as
 * -0.5 to 0.5 rotations, measured clockwise with zero being the straight-ahead position. Wheel
 * speed is calculated as 0 to 1 in the direction of the wheel angle.
 *
 * <p>This class will calculate how to implement this angle and drive direction optimally for the
 * azimuth and drive motors. In some cases it makes sense to reverse wheel direction to avoid
 * rotating the wheel azimuth 180 degrees.
 *
 * <p>Hardware assumed by this class includes a CTRE magnetic encoder on the azimuth motor and no
 * limits on wheel azimuth rotation. Azimuth Talons have an ID in the range 0-3 with corresponding
 * drive Talon IDs in the range 10-13.
 */

namespace Thirdcoast {

class Wheel {
  
  double driveSetpointMax;
  std::shared_ptr<MotorControllerWrapper> azimuthController;
  std::shared_ptr<MotorControllerWrapper> driveController;
  bool inverted = false;
  int id = 0;

  int azimuth_zero = 0;
  double azimuthError = 0.0;

  std::stringstream stream;

  
 public:
  
  /**
   * This constructs a wheel with supplied azimuth and drive talons.
   *
   * <p>Wheels will scale closed-loop drive output to {@code driveSetpointMax}. For example, if
   * closed-loop drive mode is tuned to have a max usable output of 10,000 ticks per 100ms, set this
   * to 10,000 and the wheel will send a setpoint of 10,000 to the drive talon when wheel is set to
   * max drive output (1.0).
   *
   * @param azimuth the configured azimuth TalonSRX
   * @param drive the configured drive TalonSRX
   * @param driveSetpointMax scales closed-loop drive output to this value when drive setpoint = 1.0
   */
  Wheel();
  Wheel(std::shared_ptr<MotorControllerWrapper> azimuth, std::shared_ptr<MotorControllerWrapper> drive, double driveSetpointMax, int id);

  /**
   * This method calculates the optimal driveTalon settings and applies them.
   *
   * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the wheel's zeroed
   *     position
   * @param drive 0 to 1.0 in the direction of the wheel azimuth
   */
  void set(double azimuth, double drive, bool output_smartdashboard);

  /**
   * Set azimuth to encoder position.
   *
   * @param position position in encoder ticks.
   */
  void setAzimuthPosition(int position);

  void disableAzimuth();

  /**
   * Set the operating mode of the wheel's drive motors. In this default wheel implementation {@code
   * OPEN_LOOP} and {@code TELEOP} are equivalent and {@code CLOSED_LOOP}, {@code TRAJECTORY} and
   * {@code AZIMUTH} are equivalent.
   *
   * <p>In closed-loop modes, the drive setpoint is scaled by the drive Talon {@code
   * driveSetpointMax} parameter.
   *
   * <p>This method is intended to be overridden if the open or closed-loop drive wheel drivers need
   * to be customized.
   *
   * @param driveMode the desired drive mode
   */
  void setDriveMode(MotorControllerWrapper::DriveMode driveMode);

  /**
   * Stop azimuth and drive movement. This resets the azimuth setpoint and relative encoder to the
   * current position in case the wheel has been manually rotated away from its previous setpoint.
   */
  void stop();

  /**
   * Set the azimuthTalon encoder relative to wheel zero alignment position. For example, if current
   * absolute encoder = 0 and zero setpoint = 2767, then current relative setpoint = -2767.
   *
   * <pre>
   *
   * relative:  -2767                               0
   *           ---|---------------------------------|-------
   * absolute:    0                               2767
   *
   * </pre>
   *
   * @param zero zero setpoint, absolute encoder position (in ticks) where wheel is zeroed.
   */
  void setAzimuthZero(int zero);

  /**
   * Returns the wheel's azimuth absolute position in encoder ticks.
   *
   * @return 0 - 4095, corresponding to one full revolution.
   */
  int getAzimuthAbsolutePosition();

  /**
   * Get the azimuth controller.
   *
   * @return azimuth controller instance used by wheel
   */
  std::shared_ptr<MotorControllerWrapper> getAzimuthController(){return azimuthController;}

  /**
   * Get the drive controller.
   *
   * @return drive controller instance used by wheel
   */
  std::shared_ptr<MotorControllerWrapper> getDriveTalon(){return driveController;}

  double getDriveSetpointMax(){return driveSetpointMax;}

  bool isInverted(){return inverted;}

  std::string getString();

  double getAzimuthError(){ return azimuthError; }

};

}