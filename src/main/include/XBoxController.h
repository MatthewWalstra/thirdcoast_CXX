/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Joystick.h>
#include <frc/GenericHID.h>

#include <memory>

namespace ControlBoard {

class XBoxController {
  double kJoystickThreshold = .65;
 public:
  XBoxController(int port);
  std::shared_ptr<frc::Joystick> mController;

  enum Side {LEFT, RIGHT};
  enum Axis {x, y};
  enum Button {A = 1, B = 2, X = 3, Y = 4, LB = 5, RB = 6, BACK = 7, START = 8, L_JOYSTICK = 9, R_JOYSTICK = 10};
  

  double getJoystick(Side side, Axis axis);

  bool getTrigger(Side side);
  bool getButton(int button);

  int getDPad();
  void setRumble(Side side, bool on);

  double handleDeadband(double value, double deadband);
};
}