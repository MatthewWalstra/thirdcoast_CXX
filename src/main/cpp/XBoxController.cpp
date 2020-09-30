/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "XBoxController.h"
#include <frc/smartdashboard/SmartDashboard.h>
using namespace ControlBoard;
XBoxController::XBoxController(int port) 
{
    mController = std::make_shared<frc::Joystick>(port);
    frc::SmartDashboard::PutNumber("Joystick Port", port);
}

double XBoxController::getJoystick(Side side, Axis axis)
{
    bool left = side == Side::LEFT;
    bool y = axis == Axis::y;
    //inverts normally inverted y-axis
    return (y ? -1.0 : 1.0)*mController->GetRawAxis((left ? 0 : 4) + (y ? 1 : 0));
}

bool XBoxController::getTrigger(Side side)
{
    return mController->GetRawAxis(side == Side::LEFT ? 2 : 3) > kJoystickThreshold;
}

bool XBoxController::getButton(int button)
{
    frc::SmartDashboard::PutNumber("Joystick button", button);
    return mController->GetRawButton(button);
}

int XBoxController::getDPad()
{
    return mController->GetPOV();
}

void XBoxController::setRumble(Side side, bool on)
{
    mController->SetRumble((side == Side::LEFT ? frc::GenericHID::RumbleType::kLeftRumble : frc::GenericHID::RumbleType::kLeftRumble), on ? 1.0 : 0.0);
}

double XBoxController::handleDeadband(double value, double deadband)
{
    return std::fabs(value) > std::fabs(deadband) ? value : 0.0;
}