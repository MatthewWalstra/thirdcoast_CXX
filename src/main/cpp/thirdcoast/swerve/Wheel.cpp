/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "thirdcoast/swerve/Wheel.h"

using namespace Thirdcoast;

Wheel::Wheel(std::shared_ptr<TalonSRX> azimuth, std::shared_ptr<rev::CANSparkMax> drive, double driveSetpointMax) 
{
    azimuthController = azimuth;
    driveController = drive;
    this->driveSetpointMax = driveSetpointMax;

    setDriveMode(DriveMode::TELEOP);
}

void Wheel::set(double azimuth, double drive)
{
    if (Util::epsilonEquals(drive, 0.0))
    {
        driveController->GetPIDController().SetReference(0.0, rev::ControlType::kDutyCycle);
        return;
    }

    //azimuth *= -TICKS; //flip azimuth hardware configuration dependent

    double azimuthPosition = azimuthController->GetSelectedSensorPosition(0);
    double azimuthError = std::fmod(azimuth - azimuthPosition, TICKS);

    //minimize azimuth rotation, reversing drive if necessary
    inverted = std::fabs(azimuthError) > .25 * TICKS;
    if (inverted)
    {
        azimuthError -= std::copysign(0.5 * TICKS, azimuthError);
        drive = -drive;
    }

    azimuthController->Set(ControlMode::MotionMagic, azimuthPosition + azimuthError);
    driveController->GetPIDController().SetReference(drive, rev::ControlType::kDutyCycle);
}

void Wheel::setAzimuthPosition(int position)
{
    azimuthController->Set(ControlMode::MotionMagic, position);
}

void Wheel::disableAzimuth()
{
    azimuthController->NeutralOutput();
}

void Wheel::setDriveMode(DriveMode driveMode)
{
    this->driveMode = driveMode;
}

void Wheel::stop()
{
    azimuthController->Set(ControlMode::MotionMagic, azimuthController->GetSelectedSensorPosition(0));
    driveController->GetPIDController().SetReference(0.0, rev::ControlType::kDutyCycle);
}

void Wheel::setAzimuthZero(int zero)
{
    int azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    azimuthController->SetSelectedSensorPosition(azimuthSetpoint, 0, 10);
    azimuthController->Set(ControlMode::MotionMagic, azimuthSetpoint);
}

int Wheel::getAzimuthAbsolutePosition()
{
    return azimuthController->GetSensorCollection().GetPulseWidthPosition();
}