/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "thirdcoast/swerve/Wheel.h"
#include <iostream>

//using namespace Thirdcoast;

Thirdcoast::Wheel::Wheel(std::shared_ptr<TalonSRX> azimuth, std::shared_ptr<rev::CANSparkMax> drive, double driveSetpointMax, int id) 
{
    azimuthController = azimuth;
    driveController = drive;
    this->driveSetpointMax = driveSetpointMax;
    this->id = id;
    
    setDriveMode(DriveMode::TELEOP);
}

void Thirdcoast::Wheel::set(double azimuth, double drive, bool output_smartdashboard)
{
    if (Util::epsilonEquals(drive, 0.0))
    {
        driveController->GetPIDController().SetReference(0.0, rev::ControlType::kDutyCycle);
        return;
    }

    azimuth *= -TICKS; //flip azimuth hardware configuration dependent

    double azimuthPosition = azimuthController->GetSelectedSensorPosition(0);
    double azimuthError = std::fmod(azimuth - azimuthPosition, TICKS);

    //minimize azimuth rotation, reversing drive if necessary
    
    inverted = std::fabs(azimuthError) > .25 * TICKS;

    std::string flipped = "";

    if (inverted)
    {
        azimuthError -= std::copysign(0.5 * TICKS, azimuthError);
        drive = -drive;

        flipped = ": flipped ";
    }
    
    if (output_smartdashboard)
    {
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Drive_" + Util::sstr(id) + " setpoint", drive);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " setpoint", azimuthPosition + azimuthError);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " position", azimuthPosition);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " error", azimuthError);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " zero", azimuth_zero);
    }
    

    stream << std::fixed << std::setprecision(4) << "Azimuth " << id << " ( Position: " << azimuthPosition << ", Error: " << azimuthError << ", Setpoint: " << azimuth << " )" << flipped << std::endl;

    azimuthController->Set(ControlMode::Position, azimuthPosition + azimuthError);
    driveController->Set(drive);
}

void Thirdcoast::Wheel::setAzimuthPosition(int position)
{
    azimuthController->Set(ControlMode::MotionMagic, position);
    //frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " position", azimuthController->GetSelectedSensorPosition(0));
}

void Thirdcoast::Wheel::disableAzimuth()
{
    azimuthController->NeutralOutput();
}

void Thirdcoast::Wheel::setDriveMode(DriveMode driveMode)
{
    this->driveMode = driveMode;
}

void Thirdcoast::Wheel::stop()
{
    azimuthController->Set(ControlMode::MotionMagic, azimuthController->GetSelectedSensorPosition(0));
    driveController->GetPIDController().SetReference(0.0, rev::ControlType::kDutyCycle);
}

void Thirdcoast::Wheel::setAzimuthZero(int zero)
{
    azimuth_zero = zero;
    int azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    azimuthController->SetSelectedSensorPosition(azimuthSetpoint, 0, 10);
    azimuthController->Set(ControlMode::MotionMagic, azimuthSetpoint);
}

int Thirdcoast::Wheel::getAzimuthAbsolutePosition()
{
    return azimuthController->GetSensorCollection().GetPulseWidthPosition();
}

std::string Thirdcoast::Wheel::getString()
{
    return stream.str();
}
