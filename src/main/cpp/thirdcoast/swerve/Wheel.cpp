/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "thirdcoast/swerve/Wheel.h"
# include "Constants.h"
#include <frc/Timer.h>
#include <iostream>

//using namespace Thirdcoast;

Thirdcoast::Wheel::Wheel(std::shared_ptr<MotorControllerWrapper> azimuth, std::shared_ptr<MotorControllerWrapper> drive, double driveSetpointMax, int id) 
{
    azimuthController = azimuth;
    driveController = drive;
    this->driveSetpointMax = driveSetpointMax;
    this->id = id;
    
    setDriveMode(MotorControllerWrapper::DriveMode::TELEOP);
}

void Thirdcoast::Wheel::set(double azimuth, double drive, bool output_smartdashboard)
{
    if (Util::epsilonEquals(drive, 0.0))
    {
        driveController->set(0.0);
        return;
    }

    azimuth *= -Constants::TICKS; //flip azimuth hardware configuration dependent

    double azimuthPosition = azimuthController->getPosition();
    azimuthError = std::fmod(azimuth - azimuthPosition, Constants::TICKS);

    //wrap ticks, so that it's between +-2048
    azimuthError = azimuthError > Constants::TICKS / 2.0? azimuthError - Constants::TICKS : azimuthError;

    //azimuthError = std::copysign(std::fmod(std::abs(azimuth - azimuthPosition), TICKS), azimuth - azimuthPosition);

    //minimize azimuth rotation, reversing drive if necessary
    
    inverted = std::fabs(azimuthError) > .25 * Constants::TICKS;
    frc::SmartDashboard::PutBoolean("Flipped: Azimuth" + Util::sstr(id), inverted);
    frc::SmartDashboard::PutNumber("Pre error" + Util::sstr(id), std::fabs(azimuthError));
    std::string flipped = "";

    if (inverted)
    {
        azimuthError -= std::copysign(0.5 * Constants::TICKS, azimuthError);
        drive = -drive;

        flipped = ": flipped ";
    }
    
    if (output_smartdashboard)
    {
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Timestamp_" + Util::sstr(id), frc::Timer::GetFPGATimestamp());
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Drive_" + Util::sstr(id) + " setpoint", drive);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " setpoint", azimuthPosition + azimuthError);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " position", azimuthPosition);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " error", azimuthError);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " zero", azimuth_zero);
        frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " abs position", getAzimuthAbsolutePosition());
        stream << std::fixed << std::setprecision(4) <<frc::Timer::GetFPGATimestamp() << ","<< id << ","<< azimuthPosition << ","<< azimuthError << ","<< azimuth << ","<< flipped << ",";
    }
    

    

    azimuthController->set(azimuthPosition + azimuthError);
    driveController->set(drive);
}

void Thirdcoast::Wheel::setAzimuthPosition(int position)
{
    azimuthController->set(position);
    //frc::SmartDashboard::PutNumber("Thirdcoast/Swerve/Azimuth_" + Util::sstr(id) + " position", azimuthController->GetSelectedSensorPosition(0));
}

void Thirdcoast::Wheel::disableAzimuth()
{
    azimuthController->setNeutralOutput();
}

void Thirdcoast::Wheel::setDriveMode(MotorControllerWrapper::DriveMode driveMode)
{
    //this->driveMode = driveMode;
}

void Thirdcoast::Wheel::stop()
{
    azimuthController->set(azimuthController->getPosition());
    driveController->set(0.0);
}

void Thirdcoast::Wheel::setAzimuthZero(int zero)
{
    azimuth_zero = zero;
    int azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    azimuthController->setSensorPosition(azimuthSetpoint);
}

int Thirdcoast::Wheel::getAzimuthAbsolutePosition()
{
    return azimuthController->getAbsPosition();
}

std::string Thirdcoast::Wheel::getString()
{
    std::string output = stream.str(); 
    stream = std::stringstream();
    return output;
}
