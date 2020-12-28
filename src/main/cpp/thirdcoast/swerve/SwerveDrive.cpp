/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "thirdcoast/swerve/SwerveDrive.h"
#include "Constants.h"
//using namespace Thirdcoast;

Thirdcoast::SwerveDrive::SwerveDrive(SwerveDriveConfig config) 
{

    //Calculate Width and Length components
    gyro = config.gyro;
    wheels = config.wheels;
    enableSmartDashboardOutput = config.enableSmartDashboardOutput;
    double length = config.length;
    double width = config.width;
    double radius = std::hypot(length, width);

    kLengthComponent = length / radius;
    kWidthComponent = width /radius;

    //configure field oriented driving
    setFieldOriented(gyro != NULL && gyro->IsConnected());

    if (fieldOriented)
    {
        double robotPeriod = config.robotPeriod;
        double gyroRateCoeff = config.gyroRateCoeff;
        int rate = gyro->GetActualUpdateRate();
        double gyroPeriod = 1.0 / rate;

        kGyroRateCorrection = (robotPeriod / gyroPeriod) * gyroRateCoeff;
    } else
    {
        kGyroRateCorrection = 0.0;
    }   
}

std::string Thirdcoast::SwerveDrive::getPreferenceKeyForWheel(int wheel)
{
    return "SwerveDrive/wheel." + Util::sstr(wheel);
}

void Thirdcoast::SwerveDrive::setDriveMode(Thirdcoast::MotorControllerWrapper::DriveMode driveMode)
{
    for (auto wheel : wheels)
    {
        wheel->setDriveMode(driveMode);
    }
}

void Thirdcoast::SwerveDrive::set(double azimuth, double drive)
{
    for (auto wheel : wheels)
    {
        wheel->set(azimuth, drive, enableSmartDashboardOutput);
    }
}


void Thirdcoast::SwerveDrive::drive(double forward, double strafe, double azimuth)
{
    // Use gyro for field-oriented drive. We use getAngle instead of getYaw to enable arbitrary
    // autonomous starting positions.
    if (fieldOriented)
    {
        double angle = gyro->GetFusedHeading();
        //double angle = gyro->GetAngle();
        angle += gyro->GetRate() * kGyroRateCorrection;
        angle = std::fmod(angle, 360.0);

        angle = Util::degToRads(angle);
        const double tmp = forward * std::cos(angle) + strafe * std::sin(angle);
        strafe = strafe * std::cos(angle) - forward * std::sin(angle);
        forward = tmp;
    }

    const double a = strafe - azimuth * kLengthComponent;
    const double b = strafe + azimuth * kLengthComponent;
    const double c = forward - azimuth * kWidthComponent;
    const double d = forward + azimuth * kWidthComponent;

    // wheel Speed
    ws.at(0) = std::hypot(b, d);
    ws.at(1) = std::hypot(b, c);
    ws.at(2) = std::hypot(a, d);
    ws.at(3) = std::hypot(a, c);

    // wheel azimuth
    wa.at(0) = std::atan2(b, d) * 0.5 / Constants::PI;
    wa.at(1) = std::atan2(b, c) * 0.5 / Constants::PI;
    wa.at(2) = std::atan2(a, d) * 0.5 / Constants::PI;
    wa.at(3) = std::atan2(a, c) * 0.5 / Constants::PI;

    // normalize
    const double maxWheelSpeed = std::max(std::max(ws.at(0), ws.at(1)), std::max(ws.at(2), ws.at(3)));
    if (maxWheelSpeed > 1.0)
    {
        for (int i = 0; i < SwerveDriveConfig::WHEEL_COUNT; i++)
        {
            ws[i] /= maxWheelSpeed;
        }
    }

    for(int i = 0; i < SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        wheels.at(i)->set(wa.at(i), ws.at(i), !(Util::epsilonEquals(forward, 0.0, .05) && Util::epsilonEquals(strafe, 0.0, .05)));
    }
   
}

void Thirdcoast::SwerveDrive::stop()
{
    for (auto wheel : wheels)
    {
        wheel->stop();
    }
}

void Thirdcoast::SwerveDrive::saveAzimuthPositions()
{
    saveAzimuthPositions(frc::Preferences::GetInstance());
}

void Thirdcoast::SwerveDrive::saveAzimuthPositions(frc::Preferences *prefs)
{
    for (int i = 0; i < SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        int position = wheels.at(i)->getAzimuthAbsolutePosition();
        prefs->PutInt(getPreferenceKeyForWheel(i), position);
    }
}

void Thirdcoast::SwerveDrive::zeroAzimuthEncoders()
{
    zeroAzimuthEncoders(frc::Preferences::GetInstance());
}

void Thirdcoast::SwerveDrive::zeroAzimuthEncoders(frc::Preferences *prefs)
{
    for (int i = 0; i < SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        int position = ZeroPositions[i];//prefs->GetInt(getPreferenceKeyForWheel(i), DEFAULT_ABSOLUTE_AZIMUTH_OFFSET);
        
        wheels.at(i)->setAzimuthZero(position);
    }
}

void Thirdcoast::SwerveDrive::outputSmartDashboard()
{
    for (int i = 0; i < SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        frc::SmartDashboard::PutNumber("Azimuth Setpoint: " + Util::sstr(i), wa[i]);
        frc::SmartDashboard::PutNumber("Drive Setpoint: " + Util::sstr(i), ws[i]);
    }
}