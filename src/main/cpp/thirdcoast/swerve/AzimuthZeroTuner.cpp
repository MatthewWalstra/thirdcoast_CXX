/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "thirdcoast/swerve/AzimuthZeroTuner.h"

Thirdcoast::AzimuthZeroTuner::AzimuthZeroTuner() 
{
    frc::SmartDashboard::PutBoolean("Thirdcoast/Tuning/Enable Azimuth Zeroing", false);
    frc::SmartDashboard::PutNumber("Thirdcoast/Tuning/Azimuth Setpoint", 0);
    frc::SmartDashboard::PutBoolean("Thirdcoast/Tuning/Save Azimuth Value", false);


    mWheelChooser.SetDefaultOption("FRONT_LEFT (0)", Wheels::FRONT_LEFT);
    mWheelChooser.AddOption("FRONT_RIGHT (1)", Wheels::FRONT_RIGHT);
    mWheelChooser.AddOption("BACK_LEFT (2)", Wheels::BACK_LEFT);
    mWheelChooser.AddOption("BACK_RIGHT (3)", Wheels::BACK_RIGHT);
    frc::SmartDashboard::PutData("Thirdcoast/Tuning/Wheel Selector", &mWheelChooser);

    frc::Shuffleboard::GetTab("Thirdcoast Tuner").GetLayout("Azimuth Zero Tuner", frc::BuiltInLayouts::kList).Add(mWheelChooser);

    //const AzimuthZeroTuner tuner;
    //std::function<bool()> enable = getEnable();
    //frc::Shuffleboard::GetTab("Thirdcoast Tuner").GetLayout("Azimuth Zero Tuner", frc::BuiltInLayouts::kList).AddBoolean("Enable Azimuth Zeroing", enable);
}

void Thirdcoast::AzimuthZeroTuner::update(std::shared_ptr<Thirdcoast::SwerveDrive> drive)
{
    bool enable = frc::SmartDashboard::GetBoolean("Thirdcoast/Tuning/Enable Azimuth Zeroing", false);
    double setpoint = frc::SmartDashboard::GetNumber("Thirdcoast/Tuning/Azimuth Setpoint", 0);
    bool save = frc::SmartDashboard::GetBoolean("Thirdcoast/Tuning/Save Azimuth Value", false);

    setpoint = Util::limit(setpoint, 0, 4096);
    if (enable)
    {
        drive->getWheels().at(mWheelChooser.GetSelected())->setAzimuthPosition(setpoint);
    }

    if (save)
    {
        drive->saveAzimuthPositions();
    }

    i++;
    frc::SmartDashboard::PutNumber("Thirdcoast/Tuning/updates", i);
}
