/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>

void Robot::RobotInit() 
{
    Thirdcoast::SwerveDriveConfig config{};
    drive = config.configSwerve();

    drive->getGyro()->ZeroYaw();

    //nanolog::initialize(nanolog::GuaranteedLogger(), "", "nanolog", 100);

}

void Robot::RobotPeriodic() 
{
    
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() 
{
    drive->zeroAzimuthEncoders();
    drive->setDriveMode(Thirdcoast::MotorControllerWrapper::DriveMode::TELEOP);
    drive->setFieldOriented(true);
}

void Robot::TeleopPeriodic() 
{
    drive->outputSmartDashboard();
    
    frc::SmartDashboard::PutNumber("Forward (Pre Expo)", getForward());
    frc::SmartDashboard::PutNumber("Strafe (Pre Expo)", getStrafe());
    frc::SmartDashboard::PutNumber("Yaw (Pre Expo)", getYaw());
    
    double yaw = yawExpo.apply(getYaw());
    double forward = driveExpo.apply(getForward());
    double strafe = driveExpo.apply(getStrafe());
    
    

    frc::SmartDashboard::PutNumber("Yaw", yaw);
    frc::SmartDashboard::PutNumber("Forward", forward);
    frc::SmartDashboard::PutNumber("Strafe", strafe);

    forward = Util::limit(forward, Constants::MAX_FWD_STR);
    strafe = Util::limit(strafe, Constants::MAX_FWD_STR);
    yaw = Util::limit(yaw, Constants::MAX_YAW);

    std::array<double, 2> output = vectorLimit.apply(forward, strafe);

    drive->drive(output[0], output[1], yaw);
    
    bool azimuth_save = getAzimuthSaveTrigger();
    bool azimuth_reset = getAzimuthResetTrigger();
    if (azimuth_save && azimuth_save != prev_save)
    {
        //save current position to each azimuth zero position
        drive->saveAzimuthPositions();
        std::cout<<"Saving Azimuth Positions!"<<std::endl;
    }
    else if (azimuth_reset && azimuth_reset != prev_reset)
    {
        //reset azimuths to their zero position
        drive->resetGyro();
        std::cout<<"Reseting Gyro!"<<std::endl;
    }

    prev_save = azimuth_save;
    prev_reset = azimuth_reset;
    
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() 
{
    
}
void Robot::TestPeriodic() 
{
    
}

double Robot::getForward()
{
    return controller.getJoystick(ControlBoard::XBoxController::LEFT, ControlBoard::XBoxController::y);
}

double Robot::getStrafe()
{
    return controller.getJoystick(ControlBoard::XBoxController::LEFT, ControlBoard::XBoxController::x);
}

double Robot::getYaw()
{
    return controller.getJoystick(ControlBoard::XBoxController::RIGHT, ControlBoard::XBoxController::x);
}

//Buttons
//A && B && X && Y

bool Robot::getAzimuthSaveTrigger()
{   
    return controller.getButton(1) && controller.getButton(2) && controller.getButton(3) && controller.getButton(4); 
}

//Start button

bool Robot::getAzimuthResetTrigger()
{
    return controller.getButton(8); 
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
