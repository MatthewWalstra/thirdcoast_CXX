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
    drive = configSwerve();

    drive->getGyro()->ZeroYaw();

}

void Robot::RobotPeriodic() 
{
    
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() 
{
    drive->zeroAzimuthEncoders();
    drive->setDriveMode(Thirdcoast::Wheel::TELEOP);
    drive->setFieldOriented(true);
}

void Robot::TeleopPeriodic() 
{
    drive->outputSmartDashboard();
    
    frc::SmartDashboard::PutNumber("Forward (Pre Expo)", getForward());
    frc::SmartDashboard::PutNumber("Strafe (Pre Expo)", getStrafe());
    frc::SmartDashboard::PutNumber("Yaw (Pre Expo)", getYaw());
    
    /*
    double yaw = getYaw();
    double forward = getForward();
    double strafe = -getStrafe();
    drive->drive(forward, strafe, yaw);
    */

    
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

    //frc::SmartDashboard::PutNumber("Output Yaw", yaw);
    //frc::SmartDashboard::PutNumber("Output Forward", output[0]);
    //frc::SmartDashboard::PutNumber("Output Strafe", output[1]);

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
    finished_circle = false;
    finished_forward = false;
    j = 0.0;
}
void Robot::TestPeriodic() 
{
    /*
    double timestamp = frc::Timer::GetFPGATimestamp(); 

    if (timestamp - prev_timestamp > 1.0)
    {
        setpoint = prev_setpoint + .25 * (1 + i % 2) * (i % 4 == 1 || i % 4 == 2? -1.0:1.0);
        std::cout<<setpoint<<std::endl;
        prev_timestamp = timestamp;

        //add noise +- .05
        for (auto wheel: drive->getWheels())
        {
            wheel->set(setpoint + std::rand() % 100 /1000 - .05, .25, false);
        }
        
        prev_setpoint = setpoint;
        i++;
    }
    */

   std::shared_ptr<Thirdcoast::Wheel> wheel = drive->getWheels().at(1);
    /*
   if (!finished_circle)
   {
       double azimuth = std::atan2(std::cos(j), std::sin(j)) * .5 / M_PI;
       wheel->set(azimuth, .2, false);
       j += circleStepsize;

       finished_circle = j > 2.0 * M_PI? true:false;

       if (finished_circle)
       {
           j = 0.0;
           std::cout << "Finished Circle" << std::endl;
       }

       mWheelOutput << wheel->getString();
   }

   if (!finished_forward && finished_circle)
   {
       double drive = std::cos(j);
       wheel->set(0.0, drive, false);

       j += forwardStepsize;

       finished_forward = j > 2.0 * M_PI? true:false;

       if (finished_forward)
       {
           std::cout << "Finished forward and backwards" << std::endl;
       }

       mWheelOutput << wheel->getString();
   }
    
    //drive->getWheels()[2]->set(setpoint, .25, false);
    */
    
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

std::shared_ptr<Thirdcoast::SwerveDrive> Robot::configSwerve()
{
    //creates SwerveDrive
    Thirdcoast::SwerveDriveConfig config{};
    config.wheels = getWheels();
    config.gyro = std::make_shared<AHRS>(frc::SPI::kMXP, 200);
    config.length = Constants::ROBOT_LENGTH;
    config.width = Constants::ROBOT_WIDTH;

    return std::make_shared<Thirdcoast::SwerveDrive>(config);
}

std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> Robot::getWheels()
{

    // create configuration for Azimuth Talons
    TalonSRXConfiguration azimuthConfig{};
    azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Absolute;
    azimuthConfig.continuousCurrentLimit = 15.0;
    azimuthConfig.peakCurrentDuration = 1;
    azimuthConfig.peakCurrentLimit = 30.0;
    azimuthConfig.slot0.kP = 2.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 30.0;
    azimuthConfig.slot0.kF = 0.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.motionAcceleration = 10000;
    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.velocityMeasurementWindow = 64;
    azimuthConfig.voltageCompSaturation = 12;

    // create wheel array
    std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> wheels;

    for (int i = 0; i < Thirdcoast::SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        //configure azimuthTalon
        std::shared_ptr<TalonSRX> azimuthTalon = std::make_shared<TalonSRX>(i);
        azimuthTalon->ConfigAllSettings(azimuthConfig);
        azimuthTalon->EnableCurrentLimit(true);
        azimuthTalon->EnableVoltageCompensation(true);
        azimuthTalon->SetNeutralMode(NeutralMode::Coast);

        //configure DriveSparkMax
        std::shared_ptr<rev::CANSparkMax> driveSparkMax = std::make_shared<rev::CANSparkMax>(i + 10, rev::CANSparkMax::MotorType::kBrushless);
        driveSparkMax->SetSmartCurrentLimit(80);
        driveSparkMax->GetPIDController().SetP(.05);
        driveSparkMax->GetPIDController().SetI(.0005);
        driveSparkMax->GetPIDController().SetD(.0);
        driveSparkMax->GetPIDController().SetFF(.032);
        driveSparkMax->GetPIDController().SetIZone(1000.0);
        driveSparkMax->GetPIDController().SetIAccum(150000);
        driveSparkMax->EnableVoltageCompensation(12.0);
        driveSparkMax->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        std::shared_ptr<Thirdcoast::Wheel> wheel = std::make_shared<Thirdcoast::Wheel>(azimuthTalon, driveSparkMax, Constants::DRIVE_SETPOINT_MAX, i);
        wheels[i] = wheel;
    }

    return wheels;
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
