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
//    frc::SmartDashboard::PutNumber("Nullptr TalonSRX", drive->wheels.at)
    drive->getGyro()->ZeroYaw();

    //drive->drive(.7,.5, 0.4);
    //frc::SmartDashboard::PutNumber("TestDrive:", drive->getWS()[0]);
    //frc::SmartDashboard::PutNumber("TestAzimuth:", drive->getWA()[0]);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() 
{
    drive->zeroAzimuthEncoders();
    drive->setDriveMode(Thirdcoast::Wheel::TELEOP);
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

    frc::SmartDashboard::PutNumber("Forward (Pre Limit)", forward);
    frc::SmartDashboard::PutNumber("Strafe (Pre Limit)", strafe);
    frc::SmartDashboard::PutNumber("Yaw (Pre Limit)", yaw);

    frc::SmartDashboard::PutNumber("Forward (Delta expo)", getForward() - forward);
    frc::SmartDashboard::PutNumber("Strafe (Delta expo)", getStrafe() - strafe);
    frc::SmartDashboard::PutNumber("Yaw (Delta expo)", getYaw() - yaw);

    forward = Util::limit(forward, Constants::MAX_FWD_STR);
    strafe = Util::limit(strafe, Constants::MAX_FWD_STR);
    yaw = Util::limit(yaw, Constants::MAX_YAW);

    frc::SmartDashboard::PutNumber("Forward", forward);
    frc::SmartDashboard::PutNumber("Strafe", strafe);
    frc::SmartDashboard::PutNumber("Yaw", yaw);
    std::array<double, 2> output = vectorLimit.apply(forward, strafe);

    drive->drive(output[0], output[1], yaw);
    
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

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
    Thirdcoast::SwerveDriveConfig config{};
    config.wheels = getWheels();
    config.gyro = std::make_shared<AHRS>(frc::SPI::kMXP, 200);
    config.length = Constants::ROBOT_LENGTH;
    config.width = Constants::ROBOT_WIDTH;

    return std::make_shared<Thirdcoast::SwerveDrive>(config);
}

std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> Robot::getWheels()
{

    // configure Azimuth Talons
    //std::cout<<"Started Talon Config"<<std::endl;
    TalonSRXConfiguration azimuthConfig{};
    azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Relative;
    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;
    azimuthConfig.slot0.kP = 10.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 100.0;
    azimuthConfig.slot0.kF = 0.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.motionAcceleration = 10000;
    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.velocityMeasurementWindow = 64;
    azimuthConfig.voltageCompSaturation = 12;

    //std::cout<<"Creating Array"<<std::endl;
    // create wheel array
    std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> wheels;

    for (int i = 0; i < Thirdcoast::SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        //std::cout<<"Creating Talon"<<std::endl;
        std::shared_ptr<TalonSRX> azimuthTalon = std::make_shared<TalonSRX>(i);
        azimuthTalon->ConfigAllSettings(azimuthConfig);
        azimuthTalon->EnableCurrentLimit(true);
        azimuthTalon->EnableVoltageCompensation(true);
        azimuthTalon->SetNeutralMode(NeutralMode::Coast);

        //std::cout<<"Creating Spark Max"<<std::endl;
        std::shared_ptr<rev::CANSparkMax> driveSparkMax = std::make_shared<rev::CANSparkMax>(i + 10, rev::CANSparkMax::MotorType::kBrushless);
        driveSparkMax->SetSmartCurrentLimit(40);
        driveSparkMax->GetPIDController().SetP(.05);
        driveSparkMax->GetPIDController().SetI(.0005);
        driveSparkMax->GetPIDController().SetD(.0);
        driveSparkMax->GetPIDController().SetFF(.032);
        driveSparkMax->GetPIDController().SetIZone(1000.0);
        driveSparkMax->GetPIDController().SetIAccum(150000);
        driveSparkMax->EnableVoltageCompensation(12.0);
        driveSparkMax->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        std::shared_ptr<Thirdcoast::Wheel> wheel = std::make_shared<Thirdcoast::Wheel>(azimuthTalon, driveSparkMax, Constants::DRIVE_SETPOINT_MAX);
        wheels[i] = wheel;
        //std::cout<<"created wheel "<< i <<azimuthTalon->GetSensorCollection().GetPulseWidthPosition() <<std::endl;
    }

    return wheels;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
