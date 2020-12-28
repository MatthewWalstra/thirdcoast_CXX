#include "thirdcoast/swerve/SwerveDriveConfig.h"

#include "thirdcoast/swerve/SwerveDrive.h"
#include "thirdcoast/swerve/Wheel.h"

#include "Constants.h"

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

std::shared_ptr<Thirdcoast::SwerveDrive> Thirdcoast::SwerveDriveConfig::configSwerve()
{
    //creates SwerveDrive
    Thirdcoast::SwerveDriveConfig config{};
    
    config.gyro = std::make_shared<AHRS>(frc::SPI::kMXP, 200);
    config.length = Constants::ROBOT_LENGTH;
    config.width = Constants::ROBOT_WIDTH;
    

    /* Adjust default motor controller configs here before calling getWheels() */
    //EX. config.azimuthController.slot0.kP = 3.0;

    config.wheels = getWheels();
    return std::make_shared<Thirdcoast::SwerveDrive>(config);
}

std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> Thirdcoast::SwerveDriveConfig::getWheels()
{

    // create wheel array
    std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> wheels;

    for (int i = 0; i < Thirdcoast::SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        //configure azimuthTalon
        std::shared_ptr<MotorControllerWrapper> azimuth;
        
        switch (azimuthConfig.azimuthController)
        {
            case MotorControllerConfig::AzimuthMotorController::VICTOR_SPX:
                std::cout <<"Selected VictorSPX for azimuth" << std::endl;
                azimuth = std::make_shared<VictorSPXWrapper>(azimuthConfig, i);
                break;
            case MotorControllerConfig::AzimuthMotorController::TALON_SRX:
                std::cout <<"Selected TalonSRX for azimuth" << std::endl;
                azimuth = std::make_shared<TalonSRXWrapper>(azimuthConfig, i);
                break;
            case MotorControllerConfig::AzimuthMotorController::SPARK_MAX:
                std::cout <<"Selected SparkMax for azimuth" << std::endl;
                azimuth = std::make_shared<SparkMaxWrapper>(azimuthConfig, i);
                break;
            default:
                std::cout <<"Invalid selection for azimuth, defaulting to TalonSRX" << std::endl;
                azimuth = std::make_shared<TalonSRXWrapper>(azimuthConfig, i);
                break;
        }
        
        //configure DriveSparkMax
        std::shared_ptr<MotorControllerWrapper> drive;
        
        switch (driveConfig.driveController)
        {
            case MotorControllerConfig::DriveMotorController::SPARK_MAX:
                std::cout <<"Selected SparkMax for drive" << std::endl;
                drive = std::make_shared<SparkMaxWrapper>(driveConfig, i + 10);
                break;
            case MotorControllerConfig::DriveMotorController::TALON_FX:
                std::cout <<"Selected TalonFX for drive" << std::endl;
                drive = std::make_shared<TalonFXWrapper>(driveConfig, i + 10);
                break;
            default:
                std::cout <<"Invalid Drive Controller, defaulting to SparkMax" << std::endl;
                drive = std::make_shared<SparkMaxWrapper>(driveConfig, i + 10);
                break;
        }
        
        std::shared_ptr<Thirdcoast::Wheel> wheel = std::make_shared<Thirdcoast::Wheel>(azimuth, drive, Constants::DRIVE_SETPOINT_MAX, i);
        wheels[i] = wheel;
    }

    return wheels;
}