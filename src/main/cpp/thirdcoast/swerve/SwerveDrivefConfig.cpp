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
    config.wheels = getWheels();
    config.gyro = std::make_shared<AHRS>(frc::SPI::kMXP, 200);
    config.length = Constants::ROBOT_LENGTH;
    config.width = Constants::ROBOT_WIDTH;

    return std::make_shared<Thirdcoast::SwerveDrive>(config);
}

std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> Thirdcoast::SwerveDriveConfig::getWheels()
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