#include "gtest/gtest.h"

#include "thirdcoast/swerve/SwerveDrive.h"

#include "Constants.h"

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

#include <memory>
#include <iostream>
//using namespace std;

class SwerveMathTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    std::shared_ptr<Thirdcoast::SwerveDrive> drive;


    std::shared_ptr<Thirdcoast::SwerveDrive> configSwerve()
    {
        Thirdcoast::SwerveDriveConfig config{};
        config.wheels = getWheels();
        config.gyro = std::make_shared<AHRS>(frc::SPI::kMXP, 200);
        config.length = Constants::ROBOT_LENGTH;
        config.width = Constants::ROBOT_WIDTH;

        return std::make_shared<Thirdcoast::SwerveDrive>(config);
    }

    std::array<std::shared_ptr<Thirdcoast::Wheel>, Thirdcoast::SwerveDriveConfig::WHEEL_COUNT> getWheels()
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

            std::shared_ptr<Thirdcoast::Wheel> wheel = std::make_shared<Thirdcoast::Wheel>(azimuthTalon, driveSparkMax, Constants::DRIVE_SETPOINT_MAX, i);
            wheels[i] = wheel;
            //std::cout<<"created wheel "<< i <<azimuthTalon->GetSensorCollection().GetPulseWidthPosition() <<std::endl;
        }

        return wheels;
    }

    virtual void SetUp()
    {
        //drive = configSwerve();
    }
    virtual void TearDown(){}


};

TEST_F(SwerveMathTest, Testdrive)
{
    //drive->drive(.7,.7, 0.0);
    //EXPECT_NEAR(.989949, drive->getWS()[0], kEpsilon);
    EXPECT_TRUE(true);
    //drive->generateTestCases();
}

TEST_F(SwerveMathTest, TestWheel)
{
    //Azimuth -.25 to -.5 and .25 to .5: inverted = true
    //Azimuth -.25 to 0 and .25 to 0: inverted = false
    EXPECT_FALSE(false);
    
    
    std::shared_ptr<Thirdcoast::Wheel> wheel = getWheels().at(0);

    wheel->set(.2, .7, false);

    EXPECT_FALSE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), -.2 * 4096, kEpsilon);

    wheel->set(-.2, .7, false);

    EXPECT_FALSE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), .2 * 4096, kEpsilon);

    wheel->set(0.0, .7, false);

    EXPECT_FALSE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), 0.0, kEpsilon);

    wheel->set(.25, .7, false);

    EXPECT_FALSE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), -.25 * 4096, kEpsilon);

    wheel->set(-.25, .7, false);

    EXPECT_FALSE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), .25 * 4096, kEpsilon);

    wheel->set(-.251, .7, false);

    EXPECT_TRUE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), -.249 * 4096, kEpsilon);

    wheel->set(.251, .7, false);

    EXPECT_TRUE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), .249 * 4096, kEpsilon);

    wheel->set(-.5, .7, false);

    EXPECT_TRUE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), 0.0, kEpsilon);

    wheel->set(-.4, .7, false);

    EXPECT_TRUE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), -.1 * 4096, kEpsilon);

    wheel->set(.4, .7, false);

    EXPECT_TRUE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), .1 * 4096, kEpsilon);

    wheel->set(.23, .0000002, false);

    EXPECT_TRUE(wheel->isInverted());
    EXPECT_NEAR(wheel->getAzimuthError(), .1 * 4096, kEpsilon);

}