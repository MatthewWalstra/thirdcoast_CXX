/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <AHRS.h>
#include <frc/TimedRobot.h>

#include <vector>
#include <memory>

namespace Thirdcoast {

//forward declare swerve drive and wheel classes to avoid circular dependencies
class SwerveDrive;
class Wheel;

struct PIDFSlot
{
  double kP, kI, kD, kF, kIZone, kAllowableError, kMaxIAccum = 0.0; 
  
  PIDFSlot(bool azimuth)
  {
    //initialize default PIDF values based on whether it's an azimuth or drive slot
    if (azimuth)
    {
      //Azimuth: TODO - different for spark max and talons?
      kP = 2.0;
      kI = 0.0;
      kD = 30.0;
      kF = 0.0;
      kIZone = 0.0;
      kAllowableError = 0.0;
      kMaxIAccum = 0.0;
    } else
    {
      //Drive: TODO - get better defaults
      kP = 0.05;
      kI = 0.0005;
      kD = 0.0;
      kF = 0.032;
      kIZone = 1000.0;
      kAllowableError = 0.0;
      kMaxIAccum = 150000.0;
    }
    
  }
    

};



class MotorControllerConfig
{
  private:
    
  public:
    bool isAzimuth = true;
    int feedbackChannel = -1;
    /** Supported Feedback Sensors **/
    enum FeedbackSensor
    {
      CTRE_MAG_ENCODER,
      CAN_CODER,
      THRIFTY_CODER,
      INTEGRATED_SENSOR
    } feedbackSensor = CTRE_MAG_ENCODER;

    /** Supported Drive Motor Controllers **/
    enum class DriveMotorController
    {
      SPARK_MAX,
      TALON_FX
    } driveController = DriveMotorController::SPARK_MAX;

    /** Supported Azimuth Motor Controllers **/
    enum class AzimuthMotorController
    {
      VICTOR_SPX,
      TALON_SRX,
      SPARK_MAX
    } azimuthController = AzimuthMotorController::TALON_SRX;

    /** Motor Controller Neutral Modes **/
    enum NeutralMode
    {
      BRAKE,
      COAST
    } neutralMode = BRAKE;

    /** update ONLY for Spark Max azimuth controllers with a brushed motor **/
    enum MotorType
    {
      BRUSHED,
      BRUSHLESS
    } motorType = BRUSHLESS;

    /**
     * peakCurrentLimit -> Talon: peakCurrentLimit, SparkMax: secondaryCurrentLimit
     * continuousCurrentLimit -> Talon: continuousCurrentLimit, SparkMax: smartCurrentLimit
     * 
     * motionAcceleration -> Talon: motionAcceleration, SparkMax.PIDController: smartMotionAcceleration
     * motionCruiseVelocity -> Talon: motionCruiseVelocity, SparkMax.PIDController: smartMotionMaxVelocity
     * 
     * voltageCompensation -> Talon: voltageCompSaturation, SparkMax: voltageCompensation
     * 
    **/
    double peakCurrentLimit, continuousCurrentLimit, motionAcceleration, motionCruiseVelocity, voltageCompensation = 0.0;
    
    MotorControllerConfig(AzimuthMotorController azimuth, FeedbackSensor sensor): isAzimuth(true)
    {
      //update values for azimuth
      azimuthController = azimuth;
      feedbackSensor = sensor;
      neutralMode = COAST;
      peakCurrentLimit = 30.0;
      continuousCurrentLimit = 15.0;
      motionAcceleration = 10000.0;
      motionCruiseVelocity = 800.0;
      voltageCompensation = 12.0;

    }

    MotorControllerConfig(DriveMotorController drive, FeedbackSensor sensor): isAzimuth(false)
    {
      //update values for azimuth
      driveController = drive;
      feedbackSensor = sensor;
      neutralMode = BRAKE;

      peakCurrentLimit = 80.0;
      continuousCurrentLimit = 60.0;
    }
    
    //4 slots for each motor controller -- might need to be moved inside constructor
    PIDFSlot slot0 = PIDFSlot(isAzimuth);
    PIDFSlot slot1 = PIDFSlot(isAzimuth);
    PIDFSlot slot2 = PIDFSlot(isAzimuth);
    PIDFSlot slot3 = PIDFSlot(isAzimuth);
    

};

class SwerveDriveConfig {
 public:
  static const int WHEEL_COUNT = 4;
  
  SwerveDriveConfig(){}

  //creates Thirdcoast::SwerveDrive object
  std::shared_ptr<Thirdcoast::SwerveDrive> configSwerve();

  //creates wheels for SwerveDrive
  std::array<std::shared_ptr<Wheel>, WHEEL_COUNT> getWheels();

  
  /**
   * NavX gyro connected to MXP SPI port, used for field-oriented driving. If null, field-oriented
   * driving is disabled.
   */
  std::shared_ptr<AHRS> gyro;

  /** Initialize with four initialized wheels, in order from wheel 0 to wheel 3. */
  std::array<std::shared_ptr<Wheel>, WHEEL_COUNT> wheels;

  /** Wheel base length from front to rear of robot. */
  double length = 1.0;

  /** Wheel base width from left to right of robot. */
  double width = 1.0;

  /**
   * Robot period is the {@code TimedRobot} period in seconds, defaults to {@code
   * TimedRobot.kDefaultPeriod}.
   */
  double robotPeriod = frc::TimedRobot::kDefaultPeriod.value();

  /** Factor to correct gyro lag when simultaneously applying azimuth and drive. **/
  double gyroRateCoeff = 0.0;

  /**System Dependent: switch if wheels form an x rather than a circle when only applying yaw to rotate in place (right x stick) **/
  bool invertError = true;

  /** Enables Smartdashboard debugging info **/
  bool enableSmartDashboardOutput = true;

  /** Configs for the azimuth and drive motor controllers **/
  MotorControllerConfig azimuthConfig{MotorControllerConfig::AzimuthMotorController::TALON_SRX, MotorControllerConfig::FeedbackSensor::CTRE_MAG_ENCODER};
  MotorControllerConfig driveConfig{MotorControllerConfig::DriveMotorController::SPARK_MAX, MotorControllerConfig::FeedbackSensor::INTEGRATED_SENSOR};

};


}