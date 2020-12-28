/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "thirdcoast/swerve/SwerveDriveConfig.h"

#include <frc/DriverStation.h>

#include <frc/PWM.h>
#include <frc/controller/PIDController.h>

namespace Thirdcoast {

class MotorControllerWrapper {
  
 public:
  bool isAzimuth;
  int id;
  int slot = 0;
  enum class DriveMode {
    OPEN_LOOP,
    CLOSED_LOOP,
    TELEOP, //Same as open loop, but option to add extra configurations
    TRAJECTORY, //Same as closed loop
    AZIMUTH, //Only azimuths
  } driveMode = DriveMode::TELEOP;
  MotorControllerWrapper(){}

  /* sets motor controller output and mode */
  virtual void set(double output)
  {
    frc::DriverStation::ReportError("Please override set() in an inherited class");
  }
  
  /* sets sensor position */
  virtual void setSensorPosition(double position)
  {
    frc::DriverStation::ReportError("Please override setSensorPosition() in an inherited class");
  }
  
  /* disables motor output */
  virtual void setNeutralOutput()
  {
    //TODO: Test not making inherited, but using set method here.
    frc::DriverStation::ReportError("Please override setNeutralOutput() in an inherited class");
  }
  
  /* returns motor controller data */
  virtual double getPosition()
  {
    frc::DriverStation::ReportError("Please override getPosition() in an inherited class");
    return 0.0;
  }
  virtual double getAbsPosition()
  {
    frc::DriverStation::ReportError("Please override getAbsPosition() in an inherited class");
    return 0.0;
  }
  virtual double getVelocity()
  {
    frc::DriverStation::ReportError("Please override getVelocity() in an inherited class");
    return 0.0;
  }
  virtual double getOutput()
  {
    frc::DriverStation::ReportError("Please override getOutput() in an inherited class");
    return 0.0;
  }
  virtual double getCurrent()
  {
    frc::DriverStation::ReportError("Please override getCurrent() in an inherited class");
    return 0.0;
  }

  /* returns motor controller data and output as a string */
  virtual std::string getString()
  {
    frc::DriverStation::ReportError("Please override getString() in an inherited class");
    return "";
  }
};

class SparkMaxWrapper: public MotorControllerWrapper {
  std::shared_ptr<rev::CANSparkMax> sparkMax;
  std::shared_ptr<rev::CANPIDController> pidController;
  std::shared_ptr<frc::PWM> pwm = NULL;
  std::shared_ptr<frc2::PIDController> pid = NULL;
  std::shared_ptr<rev::CANEncoder> encoder = NULL;
  
  // IDS from 20-23, so it does not conflict with drive base
  std::shared_ptr<CANCoder> canCoder = NULL;
  
 
  /* convert enum classes to specific motor controller types */
  //Rev
  rev::CANSparkMax::MotorType getRevMotorType(MotorControllerConfig::MotorType type);
  rev::ControlType getRevControlType();
  /* returns CANSensor or NULL for integrated encoder */
  rev::CANEncoder::EncoderType getRevFeedbackDevice(MotorControllerConfig::FeedbackSensor sensor, MotorControllerConfig::MotorType type);
  rev::CANSparkMax::IdleMode getRevIdleMode(MotorControllerConfig::NeutralMode mode);
  bool handleREVCanError(int id, rev::CANError error, const std::string &method_name);
  
 public:
  SparkMaxWrapper(MotorControllerConfig config, int id);

  /* sets motor controller output and mode */
  void set(double output) override;

  /* sets sensor position */
  void setSensorPosition(double position) override;

  /* disables motor output */
  void setNeutralOutput() override;
  
  /* returns motor controller data */
  double getPosition() override;
  double getAbsPosition() override;
  double getVelocity() override;
  double getOutput() override;
  double getCurrent() override;

  /* returns motor controller data and output as a string */
  std::string getString() override;
  

};

class TalonBaseWrapper: public MotorControllerWrapper {
  //CTRE
 public: 
  motorcontrol::FeedbackDevice getCTREFeedbackDevice(MotorControllerConfig::FeedbackSensor sensor);
  motorcontrol::NeutralMode getCTRENeutralMode(MotorControllerConfig::NeutralMode mode);
  bool handleCTRECanError(int id, ctre::phoenix::ErrorCode error_code, const std::string &method_name);
  motorcontrol::ControlMode getCTREControlMode();
 
  TalonBaseWrapper(){}
};

class VictorSPXWrapper: public TalonBaseWrapper {
  std::shared_ptr<VictorSPX> victorSPX;
  int feedbackSensor = 2;
  // IDS 0-3 on RIO
  std::shared_ptr<frc::PWM> pwm = NULL;

  // Only supports PID (no Feedforward) on slot0
  std::shared_ptr<frc2::PIDController> pid = NULL;
  
  // IDS from 20-23, so it does not conflict with drive base
  std::shared_ptr<CANCoder> canCoder = NULL;
 public:
  VictorSPXWrapper(MotorControllerConfig config, int id);

  /* sets motor controller output and mode */
  void set(double output) override;

  /* sets sensor position */
  void setSensorPosition(double position) override;

  /* disables motor output */
  void setNeutralOutput() override;
  
  /* returns motor controller data */
  double getPosition() override;
  double getAbsPosition() override;
  double getVelocity() override;
  double getOutput() override;
  double getCurrent() override;

  /* returns motor controller data and output as a string */
  std::string getString() override;
  

};

class TalonSRXWrapper: public TalonBaseWrapper {
  std::shared_ptr<TalonSRX> talonSRX;
 public:
  TalonSRXWrapper(MotorControllerConfig config, int id);

  /* sets motor controller output and mode */
  void set(double output) override;

  /* sets sensor position */
  void setSensorPosition(double position) override;

  /* disables motor output */
  void setNeutralOutput() override;
  
  /* returns motor controller data */
  double getPosition() override;
  double getAbsPosition() override;
  double getVelocity() override;
  double getOutput() override;
  double getCurrent() override;

  /* returns motor controller data and output as a string */
  std::string getString() override;
  

};

class TalonFXWrapper: public TalonBaseWrapper {
  std::shared_ptr<TalonFX> talonFX;
 public:
  TalonFXWrapper(MotorControllerConfig config, int id);

  /* sets motor controller output and mode */
  void set(double output) override;

  /* sets sensor position */
  void setSensorPosition(double position) override;

  /* disables motor output */
  void setNeutralOutput() override;
  
  /* returns motor controller data */
  double getPosition() override;
  double getAbsPosition() override;
  double getVelocity() override;
  double getOutput() override;
  double getCurrent() override;

  /* returns motor controller data and output as a string */
  std::string getString() override;
  

};

} //Thirdcoast namespace