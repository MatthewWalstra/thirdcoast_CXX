/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "thirdcoast/swerve/MotorControllerWrapper.h"

#include "thirdcoast/nanoLog/NanoLog.hpp"

#include "thirdcoast/util/Util.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace Thirdcoast;

rev::CANSparkMax::MotorType SparkMaxWrapper::getRevMotorType(MotorControllerConfig::MotorType type)
{
    switch (type)
    {
        case MotorControllerConfig::MotorType::BRUSHED:
            return rev::CANSparkMax::MotorType::kBrushed;
        case MotorControllerConfig::MotorType::BRUSHLESS:
            return rev::CANSparkMax::MotorType::kBrushless;
        default:
            frc::DriverStation::ReportError("Unexpected Motor Type: " + type);
            //LOG_WARN << "Unexpected Motor Type: " << type;
			return rev::CANSparkMax::MotorType::kBrushless;

    }
}

/* TODO fix once we know how sensors work with spark maxes (abs/quadrature and analog) */
rev::CANEncoder::EncoderType SparkMaxWrapper::getRevFeedbackDevice(MotorControllerConfig::FeedbackSensor sensor, MotorControllerConfig::MotorType type)
{
    if (!isAzimuth)
	{
		return rev::CANEncoder::EncoderType::kHallSensor;
	}
	
	switch (sensor)
    {
        case MotorControllerConfig::FeedbackSensor::INTEGRATED_SENSOR:
            if (type == MotorControllerConfig::BRUSHED)
            {
                frc::DriverStation::ReportError("Brushed Motor doesn't have an integrated Sensor");
				//LOG_WARN << "Brushed Motor doesn't have an integrated Sensor: " << type;
				return rev::CANEncoder::EncoderType::kNoSensor;
            }
            return rev::CANEncoder::EncoderType::kHallSensor;
        case MotorControllerConfig::FeedbackSensor::THRIFTY_CODER:
			return rev::CANEncoder::EncoderType::kNoSensor;
		case MotorControllerConfig::FeedbackSensor::CAN_CODER:
			return rev::CANEncoder::EncoderType::kNoSensor;
		case MotorControllerConfig::FeedbackSensor::CTRE_MAG_ENCODER:
			frc::DriverStation::ReportError("Unsupported Spark Max Sensor Type: " + sensor);
            //LOG_WARN << "Unsupported Spark Max Sensor Type: " << sensor;

			return rev::CANEncoder::EncoderType::kNoSensor;
		
		default:
            frc::DriverStation::ReportError("Unexpected Motor Type: " + sensor);
            //LOG_WARN << "Unexpected Sensor Type: " << sensor;
			return rev::CANEncoder::EncoderType::kHallSensor;
    }
}

rev::CANSparkMax::IdleMode SparkMaxWrapper::getRevIdleMode(MotorControllerConfig::NeutralMode mode)
{
    switch (mode)
    {
        case MotorControllerConfig::NeutralMode::BRAKE:
            return rev::CANSparkMax::IdleMode::kBrake;
        case MotorControllerConfig::NeutralMode::COAST:
            return rev::CANSparkMax::IdleMode::kCoast;
		default:
			frc::DriverStation::ReportError("Unexpected Idle Mode: " + Util::sstr(mode) + ", Returning Coast");
			//LOG_WARN << "Unexpected Idle Mode: " << mode;
			return rev::CANSparkMax::IdleMode::kCoast;
    }
}

bool SparkMaxWrapper::handleREVCanError(int id, rev::CANError error, const std::string &method_name)
{
    std::string error_name;

	switch(error)
	{
		case rev::CANError::kOk:
			return true;
		case rev::CANError::kError:
			error_name = "kError";
			break;
		case rev::CANError::kTimeout:
			error_name = "kTimeout";
			break;
		case rev::CANError::kNotImplmented:
			error_name = "kNotImplmented";
			break;
		case rev::CANError::kHALError:
			error_name = "kHALError";
			break;
		case rev::CANError::kCantFindFirmware:
			error_name = "kCantFindFirmware";
			break;
		case rev::CANError::kFirmwareTooOld:
			error_name = "kFirmwareTooOld";
			break;
		case rev::CANError::kFirmwareTooNew:
			error_name = "kFirmwareTooNew";
			break;
		case rev::CANError::kParamInvalidID:
			error_name = "kParamInvalidID";
			break;
		case rev::CANError::kParamMismatchType:
			error_name = "kParamMismatchType";
			break;
		case rev::CANError::kParamAccessMode:
			error_name = "kParamAccessMode";
			break;
		case rev::CANError::kParamInvalid:
			error_name = "kParamInvalid";
			break;
		case rev::CANError::kParamNotImplementedDeprecated:
			error_name = "kParamNotImplementedDeprecated";
			break;
		case rev::CANError::kFollowConfigMismatch:
			error_name = "kFollowConfigMismatch";
			break;
		case rev::CANError::kInvalid:
			error_name = "kInvalid";
			break;
		case rev::CANError::kSetpointOutOfRange:
			error_name = "kSetpointOutOfRange";
			break;
		default:
			{
				std::stringstream s;
				s << "Unknown Spark Max error " << static_cast<int>(error);
				error_name = s.str();
				break;
			}
      
      
    	}
	frc::DriverStation::ReportError("Could not configure spark id: " + std::to_string(id) + " error: " + error_name + " method: "+ method_name);
	//LOG_WARN << "Could not configure spark id: " << id << ", error: " << error_name << ", method: " << method_name;
	return false;

}

motorcontrol::FeedbackDevice TalonBaseWrapper::getCTREFeedbackDevice(MotorControllerConfig::FeedbackSensor sensor)
{
	if (!isAzimuth)
	{
		return motorcontrol::FeedbackDevice::IntegratedSensor;
	}
	
	switch (sensor)
	{
		case MotorControllerConfig::FeedbackSensor::CTRE_MAG_ENCODER:
			return motorcontrol::FeedbackDevice::PulseWidthEncodedPosition;
		case MotorControllerConfig::FeedbackSensor::CAN_CODER:
			return motorcontrol::FeedbackDevice::RemoteSensor0;
		case MotorControllerConfig::FeedbackSensor::THRIFTY_CODER:
			return motorcontrol::FeedbackDevice::PulseWidthEncodedPosition;
		case MotorControllerConfig::FeedbackSensor::INTEGRATED_SENSOR:
			if (isAzimuth)
			{
				frc::DriverStation::ReportError("Unexpected feedback sensor: Can't use Integrated Encoder without a Falcon 500");
				return motorcontrol::FeedbackDevice::None;	
			}

			return motorcontrol::FeedbackDevice::IntegratedSensor;
		default:
			frc::DriverStation::ReportError("Unexpected feedback sensor: " + Util::sstr(sensor) + ", Returning default PWM sensor");
			//LOG_WARN << "Unexpected Feedback Device: " << sensor;
			return motorcontrol::FeedbackDevice::None;
	}
}

motorcontrol::NeutralMode TalonBaseWrapper::getCTRENeutralMode(MotorControllerConfig::NeutralMode mode)
{
	switch (mode)
	{
		case MotorControllerConfig::NeutralMode::BRAKE:
			return motorcontrol::NeutralMode::Brake;
		case MotorControllerConfig::NeutralMode::COAST:
			return motorcontrol::NeutralMode::Coast;
		default:
			frc::DriverStation::ReportError("Unexpected Neutral Mode: " + mode);
			//LOG_WARN << "Unexpected Neutral Mode: " << mode;
			return motorcontrol::NeutralMode::Coast;
	}
}

motorcontrol::ControlMode TalonBaseWrapper::getCTREControlMode()
{
	if (isAzimuth)
	{
		//TalonSRX Azimuth defaults to 
		return motorcontrol::ControlMode::MotionMagic;
	}
	switch (driveMode)
	{
		case MotorControllerWrapper::DriveMode::OPEN_LOOP:
			return motorcontrol::ControlMode::PercentOutput;
		case MotorControllerWrapper::DriveMode::CLOSED_LOOP:
			return motorcontrol::ControlMode::Velocity;
		case MotorControllerWrapper::DriveMode::TELEOP:
			return motorcontrol::ControlMode::PercentOutput;
		case MotorControllerWrapper::DriveMode::TRAJECTORY:
			return motorcontrol::ControlMode::Velocity;
		case MotorControllerWrapper::DriveMode::AZIMUTH:
			return motorcontrol::ControlMode::PercentOutput;
		default:
			frc::DriverStation::ReportError("Unknown Drive Mode: " + std::to_string(id));
			//LOG_WARN << "Unknown Drive Mode: " << id;
			return motorcontrol::ControlMode::PercentOutput;

	}
}

bool TalonBaseWrapper::handleCTRECanError(int id, ctre::phoenix::ErrorCode error_code, const std::string &method_name)
{
    std::string error_name;
	switch (error_code)
	{
		case ctre::phoenix::OK :
			return true; // Yay us!

		case ctre::phoenix::CAN_MSG_STALE :
			error_name = "CAN_MSG_STALE/CAN_TX_FULL/TxFailed";
			break;
		case ctre::phoenix::InvalidParamValue :
			error_name = "InvalidParamValue/CAN_INVALID_PARAM";
			break;

		case ctre::phoenix::RxTimeout :
			error_name = "RxTimeout/CAN_MSG_NOT_FOUND";
			break;
		case ctre::phoenix::TxTimeout :
			error_name = "TxTimeout/CAN_NO_MORE_TX_JOBS";
			break;
		case ctre::phoenix::UnexpectedArbId :
			error_name = "UnexpectedArbId/CAN_NO_SESSIONS_AVAIL";
			break;
		case ctre::phoenix::BufferFull :
			error_name = "BufferFull";
			break;
		case ctre::phoenix::CAN_OVERFLOW:
			error_name = "CAN_OVERFLOW";
			break;
		case ctre::phoenix::SensorNotPresent :
			error_name = "SensorNotPresent";
			break;
		case ctre::phoenix::FirmwareTooOld :
			error_name = "FirmwareTooOld";
			break;
		case ctre::phoenix::CouldNotChangePeriod :
			error_name = "CouldNotChangePeriod";
			break;
		case ctre::phoenix::BufferFailure :
			error_name = "BufferFailure";
			break;

		case ctre::phoenix::GENERAL_ERROR :
			error_name = "GENERAL_ERROR";
			break;

		case ctre::phoenix::SIG_NOT_UPDATED :
			error_name = "SIG_NOT_UPDATED";
			break;
		case ctre::phoenix::NotAllPIDValuesUpdated :
			error_name = "NotAllPIDValuesUpdated";
			break;

		case ctre::phoenix::GEN_PORT_ERROR :
			error_name = "GEN_PORT_ERROR";
			break;
		case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH :
			error_name = "PORT_MODULE_TYPE_MISMATCH";
			break;

		case ctre::phoenix::GEN_MODULE_ERROR :
			error_name = "GEN_MODULE_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR :
			error_name = "MODULE_NOT_INIT_SET_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR :
			error_name = "MODULE_NOT_INIT_GET_ERROR";
			break;

		case ctre::phoenix::WheelRadiusTooSmall :
			error_name = "WheelRadiusTooSmall";
			break;
		case ctre::phoenix::TicksPerRevZero :
			error_name = "TicksPerRevZero";
			break;
		case ctre::phoenix::DistanceBetweenWheelsTooSmall :
			error_name = "DistanceBetweenWheelsTooSmall";
			break;
		case ctre::phoenix::GainsAreNotSet :
			error_name = "GainsAreNotSet";
			break;
		case ctre::phoenix::WrongRemoteLimitSwitchSource :
			error_name = "WrongRemoteLimitSwitchSource";
			break;

		case ctre::phoenix::IncompatibleMode :
			error_name = "IncompatibleMode";
			break;
		case ctre::phoenix::InvalidHandle :
			error_name = "InvalidHandle";
			break;

		case ctre::phoenix::FeatureRequiresHigherFirm:
			error_name = "FeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::TalonFeatureRequiresHigherFirm:
			error_name = "TalonFeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::ConfigFactoryDefaultRequiresHigherFirm:
			error_name = "ConfigFactoryDefaultRequiresHigherFirm";
			break;
		case ctre::phoenix::LibraryCouldNotBeLoaded :
			error_name = "LibraryCouldNotBeLoaded";
			break;
		case ctre::phoenix::MissingRoutineInLibrary :
			error_name = "MissingRoutineInLibrary";
			break;
		case ctre::phoenix::ResourceNotAvailable :
			error_name = "ResourceNotAvailable";
			break;

		case ctre::phoenix::PulseWidthSensorNotPresent :
			error_name = "PulseWidthSensorNotPresent";
			break;
		case ctre::phoenix::GeneralWarning :
			error_name = "GeneralWarning";
			break;
		case ctre::phoenix::FeatureNotSupported :
			error_name = "FeatureNotSupported";
			break;
		case ctre::phoenix::NotImplemented :
			error_name = "NotImplemented";
			break;
		case ctre::phoenix::FirmVersionCouldNotBeRetrieved :
			error_name = "FirmVersionCouldNotBeRetrieved";
			break;
		case ctre::phoenix::FeaturesNotAvailableYet :
			error_name = "FeaturesNotAvailableYet";
			break;
		case ctre::phoenix::ControlModeNotValid :
			error_name = "ControlModeNotValid";
			break;

		case ctre::phoenix::ControlModeNotSupportedYet :
			error_name = "ConrolModeNotSupportedYet";
			break;
		case ctre::phoenix::CascadedPIDNotSupporteYet:
			error_name = "CascadedPIDNotSupporteYet/AuxiliaryPIDNotSupportedYet";
			break;
		case ctre::phoenix::RemoteSensorsNotSupportedYet:
			error_name = "RemoteSensorsNotSupportedYet";
			break;
		case ctre::phoenix::MotProfFirmThreshold:
			error_name = "MotProfFirmThreshold";
			break;
		case ctre::phoenix::MotProfFirmThreshold2:
			error_name = "MotProfFirmThreshold2";
			break;

		default:
			{
				std::stringstream s;
				s << "Unknown Talon error " << error_code;
				error_name = s.str();
				break;
			}

	}
    frc::DriverStation::ReportError("Could not configure Talon id: " + std::to_string(id) + " error: " + error_name + " message: "+ method_name);
    //LOG_WARN << "Could not configure Talon id: " << id << ", error: " << error_name << ", method: " << method_name;
	return false;
}

rev::ControlType SparkMaxWrapper::getRevControlType()
{
	if (isAzimuth)
	{
		// Azimuths default to SmartMotion, other option is kPosition
		return rev::ControlType::kSmartMotion;
	}

	switch (driveMode)
	{
		case MotorControllerWrapper::DriveMode::OPEN_LOOP:
			return rev::ControlType::kDutyCycle;
		case MotorControllerWrapper::DriveMode::CLOSED_LOOP:
			return rev::ControlType::kVelocity;
		case MotorControllerWrapper::DriveMode::TELEOP:
			return rev::ControlType::kDutyCycle;
		case MotorControllerWrapper::DriveMode::TRAJECTORY:
			return rev::ControlType::kVelocity;
		case MotorControllerWrapper::DriveMode::AZIMUTH:
			return rev::ControlType::kDutyCycle;
		default:
			frc::DriverStation::ReportError("Unknown Drive Mode: " + std::to_string(id));
			//LOG_WARN << "Unknown Drive Mode: " << id;
			return rev::ControlType::kDutyCycle;

	}
}

SparkMaxWrapper::SparkMaxWrapper(MotorControllerConfig config, int id)
{

	
    //setup spark max
	bool rc = true;
    isAzimuth = config.isAzimuth;
	this->id = id;
	
	sparkMax = std::make_shared<rev::CANSparkMax>(id, getRevMotorType(config.motorType));
    
	pidController = std::make_shared<rev::CANPIDController>(sparkMax->GetPIDController());
	std::cout <<"Constructing SparkMax, id: " << id <<", firmware version: " << sparkMax->GetFirmwareString() << std::endl;
	
	rc &= handleREVCanError(id, sparkMax->RestoreFactoryDefaults(), "RestoreFactoryDefaults()");

    //config PID slots
	rc &= handleREVCanError(id, pidController->SetP(config.slot0.kP, 0), "SetP(0)");
	rc &= handleREVCanError(id, pidController->SetI(config.slot0.kI, 0), "SetI(0)");
	rc &= handleREVCanError(id, pidController->SetD(config.slot0.kD, 0), "SetD(0)");
	rc &= handleREVCanError(id, pidController->SetFF(config.slot0.kF, 0), "SetFF(0)");
	rc &= handleREVCanError(id, pidController->SetIZone(config.slot0.kIZone, 0), "SetIZone(0)");
	rc &= handleREVCanError(id, pidController->SetIMaxAccum(config.slot0.kMaxIAccum, 0), "SetIMaxAccum(0)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxVelocity(config.motionCruiseVelocity, 0), "SetSmartMotionMaxVelocity(0)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxAccel(config.motionAcceleration, 0), "SetSmartMotionAcceleration(0)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionAllowedClosedLoopError(config.slot0.kAllowableError, 0), "SetSmartMotionAllowableError(0)");
	
	rc &= handleREVCanError(id, pidController->SetP(config.slot1.kP, 1), "SetP(1)");
	rc &= handleREVCanError(id, pidController->SetI(config.slot1.kI, 1), "SetI(1)");
	rc &= handleREVCanError(id, pidController->SetD(config.slot1.kD, 1), "SetD(1)");
	rc &= handleREVCanError(id, pidController->SetFF(config.slot1.kF, 1), "SetFF(1)");
	rc &= handleREVCanError(id, pidController->SetIZone(config.slot1.kIZone, 1), "SetIZone(1)");
	rc &= handleREVCanError(id, pidController->SetIMaxAccum(config.slot1.kMaxIAccum, 1), "SetIMaxAccum(1)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxVelocity(config.motionCruiseVelocity, 1), "SetSmartMotionMaxVelocity(1)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxAccel(config.motionAcceleration, 1), "SetSmartMotionAcceleration(1)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionAllowedClosedLoopError(config.slot1.kAllowableError, 1), "SetSmartMotionAllowableError(1)");

	rc &= handleREVCanError(id, pidController->SetP(config.slot2.kP, 2), "SetP(2)");
	rc &= handleREVCanError(id, pidController->SetI(config.slot2.kI, 2), "SetI(2)");
	rc &= handleREVCanError(id, pidController->SetD(config.slot2.kD, 2), "SetD(2)");
	rc &= handleREVCanError(id, pidController->SetFF(config.slot2.kF, 2), "SetFF(2)");
	rc &= handleREVCanError(id, pidController->SetIZone(config.slot2.kIZone, 2), "SetIZone(2)");
	rc &= handleREVCanError(id, pidController->SetIMaxAccum(config.slot2.kMaxIAccum, 2), "SetIMaxAccum(2)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxVelocity(config.motionCruiseVelocity, 2), "SetSmartMotionMaxVelocity(2)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxAccel(config.motionAcceleration, 2), "SetSmartMotionAcceleration(2)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionAllowedClosedLoopError(config.slot2.kAllowableError, 2), "SetSmartMotionAllowableError(2)");

	rc &= handleREVCanError(id, pidController->SetP(config.slot3.kP, 3), "SetP(3)");
	rc &= handleREVCanError(id, pidController->SetI(config.slot3.kI, 3), "SetI(3)");
	rc &= handleREVCanError(id, pidController->SetD(config.slot3.kD, 3), "SetD(3)");
	rc &= handleREVCanError(id, pidController->SetFF(config.slot3.kF, 3), "SetFF(3)");
	rc &= handleREVCanError(id, pidController->SetIZone(config.slot3.kIZone, 3), "SetIZone(3)");
	rc &= handleREVCanError(id, pidController->SetIMaxAccum(config.slot3.kMaxIAccum, 3), "SetIMaxAccum(3)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxVelocity(config.motionCruiseVelocity, 3), "SetSmartMotionMaxVelocity(3)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionMaxAccel(config.motionAcceleration, 3), "SetSmartMotionAcceleration(3)");
	rc &= handleREVCanError(id, pidController->SetSmartMotionAllowedClosedLoopError(config.slot3.kAllowableError, 3), "SetSmartMotionAllowableError(3)");
	
	rc &= handleREVCanError(id, sparkMax->SetSmartCurrentLimit(config.continuousCurrentLimit), "SetSmartCurrentLimit()");
	rc &= handleREVCanError(id, sparkMax->SetSecondaryCurrentLimit(config.peakCurrentLimit), "SetSecondaryCurrentLimit()");
	rc &= handleREVCanError(id, sparkMax->EnableVoltageCompensation(config.voltageCompensation), "EnableVoltageCompensation()");
	rc &= handleREVCanError(id, sparkMax->SetIdleMode(getRevIdleMode(config.neutralMode)), "SetIdleMode()");
	
	//TODO: add Feedback device?

	// Rotations -> Ticks (same as Talons)
	//setPositionFactor(Constants::TICKS);

	// Rotations / second -> Ticks / 100 ms
	//setVelocityFactor(Constants::TICKS/10)

	if (config.feedbackSensor == MotorControllerConfig::FeedbackSensor::CAN_CODER)
	{
		// Configure CANCoder
		canCoder = std::make_shared<CANCoder>(id + 20);
		pid = std::make_shared<frc2::PIDController>(config.slot0.kP, config.slot0.kI, config.slot0.kD);
	} else if (config.feedbackSensor == MotorControllerConfig::FeedbackSensor::THRIFTY_CODER)
	{
		// Configure ThriftyCoder and RIO pid
		pwm = std::make_shared<frc::PWM>(id);
		pid = std::make_shared<frc2::PIDController>(config.slot0.kP, config.slot0.kI, config.slot0.kD);
	} else
	{
		encoder = std::make_shared<rev::CANEncoder>(sparkMax->GetEncoder());
		
		//TODO: add integrated sensor?

		// Rotations -> Ticks (same as Talons)
		//setPositionFactor(Constants::TICKS);

		// Rotations / second -> Ticks / 100 ms
		//setVelocityFactor(Constants::TICKS/10)
	}
	
	


	if (!rc)
	{
		//LOG_WARN << "Spark Max ("<<id<<"): failed to initialize";
	}
	
}

void SparkMaxWrapper::set(double output)
{
	
	if (canCoder)
	{
		//CANCoder
		pidController->SetReference(output, rev::ControlType::kDutyCycle, slot);	
	} else if (pwm)
	{
		//Thrifty
		pidController->SetReference(output, rev::ControlType::kDutyCycle, slot);
	} else
	{
		//Integrated
		pidController->SetReference(output, getRevControlType(), slot);
		//pidController->SetReference(output, rev::ControlType::kVelocity, slot);
		
		frc::SmartDashboard::PutNumber("SparkMax Output " + std::to_string(id), output);	
		frc::SmartDashboard::PutNumber("SparkMax Get Output" + std::to_string(id), sparkMax->Get());	
		
	}
	
	//frc::SmartDashboard::PutNumber("id: " + std::to_string(id), sparkMax->GetEncoder().GetPosition());
	//frc::SmartDashboard::PutNumber("SparkMax Get Output" + std::to_string(id), sparkMax->Get());
	//sparkMax->Set(output);
	//frc::SmartDashboard::PutNumber("SparkMax Output " + std::to_string(id), output);	
	
	//frc::SmartDashboard::PutNumber("SparkMax Set Output " + std::to_string(id), output);
	//pidController->SetReference(output, rev::ControlType::kDutyCycle, slot);
	//sparkMax->Set(output);
	
}

void SparkMaxWrapper::setSensorPosition(double position)
{
	if (canCoder)
	{
		//CANCoder
		canCoder->SetPosition(position);
	} else if (pwm)
	{
		//Thrifty
		pwm->SetPosition(position);
	} else
	{
		//Integrated
		encoder->SetPosition(position);
	}
}

void SparkMaxWrapper::setNeutralOutput()
{
	set(0.0);
}

double SparkMaxWrapper::getPosition()
{
	if (canCoder)
	{
		//CANCoder
		return canCoder->GetPosition();
	} else if (pwm)
	{
		//Thrifty
		return pwm->GetPosition();
	} else
	{
		//Integrated
		return encoder->GetPosition();
	}
}

double SparkMaxWrapper::getAbsPosition()
{
	if (canCoder)
	{
		//CANCoder
		return canCoder->GetAbsolutePosition();
	} else if (pwm)
	{
		//Thrifty
		return pwm->GetPosition();
	} else
	{
		//Integrated
		return encoder->GetPosition();
	}
}

double SparkMaxWrapper::getVelocity()
{
	if (canCoder)
	{
		//CANCoder
		return canCoder->GetVelocity();
	} else if (pwm)
	{
		//Thrifty
		return pwm->GetSpeed();
	} else
	{
		//Integrated
		return encoder->GetVelocity();
	}
}

double SparkMaxWrapper::getOutput()
{
	return sparkMax->GetBusVoltage() * sparkMax->GetAppliedOutput();
}

double SparkMaxWrapper::getCurrent()
{
	return sparkMax->GetOutputCurrent();
}

std::string SparkMaxWrapper::getString()
{
	std::stringstream stream = std::stringstream();
	stream << std::fixed << std::setprecision(4) << id << ", " << getPosition() << ", " << getAbsPosition() << ", " << getVelocity() << ", " << getOutput() << ", " << getCurrent() << ", ";
	
	return stream.str(); 
}

VictorSPXWrapper::VictorSPXWrapper(MotorControllerConfig config, int id)
{
	victorSPX = std::make_shared<ctre::phoenix::motorcontrol::can::VictorSPX>(id);
	std::cout <<"Constructing VictorSPX, id: " << id << std::endl;
	isAzimuth = config.isAzimuth;
	feedbackSensor = config.feedbackSensor;
	this->id = id;

	bool rc = true;

	rc &= handleCTRECanError(id, victorSPX->ConfigFactoryDefault(), "ConfigFactoryDefault");

	VictorSPXConfiguration victorSPXConfig{};
	victorSPXConfig.voltageCompSaturation = config.voltageCompensation;
	if (config.feedbackSensor == MotorControllerConfig::FeedbackSensor::CAN_CODER)
	{
		// Configure CANCoder
		canCoder = std::make_shared<CANCoder>(id + 20);
		// Configure Victor for Remote CANCoder
		victorSPXConfig.primaryPID.selectedFeedbackSensor = motorcontrol::RemoteFeedbackDevice::RemoteSensor0;
		
		victorSPXConfig.slot0.kP = config.slot0.kP;
		victorSPXConfig.slot0.kI = config.slot0.kI;
		victorSPXConfig.slot0.kD = config.slot0.kD;
		victorSPXConfig.slot0.kF = config.slot0.kF;
		victorSPXConfig.slot0.integralZone = config.slot0.kIZone;
		victorSPXConfig.slot0.allowableClosedloopError = config.slot0.kAllowableError;
		
		victorSPXConfig.slot1.kP = config.slot1.kP;
		victorSPXConfig.slot1.kI = config.slot1.kI;
		victorSPXConfig.slot1.kD = config.slot1.kD;
		victorSPXConfig.slot1.kF = config.slot1.kF;
		victorSPXConfig.slot1.integralZone = config.slot1.kIZone;
		victorSPXConfig.slot1.allowableClosedloopError = config.slot1.kAllowableError;
		
		victorSPXConfig.slot2.kP = config.slot2.kP;
		victorSPXConfig.slot2.kI = config.slot2.kI;
		victorSPXConfig.slot2.kD = config.slot2.kD;
		victorSPXConfig.slot2.kF = config.slot2.kF;
		victorSPXConfig.slot2.integralZone = config.slot2.kIZone;
		victorSPXConfig.slot2.allowableClosedloopError = config.slot2.kAllowableError;

		victorSPXConfig.slot3.kP = config.slot3.kP;
		victorSPXConfig.slot3.kI = config.slot3.kI;
		victorSPXConfig.slot3.kD = config.slot3.kD;
		victorSPXConfig.slot3.kF = config.slot3.kF;
		victorSPXConfig.slot3.integralZone = config.slot3.kIZone;
		victorSPXConfig.slot3.allowableClosedloopError = config.slot3.kAllowableError;

		victorSPXConfig.motionAcceleration = config.motionAcceleration;
		victorSPXConfig.motionCruiseVelocity = config.motionCruiseVelocity;
		victorSPXConfig.velocityMeasurementWindow = 64;

		rc &= handleCTRECanError(id, victorSPX->ConfigRemoteFeedbackFilter(this->id + 20, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 10), "ConfigRemoteFeedbackFilter");

	} else
	{
		victorSPXConfig.primaryPID.selectedFeedbackSensor = motorcontrol::RemoteFeedbackDevice::None;
		//TODO: add other devices
		pwm = std::make_shared<frc::PWM>(id);
		pid = std::make_shared<frc2::PIDController>(config.slot0.kP, config.slot0.kI, config.slot0.kD);
	}
	
	rc &= handleCTRECanError(id, victorSPX->ConfigAllSettings(victorSPXConfig), "ConfigAllSettings");
	
	//TODO: Add Fault Handling
	victorSPX->EnableVoltageCompensation(true);
	victorSPX->SetNeutralMode(getCTRENeutralMode(config.neutralMode));
	
	if (!rc)
	{
		//LOG_WARN << "VictorSPX ("<<id<<"): failed to initialize";
	}
}

void VictorSPXWrapper::set(double output)
{
	//TODO: Check Percent ouptut is correct for pwm
	if (canCoder)
	{
		//CANCoder
		victorSPX->Set(motorcontrol::ControlMode::MotionMagic, output);
	} else
	{
		//Thrifty TODO: add ouptut conversion
		victorSPX->Set(motorcontrol::ControlMode::PercentOutput, output);	
	}
	
	
}

void VictorSPXWrapper::setSensorPosition(double position)
{
	//TODO: Update once feedback device is figured out
	if (canCoder)
	{
		//CANCoder
		canCoder->SetPosition(position);
	} else
	{
		//Thrifty
		//pwm->SetPosition(position);	
	}
}

void VictorSPXWrapper::setNeutralOutput()
{
	set(0.0);
}

double VictorSPXWrapper::getPosition()
{
	if (canCoder)
	{
		//CANCoder
		return canCoder->GetPosition();
	} else
	{
		//Thrifty
		return pwm->GetPosition();	
	}
}

double VictorSPXWrapper::getAbsPosition()
{
	if (canCoder)
	{
		//CANCoder
		return canCoder->GetAbsolutePosition();
	} else
	{
		//Thrifty
		return pwm->GetPosition();	
	}
}

double VictorSPXWrapper::getVelocity()
{
	if (canCoder)
	{
		//CANCoder
		return canCoder->GetVelocity();
	} else
	{
		//Thrifty
		return pwm->GetSpeed();	
	}
}

double VictorSPXWrapper::getOutput()
{
	return victorSPX->GetMotorOutputPercent();
}

double VictorSPXWrapper::getCurrent()
{
	// Not available on Victor
	return 0.0;
}

std::string VictorSPXWrapper::getString()
{
	// TODO: Test moving to base class
	std::stringstream stream = std::stringstream();
	stream << std::fixed << std::setprecision(4) << id << ", " << getPosition() << ", " << getAbsPosition() << ", " << getVelocity() << ", " << getOutput() << ", " << getCurrent() << ", ";
	
	return stream.str(); 
}

TalonSRXWrapper::TalonSRXWrapper(MotorControllerConfig config, int id)
{
	talonSRX = std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(id);
	std::cout <<"Constructing TalonSRX, id: " << id << std::endl;
	
    isAzimuth = config.isAzimuth;
	this->id = id;
	bool rc = true;

	rc &= handleCTRECanError(id, talonSRX->ConfigFactoryDefault(), "ConfigFactoryDefault");

	TalonSRXConfiguration talonSRXConfig{};
    talonSRXConfig.primaryPID.selectedFeedbackSensor = getCTREFeedbackDevice(config.feedbackSensor);
    talonSRXConfig.continuousCurrentLimit = config.continuousCurrentLimit;
    talonSRXConfig.peakCurrentDuration = 1.0;
    talonSRXConfig.peakCurrentLimit = config.peakCurrentLimit;
    
	talonSRXConfig.slot0.kP = config.slot0.kP;
    talonSRXConfig.slot0.kI = config.slot0.kI;
    talonSRXConfig.slot0.kD = config.slot0.kD;
    talonSRXConfig.slot0.kF = config.slot0.kF;
    talonSRXConfig.slot0.integralZone = config.slot0.kIZone;
    talonSRXConfig.slot0.allowableClosedloopError = config.slot0.kAllowableError;
    
	talonSRXConfig.slot1.kP = config.slot1.kP;
    talonSRXConfig.slot1.kI = config.slot1.kI;
    talonSRXConfig.slot1.kD = config.slot1.kD;
    talonSRXConfig.slot1.kF = config.slot1.kF;
    talonSRXConfig.slot1.integralZone = config.slot1.kIZone;
    talonSRXConfig.slot1.allowableClosedloopError = config.slot1.kAllowableError;
	
	talonSRXConfig.slot2.kP = config.slot2.kP;
    talonSRXConfig.slot2.kI = config.slot2.kI;
    talonSRXConfig.slot2.kD = config.slot2.kD;
    talonSRXConfig.slot2.kF = config.slot2.kF;
    talonSRXConfig.slot2.integralZone = config.slot2.kIZone;
    talonSRXConfig.slot2.allowableClosedloopError = config.slot2.kAllowableError;

	talonSRXConfig.slot3.kP = config.slot3.kP;
    talonSRXConfig.slot3.kI = config.slot3.kI;
    talonSRXConfig.slot3.kD = config.slot3.kD;
    talonSRXConfig.slot3.kF = config.slot3.kF;
    talonSRXConfig.slot3.integralZone = config.slot3.kIZone;
    talonSRXConfig.slot3.allowableClosedloopError = config.slot3.kAllowableError;

	talonSRXConfig.motionAcceleration = config.motionAcceleration;
    talonSRXConfig.motionCruiseVelocity = config.motionCruiseVelocity;
    talonSRXConfig.velocityMeasurementWindow = 64;
    talonSRXConfig.voltageCompSaturation = config.voltageCompensation;

	rc &= handleCTRECanError(id, talonSRX->ConfigAllSettings(talonSRXConfig), "ConfigAllSettings");
	talonSRX->EnableCurrentLimit(true);
	talonSRX->EnableVoltageCompensation(true);
	talonSRX->SetNeutralMode(getCTRENeutralMode(config.neutralMode));

	if (!rc)
	{
		//LOG_WARN << "TalonSRX ("<<id<<"): failed to initialize";
	}
}

void TalonSRXWrapper::set(double output)
{
	talonSRX->Set(getCTREControlMode(), output);
}

void TalonSRXWrapper::setSensorPosition(double position)
{
	talonSRX->SetSelectedSensorPosition(position, 0, 10);
	set(position);
}

void TalonSRXWrapper::setNeutralOutput()
{
	talonSRX->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
}

double TalonSRXWrapper::getPosition()
{
	return talonSRX->GetSelectedSensorPosition(0);
}

double TalonSRXWrapper::getAbsPosition()
{
	return talonSRX->GetSensorCollection().GetPulseWidthPosition();
}

double TalonSRXWrapper::getVelocity()
{
	// TODO: Check if should be (talonSRX->GetSelectedSensorVelocity())
	return talonSRX->GetSensorCollection().GetPulseWidthVelocity();
}

double TalonSRXWrapper::getOutput()
{
	return talonSRX->GetMotorOutputVoltage();
}

double TalonSRXWrapper::getCurrent()
{
	return talonSRX->GetOutputCurrent();
}

std::string TalonSRXWrapper::getString()
{
	// TODO: Test moving to base class
	std::stringstream stream = std::stringstream();
	stream << std::fixed << std::setprecision(4) << id << ", " << getPosition() << ", " << getAbsPosition() << ", " << getVelocity() << ", " << getOutput() << ", " << getCurrent() << ", ";
	
	return stream.str(); 
}

TalonFXWrapper::TalonFXWrapper(MotorControllerConfig config, int id)
{
	talonFX = std::make_shared<ctre::phoenix::motorcontrol::can::TalonFX>(id);
	std::cout <<"Constructing TalonFX, id: " << id << std::endl;
	isAzimuth = config.isAzimuth;
	this->id = id;

	bool rc = true;
	
	rc &= handleCTRECanError(id, talonFX->ConfigFactoryDefault(), "ConfigFactoryDefault");
	
	TalonFXConfiguration talonFXConfig{};
    talonFXConfig.primaryPID.selectedFeedbackSensor = getCTREFeedbackDevice(config.feedbackSensor);
    talonFXConfig.supplyCurrLimit.enable = true;
	talonFXConfig.supplyCurrLimit.triggerThresholdCurrent = config.continuousCurrentLimit;
    talonFXConfig.supplyCurrLimit.triggerThresholdTime = 1.0;
    talonFXConfig.supplyCurrLimit.currentLimit = config.peakCurrentLimit;

	talonFXConfig.statorCurrLimit.enable = true;
	talonFXConfig.statorCurrLimit.triggerThresholdCurrent = config.continuousCurrentLimit;
    talonFXConfig.statorCurrLimit.triggerThresholdTime = 1.0;
    talonFXConfig.statorCurrLimit.currentLimit = config.peakCurrentLimit;
    
	talonFXConfig.slot0.kP = config.slot0.kP;
    talonFXConfig.slot0.kI = config.slot0.kI;
    talonFXConfig.slot0.kD = config.slot0.kD;
    talonFXConfig.slot0.kF = config.slot0.kF;
    talonFXConfig.slot0.integralZone = config.slot0.kIZone;
    talonFXConfig.slot0.allowableClosedloopError = config.slot0.kAllowableError;
    
	talonFXConfig.slot1.kP = config.slot1.kP;
    talonFXConfig.slot1.kI = config.slot1.kI;
    talonFXConfig.slot1.kD = config.slot1.kD;
    talonFXConfig.slot1.kF = config.slot1.kF;
    talonFXConfig.slot1.integralZone = config.slot1.kIZone;
    talonFXConfig.slot1.allowableClosedloopError = config.slot1.kAllowableError;
	
	talonFXConfig.slot2.kP = config.slot2.kP;
    talonFXConfig.slot2.kI = config.slot2.kI;
    talonFXConfig.slot2.kD = config.slot2.kD;
    talonFXConfig.slot2.kF = config.slot2.kF;
    talonFXConfig.slot2.integralZone = config.slot2.kIZone;
    talonFXConfig.slot2.allowableClosedloopError = config.slot2.kAllowableError;

	talonFXConfig.slot3.kP = config.slot3.kP;
    talonFXConfig.slot3.kI = config.slot3.kI;
    talonFXConfig.slot3.kD = config.slot3.kD;
    talonFXConfig.slot3.kF = config.slot3.kF;
    talonFXConfig.slot3.integralZone = config.slot3.kIZone;
    talonFXConfig.slot3.allowableClosedloopError = config.slot3.kAllowableError;

	talonFXConfig.motionAcceleration = config.motionAcceleration;
    talonFXConfig.motionCruiseVelocity = config.motionCruiseVelocity;
    talonFXConfig.velocityMeasurementWindow = 64;
    talonFXConfig.voltageCompSaturation = config.voltageCompensation;

	rc &= handleCTRECanError(id, talonFX->ConfigAllSettings(talonFXConfig), "ConfigAllSettings");
	talonFX->EnableVoltageCompensation(true);
	talonFX->SetNeutralMode(getCTRENeutralMode(config.neutralMode));

	if (!rc)
	{
		//LOG_WARN << "TalonFX ("<<id<<"): failed to initialize";
	}
}

void TalonFXWrapper::set(double output)
{
	talonFX->Set(getCTREControlMode(), output);
}

void TalonFXWrapper::setSensorPosition(double position)
{
	talonFX->SetSelectedSensorPosition(position, 0, 10);
	set(position);
}

void TalonFXWrapper::setNeutralOutput()
{
	talonFX->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
}

double TalonFXWrapper::getPosition()
{
	return talonFX->GetSelectedSensorPosition(0);
}

double TalonFXWrapper::getAbsPosition()
{
	return talonFX->GetSelectedSensorPosition(0);
}

double TalonFXWrapper::getVelocity()
{
	// TODO: Check if should be (talonSRX->GetSelectedSensorVelocity())
	return talonFX->GetSelectedSensorVelocity(0);
}

double TalonFXWrapper::getOutput()
{
	return talonFX->GetMotorOutputVoltage();
}

double TalonFXWrapper::getCurrent()
{
	return talonFX->GetOutputCurrent();
}

std::string TalonFXWrapper::getString()
{
	// TODO: Test moving to base class
	std::stringstream stream = std::stringstream();
	stream << std::fixed << std::setprecision(4) << id << ", " << getPosition() << ", " << getAbsPosition() << ", " << getVelocity() << ", " << getOutput() << ", " << getCurrent() << ", ";
	
	return stream.str(); 
}