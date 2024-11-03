#include "FourBarLinkage.h"

void FourBarLinkage::init() {
    // resets motor settings to default settings
    linkageMotor1.RestoreFactoryDefaults();
    linkageMotor2.RestoreFactoryDefaults();
    // sets idle mode to brake so the motors do not move 
    linkageMotor1.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    linkageMotor2.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    // sets current limit for motors
    linkageMotor1.SetSmartCurrentLimit(linkageMotorCurrentLimit);
    linkageMotor2.SetSmartCurrentLimit(linkageMotorCurrentLimit);
    // zeros encoders by setting the position of the encoders to zero
    linkageMtrEnc1.SetPosition(0.0);
    linkageMtrEnc2.SetPosition(0.0);
}

void FourBarLinkage::disable() {
    // stop motors
    linkageMotor1.StopMotor();
    linkageMotor2.StopMotor();
}

void FourBarLinkage::setExtendAngle(double angle) { // give as degrees, sets linkageExtendAngle as revolutions
    linkageExtendAngle = angle/360.0; // converts degrees to revolutions
    frc::SmartDashboard::PutNumber("Extend Angle", linkageExtendAngle); 
}

double FourBarLinkage::getEncoderPosition() { // returns in degrees 
    double encoderPosition1 = linkageMtrEnc1.GetPosition()*360.0; // converts revolutions to degrees
    double encoderPosition2 = linkageMtrEnc2.GetPosition()*360.0; // converts revolutions to degrees
    frc::SmartDashboard::PutNumber("Encoder Position, Motor 1", encoderPosition1);
    frc::SmartDashboard::PutNumber("Encoder Postition, Motor 2", encoderPosition2);

    if(encoderPosition1 == encoderPosition2) { // check if encoder positions are the same
        frc::SmartDashboard::PutBoolean("Encoders Same", true);
        return encoderPosition1; 
    }
    else {
        frc::SmartDashboard::PutBoolean("Encoders Same", false);
    }
}

void FourBarLinkage::setLinkagePosition(linkageState state) {
    switch (state) {
        case EXTEND:
            // current limit
            linkageMotor1.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            linkageMotor2.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            // move motor to certain angle (given in revolutions)
            linkageMotorCtr1.SetReference(linkageExtendAngle, rev::CANSparkBase::ControlType::kPosition);
            linkageMotorCtr2.SetReference(linkageExtendAngle, rev::CANSparkBase::ControlType::kPosition);
            frc::SmartDashboard::PutBoolean("Extended", true);
        case RETRACT:
            // current limit
            linkageMotor1.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            linkageMotor2.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            // move motor back to its zeroed position
            linkageMotorCtr1.SetReference(0.0, rev::CANSparkBase::ControlType::kPosition);
            linkageMotorCtr2.SetReference(0.0, rev::CANSparkBase::ControlType::kPosition);
            frc::SmartDashboard::PutBoolean("Extended", false);
        case STOP:
            // stop motors
            disable();
    }
}

void FourBarLinkage::extendLinkage() { // chooses whether or not to retract or extend the linkage
    if(getEncoderPosition() == linkageExtendAngle*360.0) {
        setLinkagePosition(RETRACT);
    }
    if(getEncoderPosition() == 0.0) {
        setLinkagePosition(EXTEND);
    }
}