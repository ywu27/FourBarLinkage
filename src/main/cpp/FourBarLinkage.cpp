#include "FourBarLinkage.h"

void FourBarLinkage::init() {
    linkageMotor1.RestoreFactoryDefaults();
    linkageMotor2.RestoreFactoryDefaults();
    linkageMotor1.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    linkageMotor2.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    linkageMotor1.SetSmartCurrentLimit(linkageMotorCurrentLimit);
    linkageMotor2.SetSmartCurrentLimit(linkageMotorCurrentLimit);

    linkageMtrEnc1.SetPosition(0.0);
    linkageMtrEnc2.SetPosition(0.0);
}

void FourBarLinkage::disable() {
    linkageMotor1.StopMotor();
    linkageMotor2.StopMotor();
}

void FourBarLinkage::setExtendAngle(double angle) {
    linkageExtendAngle = angle;
    frc::SmartDashboard::PutNumber("Extend Angle", linkageExtendAngle);
}

void FourBarLinkage::setLinkagePosition(linkageState state) {
    switch (state) {
        case EXTEND:
            linkageMotor1.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            linkageMotor2.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            linkageMotorCtr1.SetReference(linkageExtendAngle, rev::CANSparkBase::ControlType::kPosition);
            linkageMotorCtr2.SetReference(linkageExtendAngle, rev::CANSparkBase::ControlType::kPosition);
        case RETRACT:
            linkageMotor1.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            linkageMotor2.SetSmartCurrentLimit(linkageMotorCurrentLimit);
            linkageMotorCtr1.SetReference(0, rev::CANSparkBase::ControlType::kPosition);
            linkageMotorCtr2.SetReference(0, rev::CANSparkBase::ControlType::kPosition);
        case STOP:
            disable();
    }
}