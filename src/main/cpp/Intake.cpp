#include "Intake.h"

void Intake::init() {
    // Restore motor settings to default settings
    intakeTopMotor.RestoreFactoryDefaults();
    intakeBottomMotor.RestoreFactoryDefaults();

    // Set amount of current that the motor can draw
    intakeTopMotor.SetSmartCurrentLimit(intakeMotorCurrentLimit);
    intakeBottomMotor.SetSmartCurrentLimit(intakeMotorCurrentLimit);

    // Set motors to brake mode (stiffened up)
    intakeTopMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    intakeBottomMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
}

void Intake::disable() {
    intakeTopMotor.StopMotor();
    intakeBottomMotor.StopMotor();
}

void Intake::setIntakeSpeed(double speed) {
    intakeMotorSpeed = speed;
}

void Intake::setIntakeState(intakeState state) {
    // intake state 
    switch(state) {
        // if intake state is on, set reference point and set current limit
        case ON:
            intakeTopMotor.SetSmartCurrentLimit(intakeMotorCurrentLimit);
            intakeBottomMotor.SetSmartCurrentLimit(intakeMotorCurrentLimit);
            intakeTopCtr.SetReference(intakeMotorSpeed, rev::CANSparkBase::ControlType::kVelocity);
            intakeBottomCtr.SetReference(intakeMotorSpeed, rev::CANSparkBase::ControlType::kVelocity);
            break;
        // if intake state is off, stop the motor
        case STOP:
            disable();
    }
}