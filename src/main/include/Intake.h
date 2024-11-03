#pragma once
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Motor IDs
constexpr unsigned int intakeTopMotorID = 14; // put actual motor ID
constexpr unsigned int intakeBottomMotorID = 15; // put actual motor ID
constexpr unsigned int intakeMotorCurrentLimit = 10; // same as last year but change accordingly

class Intake {
    private:
        int intakeMotorSpeed = 5700; // Kept from last year

        // rev motor, controller, and encoder declarations
        rev::CANSparkMax intakeTopMotor = rev::CANSparkMax(intakeTopMotorID, rev::CANSparkMax::MotorType::kBrushless);
        rev::CANSparkMax intakeBottomMotor = rev::CANSparkMax(intakeBottomMotorID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkPIDController intakeBottomCtr = intakeBottomMotor.GetPIDController();
        rev::SparkPIDController intakeTopCtr = intakeTopMotor.GetPIDController();
        rev::SparkRelativeEncoder intakeBottomEncoder = intakeBottomMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
        rev::SparkRelativeEncoder intakeTopEncoder = intakeTopMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    
    public:
        // intake state
        enum intakeState {
            ON, 
            STOP
        };

        // functions
        void init();
        void disable();
        void setIntakeState(intakeState state);
        void setIntakeSpeed(double speed);
};