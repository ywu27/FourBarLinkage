#pragma once
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Intake {
    private:
        rev::CANSparkMax intakeTopMotor;
        rev::CANSparkMax intakeBottomMotor;
        rev::SparkPIDController intakeTopCtr;
        rev::SparkPIDController intakeBottomCtr;
        rev::SparkRelativeEncoder intakeTopEncoder;
        rev::SparkRelativeEncoder intakeBottomEncoder;

        int intakeMotorCurrentLimit; // Speed of the intake motors 
        double intakeMotorSpeed;

    public:
        Intake(int topMotorID, int bottomMotorID, int currentLimit);
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