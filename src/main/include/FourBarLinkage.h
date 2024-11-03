#pragma once
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>

constexpr unsigned int linkageMotorID1 = 2; // Change to actual motor ID
constexpr unsigned int linkageMotorID2 = 3; // Change to actual motor ID
constexpr unsigned int linkageMotorCurrentLimit = 10; // Change accordingly

class FourBarLinkage {
    private:
        int linkageExtendAngle = 0;

        rev::CANSparkMax linkageMotor1 = rev::CANSparkMax(linkageMotorID1, rev::CANSparkMax::MotorType::kBrushless);
        rev::CANSparkMax linkageMotor2 = rev::CANSparkMax(linkageMotorID2, rev::CANSparkMax::MotorType::kBrushless);

        rev::SparkPIDController linkageMotorCtr1 = linkageMotor1.GetPIDController();
        rev::SparkPIDController linkageMotorCtr2 = linkageMotor2.GetPIDController();

        rev::SparkRelativeEncoder linkageMtrEnc1 = linkageMotor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
        rev::SparkRelativeEncoder linkageMtrEnc2 = linkageMotor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    
    public:
        enum linkageState {
            EXTEND,
            RETRACT,
            STOP
        };

        void init();
        void disable();
        void setExtendAngle(double angle);
        void setLinkagePosition(linkageState state);
};