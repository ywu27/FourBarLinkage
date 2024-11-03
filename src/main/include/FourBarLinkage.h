#pragma once
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>

class FourBarLinkage {
    private:
        int linkageExtendAngle = 0;
        int linkageMotorCurrentLimit;
        // initialize linkage motors
        rev::CANSparkMax linkageMotor1;
        rev::CANSparkMax linkageMotor2;
        // initialize motor controllers
        rev::SparkPIDController linkageMotorCtr1;
        rev::SparkPIDController linkageMotorCtr2;
        // initialize motor encoders
        rev::SparkRelativeEncoder linkageMtrEnc1;
        rev::SparkRelativeEncoder linkageMtrEnc2;
    
    public:
        FourBarLinkage(int linkageMotorID1, int linkageMotorID2, int linkageMotorCurrentLimit);
        enum linkageState { // linkage position or state
            EXTEND,
            RETRACT
        };

        void init();
        void disable();
        void setExtendAngle(double angle);
        void setLinkagePosition(linkageState state);
        void extendLinkage();
        double getEncoderPosition();
};