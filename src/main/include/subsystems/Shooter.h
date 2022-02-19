#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalSource.h>

struct RobotData;

struct ShooterData
{
    bool readyShoot;
    bool shootUnassignedAsOpponent;

};

class Shooter{

    public:
        void RobotInit();
        void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
        void DisabledInit();
        void EnabledInit(ShooterData &shooterData);
        void updateData(const RobotData &robotData, ShooterData &shooterData);

    
    private:
        void manual(const RobotData &robotData, ShooterData &shooterData);
        void semiAuto(const RobotData &robotData, ShooterData &shooterData);

        double convertFromAngleToAbs(double angle);
        double convertFromAbsToAngle(double abs);
        double absoluteToREV(double value);
        void checkReadyShoot(ShooterData &shooterData);
        void encoderPluggedIn(const ShooterData &shooterData);

        void flyWheelInit();
        void shooterHoodInit();

        double getWheelVel();

        //FIXED SHOTS
        void outerLaunch(const RobotData &robotData);
        void innerLaunch(const RobotData &robotData);
        void wall(const RobotData &robotData);
        void fender(const RobotData &robotData);
        void endOfTarmac(const RobotData &robotData);

        int readyShootLimit;
        int tickCount;
    
        //FLywheel Lead
        rev::CANSparkMax flyWheelLead = rev::CANSparkMax(shooterWheelLeadID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder flyWheelLeadEncoder = flyWheelLead.GetEncoder();
        rev::SparkMaxPIDController flyWheelLead_pidController = flyWheelLead.GetPIDController();

        //flywheel hood, rev encoder, abs encoder, and pid
        rev::CANSparkMax shooterHood = rev::CANSparkMax(shooterHoodID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder shooterHoodEncoderRev = shooterHood.GetEncoder();
        rev::SparkMaxPIDController shooterHood_pidController = shooterHood.GetPIDController();
        
        frc::DigitalInput m_input{HoodAbsoluteEncoderPort};
        frc::DutyCycle shooterHoodEncoderAbs = frc::DutyCycle{m_input};

};