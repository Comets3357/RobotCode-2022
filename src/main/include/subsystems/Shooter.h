#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalSource.h>

struct RobotData;

struct ShooterData
{
    bool readyShoot;
    bool wrongBallReady;
    bool shootUnassignedAsOpponent;
    bool isHighGeneral;
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

        void flyWheelInit();
        void shooterHoodInit();

        double getHoodPos();
        double getWheelPos();
        double getWheelVel();
        double getHoodOffset();

        void setHood(double power);
        void setWheel(double power);
        void setHoodPos(double pos);
        void setTurretPos(double pos);

        void setHighHub(const RobotData &robotData);
        void outerLaunch(const RobotData &robotData);
        void innerLaunch(const RobotData &robotData);
        void wall(const RobotData &robotData);
        void fender(const RobotData &robotData);
        void endOfTarmac(const RobotData &robotData);

        bool hoodZero;
        double targetHoodPos;
        double currentHoodPos;
        double desiredPos;
        double calculatedPower;
        int readyShootLimit;
        int tickCount;
   
        bool isHigh;

        int lastTickBallCount = 0;

        int x = 0;
        int y = 0;
        int vel = 0;
    
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