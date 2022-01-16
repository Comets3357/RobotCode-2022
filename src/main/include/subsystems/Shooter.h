#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <frc/smartdashboard/SmartDashboard.h>


struct RobotData;

struct ShooterData
{
    bool readyShoot;
    int targetVel;
};

class Shooter
{

    public:
        void RobotInit();
        void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
        void DisabledInit();

    private:
        void updateData(const RobotData &robotData, ShooterData &shooterData);
        void manual(const RobotData &robotData, ShooterData &shooterData);
        void semiAuto(const RobotData &robotData, ShooterData &shooterData);

        void shooterWheelLeadInit();
        void shooterWheelFollowInit();
        void shooterHoodInit();

        void setShooterPID(rev::SparkMaxPIDController motor, int pidSlot, double p, double i, double d, double ff);
        double getHoodPos();
        double getWheelPos();
        double getWheelVel();
        double getHoodOffset();
        bool getHoodLimitSwitch();

        void setHood(double power);
        void setWheel(double power);
        void setHoodPos(double pos);
        void setTurretPos(double pos);

        bool hoodZero;

        //CHANGE MOTOr ID STUFF
        rev::CANSparkMax shooterWheelLead = rev::CANSparkMax(shooterWheelLeadID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder shooterWheelLeadEncoder = shooterWheelLead.GetEncoder();
        rev::SparkMaxPIDController shooterWheelLead_pidController = shooterWheelLead.GetPIDController();

        rev::CANSparkMax shooterWheelFollow = rev::CANSparkMax(shooterWheelFollowID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder shooterWheelFollowEncoder = shooterWheelFollow.GetEncoder();
        rev::SparkMaxPIDController shooterWheelFollow_pidController = shooterWheelFollow.GetPIDController();

        rev::CANSparkMax shooterHood = rev::CANSparkMax(shooterHoodID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder shooterHoodEncoder = shooterHood.GetEncoder();
        rev::SparkMaxPIDController shooterHood_pidController = shooterHood.GetPIDController();

        rev::SparkMaxLimitSwitch hoodReverseLimit = shooterHood.GetReverseLimitSwitch((rev::SparkMaxLimitSwitch::LimitSwitchPolarity::kNormallyClosed));

};