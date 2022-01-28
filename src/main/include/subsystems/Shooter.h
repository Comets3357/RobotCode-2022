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
#include <frc/DutyCycleEncoder.h>

struct RobotData;

struct ShooterData
{
    bool readyShoot;
    int targetVel;
    bool wrongBallReady;
};

class Shooter{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
    void DisabledInit();

private:
    void updateData(const RobotData &robotData, ShooterData &shooterData);
    void manual(const RobotData &robotData, ShooterData &shooterData);
    void semiAuto(const RobotData &robotData, ShooterData &shooterData);

    void flyWheelInit();
    void shooterHoodInit();

    double getHoodPos();
    double getWheelPos();
    double getWheelVel();
    double getHoodOffset();
    bool getHoodLimitSwitch();

    void setHood(double power);
    void setWheel(double power);
    void setHoodPos(double pos);
    void setTurretPos(double pos);

    double convertFromABSToZeroToOne(double abs);

    void setHighHub(bool isHighHub);
    void outerLaunch();
    void innerLaunch();
    void wall();
    void fender();
    void byHumanPlayer();


    bool hoodZero;
    double targetHoodPos;
    double currentHoodPos;
    double desiredPos;
    double calculatedPower;

    bool isHigh;

    int readyShootLimit;

    //CHANGE MOTOr ID STUFF
    rev::CANSparkMax flyWheelLead = rev::CANSparkMax(shooterWheelLeadID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder flyWheelLeadEncoder = flyWheelLead.GetEncoder();
    rev::SparkMaxPIDController flyWheelLead_pidController = flyWheelLead.GetPIDController();

    rev::CANSparkMax flyWheelFollow = rev::CANSparkMax(shooterWheelFollowID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder shooterWheelFollowEncoder = flyWheelFollow.GetEncoder(); // what are the points of these if they aren't used?
    rev::SparkMaxPIDController shooterWheelFollow_pidController = flyWheelFollow.GetPIDController(); // what are the points of these if they aren't used?

    rev::CANSparkMax shooterHood = rev::CANSparkMax(shooterHoodID, rev::CANSparkMax::MotorType::kBrushless);
    frc::DutyCycleEncoder shooterHoodEncoder = frc::DutyCycleEncoder{1};
    frc2::PIDController hoodPID = frc2::PIDController{50,0,0};
    // rev::SparkMaxRelativeEncoder shooterHoodEncoder = shooterHood.GetEncoder();
    //rev::SparkMaxPIDController shooterHood_pidController = shooterHood.GetPIDController();

};