// #pragma once

// #include "Constants.h"

<<<<<<< Updated upstream
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
=======
// #include <frc/DriverStation.h>
// #include <frc/TimedRobot.h>
// #include <rev/CANSparkMax.h>
// #include <rev/SparkMaxPIDController.h>
// #include <rev/CANEncoder.h>
// #include <rev/SparkMaxLimitSwitch.h>
// #include <frc/smartdashboard/SmartDashboard.h>
>>>>>>> Stashed changes


// struct RobotData;

<<<<<<< Updated upstream
struct ShooterData
{

};
=======
// struct ShooterData
// {
//     bool readyShoot;
//     int targetVel;
//     bool wrongBallReady;
// };
>>>>>>> Stashed changes

// class Shooter
// {

<<<<<<< Updated upstream
public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
    void DisabledInit();

private:
    void updateData(const RobotData &robotData, ShooterData &shooterData);
    void manual(const RobotData &robotData, ShooterData &shooterData);
    void semiAuto(const RobotData &robotData, ShooterData &shooterData);

    void setShooterPID(rev::SparkMaxPIDController motor, int pidSlot, double p, double i, double d, double ff);

    //CHANGE MOTOr ID STUFF  (just outline lol don't take your life too seriously:))
    rev::CANSparkMax shooterWheel = rev::CANSparkMax(31, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder shooterWheelEncoder = shooterWheel.GetEncoder();
    rev::SparkMaxPIDController shooterWheel_pidController = shooterWheel.GetPIDController();


=======
//     public:
//         void RobotInit();
//         void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
//         void DisabledInit();

//     private:
//         void updateData(const RobotData &robotData, ShooterData &shooterData);
//         void manual(const RobotData &robotData, ShooterData &shooterData);
//         void semiAuto(const RobotData &robotData, ShooterData &shooterData);

//         void shooterWheelLeadInit();
//         void shooterWheelFollowInit();
//         void shooterHoodInit();

//         double getHoodPos();
//         double getWheelPos();
//         double getWheelVel();
//         double getHoodOffset();
//         bool getHoodLimitSwitch();

//         void setHood(double power);
//         void setWheel(double power);
//         void setHoodPos(double pos);
//         void setTurretPos(double pos);

//         bool hoodZero;

//         //CHANGE MOTOr ID STUFF
//         rev::CANSparkMax shooterWheelLead = rev::CANSparkMax(shooterWheelLeadID, rev::CANSparkMax::MotorType::kBrushless);
//         rev::SparkMaxRelativeEncoder shooterWheelLeadEncoder = shooterWheelLead.GetEncoder();
//         rev::SparkMaxPIDController shooterWheelLead_pidController = shooterWheelLead.GetPIDController();

//         rev::CANSparkMax shooterWheelFollow = rev::CANSparkMax(shooterWheelFollowID, rev::CANSparkMax::MotorType::kBrushless);
//         rev::SparkMaxRelativeEncoder shooterWheelFollowEncoder = shooterWheelFollow.GetEncoder();
//         rev::SparkMaxPIDController shooterWheelFollow_pidController = shooterWheelFollow.GetPIDController();

//         rev::CANSparkMax shooterHood = rev::CANSparkMax(shooterHoodID, rev::CANSparkMax::MotorType::kBrushless);
//         rev::SparkMaxRelativeEncoder shooterHoodEncoder = shooterHood.GetEncoder();
//         rev::SparkMaxPIDController shooterHood_pidController = shooterHood.GetPIDController();

//         rev::SparkMaxLimitSwitch hoodReverseLimit = shooterHood.GetReverseLimitSwitch((rev::SparkMaxLimitSwitch::LimitSwitchPolarity::kNormallyClosed));
>>>>>>> Stashed changes

// };