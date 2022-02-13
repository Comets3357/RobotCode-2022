#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>


struct RobotData;

struct ClimbData
{

};

class Climb
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, ClimbData &climbData);    
    void DisabledInit();

private:
    void updateData(const RobotData &robotData, ClimbData &climbData);
    void semiAuto(const RobotData &robotData, ClimbData &climbData);
    void manual(const RobotData &robotData, ClimbData &climbData);


    //CHANGE MOTOr ID STUFF  (just outline lol don't take your life too seriously:))
    // rev::CANSparkMax climb1 = rev::CANSparkMax(61, rev::CANSparkMax::MotorType::kBrushless);
    // rev::SparkMaxRelativeEncoder climb1Encoder = climb1.GetEncoder();
    // rev::SparkMaxPIDController climb1_pidController = climb1.GetPIDController();


    // //zeroing sensor
    // rev::SparkMaxLimitSwitch elevatorLimit = climbElevator.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
    // rev::SparkMaxLimitSwitch armsLimit = climbArms.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);

};