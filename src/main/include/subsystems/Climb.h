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

    int stage = 0;
    int bar = 2;

    bool elevatorDirection; //True is positive, False is negative
    bool elevatorRunning = false;

    bool armsDirection; //True is positive, False is negative
    bool armsRunning = false;

    void updateData(const RobotData &robotData, ClimbData &climbData);
    void semiAuto(const RobotData &robotData, ClimbData &climbData);
    void manual(const RobotData &robotData, ClimbData &climbData);

    void RunClimbToPos(int position, float power);
    void RunArmsToPos(int position, float power);


    //CHANGE MOTOr ID STUFF  (just outline lol don't take your life too seriously:))
    //initualizes climb elevator motor
    rev::CANSparkMax climbElevator = rev::CANSparkMax(41, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder climbElevatorEncoder = climbElevator.GetEncoder();
    rev::SparkMaxPIDController climbElevator_pidController = climbElevator.GetPIDController();

    //initualizes climb arms motor (i dont know if there are 2 motors yet)
    rev::CANSparkMax climbArms = rev::CANSparkMax(41, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder climbArmsEncoder = climbArms.GetEncoder();
    rev::SparkMaxPIDController climbArms_pidController = climbArms.GetPIDController();



};