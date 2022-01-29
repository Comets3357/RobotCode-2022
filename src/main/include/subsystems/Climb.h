#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <rev/CANDigitalInput.h>


struct RobotData;

struct ClimbData {
    int bar;
    bool climbing;
};

class Climb {

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, ClimbData &climbData);    
    void DisabledInit();

private:

    int stage = 0;

    bool climbInitiating = false;
    bool climbUp = false;
    bool executeSemiAuto = false;
    int targetBar = 0;

    bool elevatorDirection; //True is positive, False is negative
    bool elevatorRunning = false;

    bool armsDirection; //True is positive, False is negative
    bool armsRunning = false;

    void updateData(const RobotData &robotData, ClimbData &climbData);
    void semiAuto(const RobotData &robotData, ClimbData &climbData);
    void manual(const RobotData &robotData, ClimbData &climbData);

    void RunClimbToPos(int position, float power, int stageAdd);
    void RunArmsToPos(int position, float power, int stageAdd);


    //CHANGE MOTOr ID STUFF  (just outline lol don't take your life too seriously:))
    //initualizes climb elevator motor
    rev::CANSparkMax climbElevator = rev::CANSparkMax(41, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder climbElevatorEncoder = climbElevator.GetEncoder();
    rev::SparkMaxPIDController climbElevator_pidController = climbElevator.GetPIDController();

    //initualizes climb arms motor (i dont know if there are 2 motors yet)
    rev::CANSparkMax climbArms = rev::CANSparkMax(41, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder climbArmsEncoder = climbArms.GetEncoder();
    rev::SparkMaxPIDController climbArms_pidController = climbArms.GetPIDController();

    //zeroing sensor
    rev::SparkMaxLimitSwitch climbLimit = climbElevator.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen);


};