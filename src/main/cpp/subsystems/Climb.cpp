#include "subsystems/Climb.h"
#include "RobotData.h"

void Climb::RobotInit()
{
    
   
}

void Climb::RobotPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    updateData(robotData, climbData);
    if (robotData.controlData.manualMode)
    {
        manual(robotData, climbData);
    }
    else
    {
        semiAuto(robotData, climbData);
    }

}

void Climb::manual(const RobotData &robotData, ClimbData &climbData){

}

void Climb::semiAuto(const RobotData &robotData, ClimbData &climbData){

}

void Climb::DisabledInit()
{
    
}

// updates encoder and gyro values
void Climb::updateData(const RobotData &robotData, ClimbData &climbData)
{
    
}
