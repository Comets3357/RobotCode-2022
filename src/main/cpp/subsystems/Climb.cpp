#include "subsystems/Climb.h"
#include "RobotData.h"

void Climb::RobotInit()
{
    
   
}

void Climb::RobotPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    updateData(robotData, climbData);

}

void Climb::DisabledInit()
{
    
}

// updates encoder and gyro values
void Climb::updateData(const RobotData &robotData, ClimbData &climbData)
{
    
}
