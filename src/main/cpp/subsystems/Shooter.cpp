#include "subsystems/Shooter.h"
#include "RobotData.h"

void Shooter::RobotInit()
{
    
   
}

void Shooter::RobotPeriodic(const RobotData &robotData, ShooterData &shooterData)
{
    updateData(robotData, shooterData);

}

void Shooter::DisabledInit()
{
    
}

// updates encoder and gyro values
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData)
{
    
}

