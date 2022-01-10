#include "subsystems/Intake.h"
#include "RobotData.h"

void Intake::RobotInit()
{
    
   
}

void Intake::RobotPeriodic(const RobotData &robotData, IntakeData &intakeData)
{
    updateData(robotData, intakeData);

}

void Intake::DisabledInit()
{
    
}

// updates encoder and gyro values
void Intake::updateData(const RobotData &robotData, IntakeData &intakeData)
{
    
}

