#include "subsystems/Shooter.h"
#include "RobotData.h"

void Shooter::RobotInit()
{
    
   
}

void Shooter::RobotPeriodic(const RobotData &robotData, ShooterData &shooterData)
{
    updateData(robotData, shooterData);
    if (robotData.controlData.manualMode)
    {
        manual(robotData, shooterData);
    }
    else
    {
        semiAuto(robotData, shooterData);
    }
    

}

void Shooter::semiAuto(const RobotData &robotData, ShooterData &shooterData){

}

void Shooter::manual(const RobotData &robotData, ShooterData &shooterData){
    
}

void Shooter::setShooterPID(rev::SparkMaxPIDController motor, int pidSlot, double p, double i, double d, double ff)
{
    motor.SetP(p, pidSlot);
    motor.SetI(i, pidSlot);
    motor.SetD(d, pidSlot);
    motor.SetFF(ff, pidSlot);
}


void Shooter::DisabledInit()
{
    
}

// updates encoder and gyro values
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData)
{
    
}

