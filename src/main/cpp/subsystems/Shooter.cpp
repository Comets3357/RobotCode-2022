#include "subsystems/Shooter.h"
#include "RobotData.h"

void Shooter::RobotInit()
{
    shooterWheelLeadInit();
    shooterWheelFollowInit();
    shooterHoodInit();

    shooterWheelLead.Set(0);
    shooterHood.Set(0);
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
    shooterHood.Set(0);
    shooterWheelLead.Set(0);
    shooterWheelFollow.Set(0);
}

// updates encoder and gyro values
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData)
{
    
}

void Shooter::shooterWheelLeadInit(){
    shooterWheelLead.RestoreFactoryDefaults();

    shooterWheelLead.SetInverted(false);

    shooterWheelLead.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // shooterWheelLead_pidController.SetP(mkP);
    // shooterWheelLead_pidController.SetI(mkI);
    // shooterWheelLead_pidController.SetD(mkD);
    // shooterWheelLead_pidController.SetIZone(mkIz);
    // shooterWheelLead_pidController.SetFF(mkFF);
    // shooterWheelLead_pidController.SetOutputRange(mkMinOutput, mkMaxOutput);

    shooterWheelLead.SetSmartCurrentLimit(45);
}

void Shooter::shooterWheelFollowInit(){
    shooterWheelFollow.RestoreFactoryDefaults();

    shooterWheelFollow.SetInverted(false);

    shooterWheelFollow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    shooterWheelFollow.SetSmartCurrentLimit(45);

    shooterWheelFollow.Follow(shooterWheelLead);

}

void Shooter::shooterHoodInit(){
    shooterHood.RestoreFactoryDefaults();

    shooterHood.SetInverted(false);

    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // shooterWheelLead_pidController.SetP(mkP);
    // shooterWheelLead_pidController.SetI(mkI);
    // shooterWheelLead_pidController.SetD(mkD);
    // shooterWheelLead_pidController.SetIZone(mkIz);
    // shooterWheelLead_pidController.SetFF(mkFF);
    // shooterWheelLead_pidController.SetOutputRange(mkMinOutput, mkMaxOutput);

    shooterHood.SetSmartCurrentLimit(45);
}

