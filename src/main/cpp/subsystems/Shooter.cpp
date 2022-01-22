#include "subsystems/Shooter.h"
#include "RobotData.h"

void Shooter::RobotInit()
{
    Shooter::shooterWheelLeadInit();
    Shooter::shooterWheelFollowInit();
    Shooter::shooterHoodInit();

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
    if(robotData.controlData.saShooting){
        if(robotData.limelightData.yOffset < 5){
            shooterData.targetVel = 1000;
        }else{
            shooterData.targetVel = 2000;
        }

        //shooterWheelLead_pidController.SetReference(3400, rev::ControlType::kVelocity);
        setWheel(0.6);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > robotData.shooterData.targetVel) /**&& (std::abs(getTurretPos() - (turretSnapshot + robotData.calcTurretPos)) <= 1) && (std::abs(getHoodPos() - robotData.calcHoodPos) <= 2)**/ ){
            shooterData.readyShoot = true;
        }else{
            shooterData.readyShoot = false;
        }
        hoodZero = false;
    }else if(robotData.controlData.launchPadShot){
        shooterWheelLead_pidController.SetReference(2000, rev::ControlType::kVelocity);
        shooterHood_pidController.SetReference(4, rev::ControlType::kPosition);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > robotData.shooterData.targetVel) && (std::abs(getHoodPos()-4) <= .5)){
            shooterData.readyShoot = true;
        }else{
            shooterData.readyShoot = false;
        }
        hoodZero = false;

    }else if(robotData.controlData.hubShot){
        shooterWheelLead_pidController.SetReference(1200, rev::ControlType::kVelocity);
        shooterHood_pidController.SetReference(4, rev::ControlType::kPosition);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > robotData.shooterData.targetVel) && (std::abs(getHoodPos()-4) <= .5)){
            shooterData.readyShoot = true;
        }else{
            shooterData.readyShoot = false;
        }
        hoodZero = false;
    }else if(robotData.controlData.wrongBall){
        shooterWheelLead_pidController.SetReference(1200, rev::ControlType::kVelocity);
        shooterHood_pidController.SetReference(2, rev::ControlType::kPosition);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > robotData.shooterData.targetVel)){
            shooterData.wrongBallReady = true;
        }else{
            shooterData.wrongBallReady = false;
        }
        hoodZero = false;

    }else{

        shooterData.readyShoot = false;
        shooterData.wrongBallReady = false;
        if(!hoodZero){
            if(getHoodPos() > 4){
                shooterHood_pidController.SetReference(4, rev::ControlType::kPosition);
            }else{
                setHood(-0.2);
                if(getHoodLimitSwitch()){
                    setHoodPos(0);
                    setHood(0);
                    hoodZero = true;
                }
            } 
        }


    }
}

void Shooter::manual(const RobotData &robotData, ShooterData &shooterData){
    if(robotData.controlData.mFlyWheel){
        //spins the flywheel up beforehand
        setWheel(0.6);
        //shooterWheelLead_pidController.SetReference(2000, rev::ControlType::kVelocity);
    }else{
        setWheel(0);
        setHood(robotData.controlData.mHood*.1);
    }
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
    frc::SmartDashboard::PutNumber("shooter wheel vel", shooterWheelLeadEncoder.GetVelocity());

}

void Shooter::setHoodPos(double pos){
    shooterHoodEncoder.SetPosition(pos);
}

double Shooter::getHoodPos(){
    return shooterHoodEncoder.GetPosition();
}

double Shooter::getWheelPos(){
    return shooterWheelLeadEncoder.GetPosition();
} 

bool Shooter::getHoodLimitSwitch(){
    return hoodReverseLimit.Get();
}

void Shooter::setHood(double power){
    shooterHood.Set(power);
}

void Shooter::setWheel(double power){
    shooterWheelLead.Set(power);
}

double Shooter::getWheelVel(){
    return shooterWheelLeadEncoder.GetVelocity();
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





