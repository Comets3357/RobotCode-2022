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

    } else {
        semiAuto(robotData, shooterData);

    }
    else
    {
        manual(robotData, shooterData);
    }
    currentHoodPos = shooterHoodEncoder2.GetDistance();

}

void Shooter::semiAuto(const RobotData &robotData, ShooterData &shooterData){

    //SHOOTING
    if(robotData.controlData.saShooting){
        if(robotData.limelightData.yOffset < 5){
            shooterData.targetVel = 1000;
        }else{
            shooterData.targetVel = 2000;
        }

        //shooterWheelLead_pidController.SetReference(3400, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        double calculatedPower = shooterHoodPID.Calculate(currentHoodPos, robotData.limelightData.desiredHoodPos);

        shooterHood.Set(calculatedPower);
        setWheel(0.6);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > robotData.shooterData.targetVel) && (std::abs(getHoodPos() - robotData.limelightData.desiredHoodPos) <= 2) ){
            shooterData.readyShoot = true;
        }else{
            shooterData.readyShoot = false;
        }
        hoodZero = false;

    }else if(robotData.controlData.launchPadShot){ //FROM THE LAUNCH PAD
        shooterWheelLead_pidController.SetReference(2000, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        
        double calculatedPower = shooterHoodPID.Calculate(currentHoodPos, 5);
        shooterHood.Set(calculatedPower);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > robotData.shooterData.targetVel) && (std::abs(getHoodPos()-4) <= .5)){
            shooterData.readyShoot = true;
        }else{
            shooterData.readyShoot = false;
        }
        hoodZero = false;

    }else if(robotData.controlData.hubShot){ //FROM THE HUB
        shooterWheelLead_pidController.SetReference(1200, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        
        double calculatedPower = shooterHoodPID.Calculate(currentHoodPos, 5);
        shooterHood.Set(calculatedPower);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > robotData.shooterData.targetVel) && (std::abs(getHoodPos()-4) <= .5)){
            shooterData.readyShoot = true;
        }else{
            shooterData.readyShoot = false;
        }
        hoodZero = false;

    }else if(robotData.controlData.wrongBall){ //IF THERES A WRONG BALL
        shooterWheelLead_pidController.SetReference(1200, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        
        double calculatedPower = shooterHoodPID.Calculate(currentHoodPos, 5);
        shooterHood.Set(calculatedPower);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > 1000)){
            shooterData.wrongBallReady = true;
        }else{
            shooterData.wrongBallReady = false;
        }
        hoodZero = false;

    }else{

        shooterData.readyShoot = false;
        shooterData.wrongBallReady = false;

        if(robotData.controlData.mFlyWheel){
            shooterWheelLead_pidController.SetReference(3400, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0); //uses second pid
        }else{
            // if(getWheelVel() < 1200){ //once the flywheel reaches a low enough velocity begin constant velociy
            //     shooterWheelLead_pidController.SetReference(1000, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1); //uses second pid
            // }else{
            //     setWheel(0); //starts the shooting wheel slowing down
            // }
        }

        double calculatedPower = shooterHoodPID.Calculate(currentHoodPos, 0);
        shooterHood.Set(calculatedPower);

    }
}

void Shooter::manual(const RobotData &robotData, ShooterData &shooterData){
    // if(robotData.controlData.mFlyWheel){
    //     //spins the flywheel up beforehand
    //     // setWheel();
    //     //shooterWheelLead_pidController.SetReference(2000, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
    // }else{
    //     setWheel(0);
    // }

    setWheel(robotData.controlData.mFlyWheel);
    setHood(robotData.controlData.mHood*.1);

    if(robotData.controlData.mzeroing){
        shooterHoodEncoder.SetPosition(0);
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
    frc::SmartDashboard::PutNumber("shooter wheel speed", shooterWheelLead.Get());
    frc::SmartDashboard::PutNumber("shooter Hood ABS", shooterHoodEncoder2.GetDistance());

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

    shooterWheelLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // //Slow
    // shooterWheelLead_pidController.SetP(0.74,0);
    // shooterWheelLead_pidController.SetI(0,0);
    // shooterWheelLead_pidController.SetD(0.2,0);
    // shooterWheelLead_pidController.SetIZone(0,0);
    // shooterWheelLead_pidController.SetFF(0,0);
    // shooterWheelLead_pidController.SetOutputRange(-0.205, 0.16,0);

    // //Fast
    // shooterWheelLead_pidController.SetP(0.99,1);
    // shooterWheelLead_pidController.SetI(0,1);
    // shooterWheelLead_pidController.SetD(0.3,1);
    // shooterWheelLead_pidController.SetIZone(0,1);
    // shooterWheelLead_pidController.SetFF(0,1);
    // shooterWheelLead_pidController.SetOutputRange(-.2, 0.15,1);



    shooterWheelLead.SetSmartCurrentLimit(45);
}

void Shooter::shooterWheelFollowInit(){
    shooterWheelFollow.RestoreFactoryDefaults();
    shooterWheelFollow.Follow(shooterWheelLead, true);
    shooterWheelFollow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    shooterWheelFollow.SetSmartCurrentLimit(45);

    shooterWheelFollow.Follow(shooterWheelLead);

}

void Shooter::shooterHoodInit()
{
    shooterHood.RestoreFactoryDefaults();
    shooterHood.SetInverted(true);
    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // shooterWheelLead_pidController.SetP(mkP);
    // shooterWheelLead_pidController.SetI(mkI);
    // shooterWheelLead_pidController.SetD(mkD);
    // shooterWheelLead_pidController.SetIZone(mkIz);
    // shooterWheelLead_pidController.SetFF(mkFF);
    // shooterWheelLead_pidController.SetOutputRange(mkMinOutput, mkMaxOutput);

    shooterHood.SetSmartCurrentLimit(45);
}

void Shooter::rescaledHood(double hoodPos){
    targetHoodPos = minHoodExtend + ((maxHoodExtend-minHoodExtend)*hoodPos);
}






