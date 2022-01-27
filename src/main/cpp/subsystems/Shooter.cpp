#include "RobotData.h"
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * CLASS SPECIFIC INITS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::RobotInit()
{
    Shooter::flyWheelInit();
    Shooter::shooterHoodInit();

    flyWheelLead.Set(0);
    shooterHood.Set(0);

    isHigh = true;
    
    frc::SmartDashboard::PutNumber("target hood", 0);
}

void Shooter::shooterHoodInit()
{
    shooterHood.RestoreFactoryDefaults();
    shooterHood.SetInverted(true);
    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shooterHood.SetSmartCurrentLimit(45);

    // shooterWheelLead_pidController.SetP(mkP);
    // shooterWheelLead_pidController.SetI(mkI);
    // shooterWheelLead_pidController.SetD(mkD);
    // shooterWheelLead_pidController.SetIZone(mkIz);
    // shooterWheelLead_pidController.SetFF(mkFF);
    // shooterWheelLead_pidController.SetOutputRange(mkMinOutput, mkMaxOutput);
}

void Shooter::flyWheelInit()
{
    // fly wheel LEAD motor init
    flyWheelLead.RestoreFactoryDefaults();
    flyWheelLead.SetInverted(false);
    flyWheelLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flyWheelLead.SetSmartCurrentLimit(45);

    // fly wheel FOLLOW motor init
    flyWheelFollow.RestoreFactoryDefaults();
    flyWheelFollow.Follow(flyWheelLead, true);
    flyWheelFollow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flyWheelFollow.SetSmartCurrentLimit(45);

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
}

void Shooter::DisabledInit()
{
    shooterHood.Set(0);
    flyWheelLead.Set(0);
    flyWheelFollow.Set(0);
}
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * PERIODIC AND DRIVER CONTROL FUNCTIONS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
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
    
    currentHoodPos = shooterHoodEncoder.GetDistance();

    // desiredPos = frc::SmartDashboard::GetNumber("target hood", 0);

    calculatedPower = hoodPID.Calculate(currentHoodPos, targetHoodPos);

    shooterHood.Set(calculatedPower);

}

void Shooter::semiAuto(const RobotData &robotData, ShooterData &shooterData){

    //SHOOTING
    if(robotData.controlData.saShooting)
    {
        if(robotData.limelightData.yOffset < 5)
        {
            shooterData.targetVel = 1000;
        }
        else
        {
            shooterData.targetVel = 2000;
        }

        //shooterWheelLead_pidController.SetReference(3400, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        calculatedPower = hoodPID.Calculate(currentHoodPos, targetHoodPos);

        shooterHood.Set(calculatedPower);
        setWheel(0.4);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > 1200) /**&& (std::abs(getHoodPos() - robotData.limelightData.desiredHoodPos) <= 2)**/ )
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }
        hoodZero = false;

    }
    else
    {

        shooterData.readyShoot = false;
        shooterData.wrongBallReady = false;

        if(robotData.controlData.mFlyWheel)
        {
            flyWheelLead_pidController.SetReference(3400, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0); //uses second pid
        }
        else
        {
            // if(getWheelVel() < 1200){ //once the flywheel reaches a low enough velocity begin constant velociy
            //     shooterWheelLead_pidController.SetReference(1000, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1); //uses second pid
            // }else{
            //     setWheel(0); //starts the shooting wheel slowing down
            // }
        }

        calculatedPower = hoodPID.Calculate(currentHoodPos, 0);
        shooterHood.Set(calculatedPower);

    }
}

void Shooter::manual(const RobotData &robotData, ShooterData &shooterData)
{
    // if(robotData.controlData.mFlyWheel){
    //     //spins the flywheel up beforehand
    //     // setWheel();
    //     //shooterWheelLead_pidController.SetReference(2000, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
    // }else{
    //     setWheel(0);
    // }

    //setWheel(robotData.controlData.mFlyWheel);

    if (shooterHoodEncoder.GetDistance() >= maxHoodExtend || shooterHoodEncoder.GetDistance() <= minHoodExtend)
    {
        setHood(0);
    }
    else 
    {
        setHood(robotData.controlData.mHood*.1);
    }

    if(robotData.controlData.mzeroing)
    {
        setHoodPos(0);
    }

}
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * UPDATING DATA TO SMARTDASHBOARD
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
// updates encoder and gyro values
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData)
{
    frc::SmartDashboard::PutNumber("shooter wheel vel", flyWheelLeadEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("shooter wheel speed", flyWheelLead.Get());
    frc::SmartDashboard::PutNumber("shooter Hood ABS", shooterHoodEncoder.GetDistance());
    frc::SmartDashboard::PutBoolean("shooter ready shoot", shooterData.readyShoot);
    frc::SmartDashboard::PutNumber("shooter Hood zero to one scale", getHoodPos());
}
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * COMMON FUNCTIONS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::setHoodPos(double pos) 
{
    // takes a 0 - 1 value to update a member to move hood to set point
    targetHoodPos = minHoodExtend + ((maxHoodExtend-minHoodExtend)*pos);
}

double Shooter::getHoodPos()
{
    // takes the absolute encoder position and converts it back to 0 to 1 scale for ease of reading
    return (currentHoodPos - minHoodExtend) / (maxHoodExtend - minHoodExtend);
}

double Shooter::getWheelPos()
{
    // gets the fly wheel position 
    return flyWheelLeadEncoder.GetPosition();
} 

void Shooter::setHood(double power)
{
    // ONLY USED FOR MANUAL CONTROL OF HOOD FOR DRIVER, AND NOTHING ELSE
    shooterHood.Set(power);
}

void Shooter::setWheel(double power)
{
    // sets wheel power
    flyWheelLead.Set(power);
}

double Shooter::getWheelVel()
{
    // gets wheel velocity
    return flyWheelLeadEncoder.GetVelocity();
}

double Shooter::convertFromABSToZeroToOne(double abs)
{
    return (abs - minHoodExtend) / (maxHoodExtend - minHoodExtend);
}
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * FIXED SHOTS FOR BOTH HIGHER AND LOWER
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */

void Shooter::outerLaunch()
{
    if (isHigh)
    {
        setHoodPos(0);;
    }
    else if (!isHigh)
    {
        setHoodPos(0);
    }
}

void Shooter::innerLaunch()
{
    if (isHigh)
    {
        setHoodPos(0);;
    }
    else if (!isHigh)
    {
        setHoodPos(0);
    }
}

void Shooter::wall()
{
    if (isHigh)
    {
        setHoodPos(0);;
    }
    else if (!isHigh)
    {
        setHoodPos(0);
    }
}

void Shooter::fender()
{
    if (isHigh)
    {
        setHoodPos(0);;
    }
    else if (!isHigh)
    {
        setHoodPos(0);
    }
}

void Shooter::setHighHub(bool isHighHub)
{
    isHigh = isHighHub;
}

