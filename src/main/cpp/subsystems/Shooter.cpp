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

    readyShootLimit = 1200;

    flyWheelLead_pidController.SetP(0.0004,0);
    flyWheelLead_pidController.SetI(0,0);
    flyWheelLead_pidController.SetD(0,0);
    flyWheelLead_pidController.SetIZone(0,0);
    flyWheelLead_pidController.SetFF(0.00025,0);
    flyWheelLead_pidController.SetOutputRange(0,1,0);
    flyWheelLead.BurnFlash();
    flyWheelFollow.BurnFlash();
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
    currentHoodPos = shooterHoodEncoder.GetDistance();
    if (robotData.controlData.manualMode)
    {
        manual(robotData, shooterData);

    } 
    else 
    {
        semiAuto(robotData, shooterData);
    }
    
    

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
        // calculatedPower = hoodPID.Calculate(currentHoodPos, targetHoodPos);

        // shooterHood.Set(calculatedPower); // repetetive
        setWheel(0.4);

        //once the shooter has high enough velocity (and is aimed correctly tell robot to begin shooting)
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() - robotData.limelightData.desiredHoodPos) <= 2)**/ )
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

    // min is greater than max w/ absolute encoder 
    if (shooterHoodEncoder.GetDistance() <= maxHoodExtend)
    {
        setHoodPos(maxHoodExtend);
    } 
    else if (shooterHoodEncoder.GetDistance() >= minHoodExtend) 
    {
        setHoodPos(minHoodExtend);
    }
    else 
    {
        if (robotData.controlData.mHood > .08){
            setHoodPos(targetHoodPos + .025);
        } else if (robotData.controlData.mHood < -.08){
            setHoodPos(targetHoodPos - .025);
        }
    }

    if(robotData.controlData.mzeroing)
    {
        setHoodPos(0); /// not interfering with absolute/rev shit?
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
    targetHoodPos = maxHoodExtend + ((minHoodExtend-maxHoodExtend)*pos);
}

double Shooter::getHoodPos()
{
    // takes the absolute encoder position and converts it back to 0 to 1 scale for ease of reading
    return (currentHoodPos - maxHoodExtend) / (minHoodExtend - maxHoodExtend);
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
    return (abs - maxHoodExtend) / (minHoodExtend - maxHoodExtend);
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
        setHoodPos(0);
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
        setHoodPos(convertFromABSToZeroToOne(0.38)); // 3.2 inches out
        flyWheelLead_pidController.SetReference(1800, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 1700;
    }
    else if (!isHigh)
    {
        setHoodPos(convertFromABSToZeroToOne(0.34));
        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 1450;
    }
}

void Shooter::wall()
{
    if (isHigh)
    {
        setHoodPos(convertFromABSToZeroToOne(0.716)); // 2 inches out
        flyWheelLead_pidController.SetReference(1800, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 1700;
    }
    else if (!isHigh)
    {
        setHoodPos(0.9);
        flyWheelLead_pidController.SetReference(1400, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 1300;
    }
}

void Shooter::fender()
{
    if (isHigh)
    {
        setHoodPos(0);
        flyWheelLead_pidController.SetReference(1300, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 1250;
    }
    else if (!isHigh)
    {
        setHoodPos(1);
        flyWheelLead_pidController.SetReference(500, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 450;
    }
}

void Shooter::byHumanPlayer()
{
    if (isHigh)
    {
        setHoodPos(1);
        flyWheelLead_pidController.SetReference(2700, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 2600;
    }
    else if (!isHigh)
    {
        setHoodPos(1);
        flyWheelLead_pidController.SetReference(2200, rev::CANSparkMaxLowLevel::ControlType::kVelocity,0);
        readyShootLimit = 2100;
    }
}

void Shooter::setHighHub(bool isHighHub)
{
    isHigh = isHighHub;
}