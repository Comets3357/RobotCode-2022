#include "RobotData.h"
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * CLASS SPECIFIC INITS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::RobotInit()
{
    flyWheelInit();
    shooterHoodInit();

    flyWheelLead.Set(0);
    shooterHood.Set(0);

    isHigh = true;
}

void Shooter::shooterHoodInit()
{
    shooterHood.RestoreFactoryDefaults();
    shooterHood.SetInverted(true);
    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shooterHood.SetSmartCurrentLimit(15);

    shooterHood_pidController.SetP(0.27);
    shooterHood_pidController.SetI(0);
    shooterHood_pidController.SetD(0);
    shooterHood_pidController.SetIZone(0);
    shooterHood_pidController.SetFF(0);
    shooterHood_pidController.SetOutputRange(-0.5,0.5);

    shooterHood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 2);
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

    flyWheelLead_pidController.SetP(0.0004);
    flyWheelLead_pidController.SetI(0);
    flyWheelLead_pidController.SetD(0);
    flyWheelLead_pidController.SetIZone(0);
    flyWheelLead_pidController.SetFF(0.00025);
    flyWheelLead_pidController.SetOutputRange(0,1);
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

    if (robotData.controlData.mode == mode_teleop_manual)
    {
        manual(robotData, shooterData);
    }
    else if (robotData.controlData.mode == mode_teleop_sa)
    {
        semiAuto(robotData, shooterData);
    }

    //constantly updates rev hood pos with more accurate abs encoder values
    if(tickCount > 40){
        shooterHoodEncoderRev.SetPosition(absoluteToREV(shooterHoodEncoderAbs.GetOutput()));
        tickCount = (tickCount+1)%50;
    }else{
        tickCount = (tickCount+1)%50;
    }
    
    //idk about this control data stuff to figure out
    if(robotData.controlData.upperHubShot){
        setHighHub();
    }
}

void Shooter::semiAuto(const RobotData &robotData, ShooterData &shooterData){

    //idk about this control data stuff to figure out
    if(robotData.controlData.saShooting){ // Aiming SHOOTING with limelight

        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

        //once it's a high enough velocity its ready for indexer to run
        if ((getWheelVel() > 1450))
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }

    }else if(robotData.controlData.cornerLaunchPadShot){ //FROM THE CLOSER LAUNCH PAD
        innerLaunch();
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() + 38) <= 1)**/) //dont know why but it wasnt working so commented it out
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }

    }
    else if (robotData.controlData.wallLaunchPadShot) //FROM THE FARTHER LAUNCH PAD
    {
        outerLaunch();
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() + 42) <= 1)**/)
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }
    }
    else if(robotData.controlData.fenderShot) //FROM THE FENDER FIXED SHOT
    {
        fender();
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() - 0) <= 1)**/)
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }
    } 
    else if (robotData.controlData.sideWallShot) //FROM THE SIDE WALL FIXED SHOT
    {
        wall();
        if ((getWheelVel() > readyShootLimit) /** && (std::abs(getHoodPos() +38) <= 1)**/)
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }
    }
    else //IF NO SHOOTING DON'T DO ANYTHING
    {
        shooterData.readyShoot = false;
        shooterData.wrongBallReady = false;

        flyWheelLead.Set(0);

        //if the hood is too far out bring it in then stop the hood from running
        if(shooterHoodEncoderRev.GetPosition() < -3){
            shooterHood_pidController.SetReference(-2,rev::CANSparkMaxLowLevel::ControlType::kPosition);
        }else{
            shooterHood.Set(0);
        }

    }
}

void Shooter::manual(const RobotData &robotData, ShooterData &shooterData){
    
    //manual wheel forward
    if(robotData.controlData.mShooterWheelForward){
        flyWheelLead_pidController.SetReference(2000, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
    }else{
        flyWheelLead.Set(0); //starts the shooting wheel slowing down
    }

    //hood to joystick controls
    shooterHood.Set(robotData.controlData.mHood*.2);

    //zeros hood pos
    if(robotData.controlData.mZeroHood)
    {
        shooterHoodEncoderRev.SetPosition(0);
    }

    // if(!hoodZero){
    //     if(getHoodPos() > 4){
    //         shooterHood_pidController.SetReference(4, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    //     }else{
    //         setHood(-0.2);
    //         if(getHoodLimitSwitch()){
    //             setHoodPos(0);
    //             setHood(0);
    //             hoodZero = true;
    //         }
    //     } 
    // }

    // min is greater than max w/ absolute encoder 
    // if (shooterHoodEncoder.GetDistance() <= maxHoodExtend)
    // {
    //     setHoodPos(maxHoodExtend);
    // } 
    // else if (shooterHoodEncoder.GetDistance() >= minHoodExtend) 
    // {
    //     setHoodPos(minHoodExtend);
    // }
    // else 
    // {
    //     if (robotData.controlData.mHood > .08){
    //         setHoodPos(targetHoodPos + .025);
    //     } else if (robotData.controlData.mHood < -.08){
    //         setHoodPos(targetHoodPos - .025);
    //     }
    // }

}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * UPDATING DATA TO SMARTDASHBOARD
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
// updates encoder and gyro values
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData)
{
    frc::SmartDashboard::PutNumber("shooter Hood ABS", shooterHoodEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("shooter Hood REV", shooterHoodEncoderRev.GetPosition());
    frc::SmartDashboard::PutNumber("shooter changed", absoluteToREV(shooterHoodEncoderAbs.GetOutput()));
    frc::SmartDashboard::PutBoolean("shooter ready shoot", shooterData.readyShoot);

    //frc::SmartDashboard::PutNumber("shooter Hood zero to one scale", getHoodPos());

}
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * COMMON FUNCTIONS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
double Shooter::getWheelVel(){
    return flyWheelLeadEncoder.GetVelocity();
}

// void Shooter::setHoodPos(double pos) 
// {
//     // takes a 0 - 1 value to update a member to move hood to set point
//     targetHoodPos = maxHoodExtend + ((minHoodExtend-maxHoodExtend)*pos);
// }

// double Shooter::getHoodPos()
// {
//     // takes the absolute encoder position and converts it back to 0 to 1 scale for ease of reading
//     return (currentHoodPos - maxHoodExtend) / (minHoodExtend - maxHoodExtend);
// }

// double Shooter::convertFromABSToZeroToOne(double abs)
// {
//     return (abs - maxHoodExtend) / (minHoodExtend - maxHoodExtend);
// }


/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * FIXED SHOTS FOR BOTH HIGHER AND LOWER
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::outerLaunch()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(42, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(2050, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 2000;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(42, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1900, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1850;
    }
}

void Shooter::innerLaunch()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(-38.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1800, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1750;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(-38.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1800, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1750;
    }
}

void Shooter::wall()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(-38, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1700, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1650;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(-38, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1700, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1650;
    }
}

void Shooter::fender()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1350, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1300;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1350, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1300;
    }
}

//currently not integrated into controller data
void Shooter::endOfTarmac()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(-22, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1350;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(-22, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1450;
    }
}

//toggles if shooting to high hub based on controller data
void Shooter::setHighHub()
{
    if (isHigh)
    {
        isHigh = false;
    }
    else if (!isHigh)
    {
        isHigh = true;
    }
}

/**
 * @return converts from the absolute encoder values to ones the rev motor can read
 * constantly updates the rev position in periodic 
 **/
double Shooter::absoluteToREV(double value){
    double slope = (hoodrevOut - hoodrevIn)/(hoodabsOut - hoodabsIn);
    double b = hoodrevIn - (slope*hoodabsIn);
    return ((value*slope) + b);
    //return (value*55.8 + -41.4;
    // can we do this based on constants? and then 
}
