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

    //used for reading flywheel speeds from the dashboard
    frc::SmartDashboard::PutNumber("wheel speed", 0);

}

void Shooter::shooterHoodInit()
{
    shooterHood.RestoreFactoryDefaults();
    shooterHood.SetInverted(true);
    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shooterHood.SetSmartCurrentLimit(15);

    //PIDS
    shooterHood_pidController.SetP(0.378); //0.193 
    shooterHood_pidController.SetI(0);
    shooterHood_pidController.SetD(0);
    shooterHood_pidController.SetIZone(0);
    shooterHood_pidController.SetFF(0);
    shooterHood_pidController.SetOutputRange(-0.5,0.5);

    shooterHood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 2);
    shooterHood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -38);
}

void Shooter::flyWheelInit()
{
    // fly wheel LEAD motor init
    flyWheelLead.RestoreFactoryDefaults();
    flyWheelLead.SetInverted(true);
    flyWheelLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flyWheelLead.SetSmartCurrentLimit(45);

    readyShootLimit = 1200;

    //PIDS
    flyWheelLead_pidController.SetP(0.0005); //0.002
    flyWheelLead_pidController.SetI(0);
    flyWheelLead_pidController.SetD(0); //0.005
    flyWheelLead_pidController.SetIZone(0);
    flyWheelLead_pidController.SetFF(0.000205); //0.0002
    flyWheelLead_pidController.SetOutputRange(0,1);
    flyWheelLead.BurnFlash();           
}

void Shooter::DisabledInit()
{
    shooterHood.Set(0);
    flyWheelLead.Set(0);
}

void Shooter::EnabledInit(ShooterData &shooterData)
{
    shooterData.shootUnassignedAsOpponent = false;
}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * PERIODIC AND DRIVER CONTROL FUNCTIONS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::RobotPeriodic(const RobotData &robotData, ShooterData &shooterData)
{
    updateData(robotData, shooterData);

    if(robotData.controlData.mode == mode_climb_manual || robotData.controlData.mode == mode_climb_sa){
        //shooterHood_pidController.SetReference(0,rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead.Set(0);
        shooterHood.Set(0);

    }else{
        if (robotData.controlData.mode == mode_teleop_manual)
        {
            manual(robotData, shooterData);
        }
        else if (robotData.controlData.mode == mode_teleop_sa)
        {
            semiAuto(robotData, shooterData);
        }

    }
    

}

void Shooter::semiAuto(const RobotData &robotData, ShooterData &shooterData){

    //checks if encoder is functioning, if it is constantly update rev encoder, otherwise don't
    if(shooterHoodEncoderAbs.GetOutput() > 0.03)
    {
        // constantly updates rev hood pos with more accurate abs encoder values
        if(tickCount > 40){
            shooterHoodEncoderRev.SetPosition(absoluteToREV(shooterHoodEncoderAbs.GetOutput()));
            tickCount = (tickCount+1)%50;
        }else{
            tickCount = (tickCount+1)%50;
        }

    }else{
        // shooterHood.Set(0);
        // flyWheelLead.Set(0);
    }

    //all the shooting logic
    if(robotData.controlData.shootMode == shootMode_vision){ // Aiming with limelight
        //set the hood and flywheel using pids to the desired values based off the limelight code
        flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        shooterHood_pidController.SetReference(absoluteToREV(convertFromAngleToAbs(robotData.limelightData.desiredHoodPos)), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        
        //retrieves flywheel speeds from the dashboard
        double hi = frc::SmartDashboard::GetNumber("wheel speed", 0);
        
        //FOR TESTING PURPOSES
        // flyWheelLead_pidController.SetReference(hi, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        // if (getWheelVel() > (hi - 50))

        //once it's a high enough velocity its ready for indexer to run
        if (getWheelVel() > (robotData.limelightData.desiredVel - 15))
        // if(getWheelVel() > 1950)
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }

//FIXED SHOTS 
    }else if(robotData.controlData.shootMode == shootMode_cornerLaunchPad){ //FROM THE CLOSER LAUNCH PAD
        innerLaunch(robotData);
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() + 38) <= 1)**/) //dont know why but it wasnt working so commented it out
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }
    }
    else if (robotData.controlData.shootMode == shootMode_wallLaunchPad) //FROM THE FARTHER LAUNCH PAD
    {
        outerLaunch(robotData);
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() + 42) <= 1)**/)
        {
            shooterData.readyShoot = true;
        }else
        {
            shooterData.readyShoot = false;
        }
    }
    else if(robotData.controlData.shootMode == shootMode_fender) //FROM THE FENDER FIXED SHOT
    {
        fender(robotData);
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() - 0) <= 1)**/)
        {
            shooterData.readyShoot = true;
        }
        else
        {
            shooterData.readyShoot = false;
        }
    } 
    else if (robotData.controlData.shootMode == shootMode_sideWall) //FROM THE SIDE WALL FIXED SHOT
    {
        wall(robotData);
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() - 0) <= 1)**/)
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

void Shooter::manual(const RobotData &robotData, ShooterData &shooterData)
{
    
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

}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * UPDATING DATA TO SMARTDASHBOARD
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
// updates encoder and gyro values
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData)
{
    // frc::SmartDashboard::PutNumber("shooter Hood ABS", shooterHoodEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("shooter Hood REV", shooterHoodEncoderRev.GetPosition());
    //frc::SmartDashboard::PutNumber("shooter changed", absoluteToREV(shooterHoodEncoderAbs.GetOutput()));
    //frc::SmartDashboard::PutNumber("desired hood to rev", absoluteToREV(convertFromAngleToAbs(robotData.limelightData.desiredHoodPos)));
    frc::SmartDashboard::PutNumber("shootMode", robotData.controlData.shootMode);
    frc::SmartDashboard::PutNumber("high or no", robotData.controlData.upperHubShot);

    frc::SmartDashboard::PutBoolean("shooter ready shoot", shooterData.readyShoot);
    frc::SmartDashboard::PutNumber("HOOD ANGLE", convertFromAbsToAngle(shooterHoodEncoderAbs.GetOutput()));
    frc::SmartDashboard::PutNumber("flywheel vel", flyWheelLeadEncoder.GetVelocity());


    frc::SmartDashboard::PutNumber("shooter vision", shootMode_vision);

    shooterData.isHighGeneral = isHigh;
}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * COMMON FUNCTIONS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
double Shooter::getWheelVel(){
    return flyWheelLeadEncoder.GetVelocity();
}

/**
 * @return converts shooter hood encoder values from angles (degrees) to the values of the absolute encoder
 **/
double Shooter::convertFromAngleToAbs(double angle)
{
    double slope = (hoodabsOut - hoodabsIn)/(hoodAngleOut - hoodAngleIn);
    double b = hoodabsIn - (slope*hoodAngleIn);
    return ((angle*slope) + b);
}

/**
 * @return converts from the absolute encoder values to angles (degrees)
 **/
double Shooter::convertFromAbsToAngle(double abs)
{
    double slope = (hoodAngleOut - hoodAngleIn)/(hoodabsOut - hoodabsIn);
    double b = hoodAngleIn - (slope*hoodabsIn);
    return ((abs*slope) + b);
}

/**
 * @return converts from the absolute encoder values to ones the rev motor can read
 * constantly updates the rev position in periodic 
 **/
double Shooter::absoluteToREV(double value){
    double slope = (hoodrevOut - hoodrevIn)/(hoodabsOut - hoodabsIn);
    double b = hoodrevIn - (slope*hoodabsIn);
    return ((value*slope) + b);
}


/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * FIXED SHOTS FOR BOTH HIGHER AND LOWER currently only higher
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::outerLaunch(const RobotData &robotData)
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut + 0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(2400, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 2330;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut + 0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(2140, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 2110;
    }
}

void Shooter::innerLaunch(const RobotData &robotData)
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut +0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(2150, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 2100;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut +0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(2000, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1950;
    }
}

void Shooter::wall(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        //38
        shooterHood_pidController.SetReference(hoodrevOut + 9, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1970, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1950;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        //38
        shooterHood_pidController.SetReference(hoodrevOut + 3, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1750, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1700;
    }
}

void Shooter::fender(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(hoodrevIn-0.25, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1690, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1670;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(hoodrevIn-0.25, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1240, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1210;
    }
}

//currently not integrated into controller data
void Shooter::endOfTarmac(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(-22, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1450;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(-22, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1450;
    }
}


/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * CONTROL DATA CONTROLS AND FUNCTIONALITY toggling
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */



