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
    flyWheelLead_pidController.SetP(0.002); 
    flyWheelLead_pidController.SetI(0);
    flyWheelLead_pidController.SetD(0.005);
    flyWheelLead_pidController.SetIZone(0);
    flyWheelLead_pidController.SetFF(0.0002);
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
    shooterData.shootMode = shootMode_none;
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
    updateShootMode(robotData, shooterData);

    if(robotData.controlData.mode == mode_climb_manual || robotData.controlData.mode == mode_climb_sa){
        shooterHood_pidController.SetReference(-1,rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead.Set(0);

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


    
    
    //idk about this control data stuff to figure out
    if(robotData.controlData.upperHubShot){
        setHighHub();
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
    if(shooterData.shootMode == shootMode_vision){ // Aiming with limelight
        //set the hood and flywheel using pids to the desired values based off the limelight code
        flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        shooterHood_pidController.SetReference(absoluteToREV(convertFromAngleToAbs(robotData.limelightData.desiredHoodPos)), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        
        //retrieves flywheel speeds from the dashboard
        double hi = frc::SmartDashboard::GetNumber("wheel speed", 0);
        
        //FOR TESTING PURPOSES
        //flyWheelLead_pidController.SetReference(hi, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        //if (getWheelVel() > (hi - 15))

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
    }else if(shooterData.shootMode == shootMode_cornerLaunchPad){ //FROM THE CLOSER LAUNCH PAD
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
    else if (shooterData.shootMode == shootMode_wallLaunchPad) //FROM THE FARTHER LAUNCH PAD
    {
        outerLaunch();
        if ((getWheelVel() > readyShootLimit) /**&& (std::abs(getHoodPos() + 42) <= 1)**/)
        {
            shooterData.readyShoot = true;
        }else
        {
            shooterData.readyShoot = false;
        }
    }
    else if(shooterData.shootMode == shootMode_fender) //FROM THE FENDER FIXED SHOT
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
    else if (shooterData.shootMode == shootMode_sideWall) //FROM THE SIDE WALL FIXED SHOT
    {
        wall();
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
    // frc::SmartDashboard::PutNumber("shooter Hood REV", shooterHoodEncoderRev.GetPosition());
    //frc::SmartDashboard::PutNumber("shooter changed", absoluteToREV(shooterHoodEncoderAbs.GetOutput()));
    //frc::SmartDashboard::PutNumber("desired hood to rev", absoluteToREV(convertFromAngleToAbs(robotData.limelightData.desiredHoodPos)));
    //frc::SmartDashboard::PutNumber("shootMode", shooterData.shootMode);

    frc::SmartDashboard::PutBoolean("shooter ready shoot", shooterData.readyShoot);
    frc::SmartDashboard::PutNumber("HOOD ANGLE", convertFromAbsToAngle(shooterHoodEncoderAbs.GetOutput()));
    frc::SmartDashboard::PutNumber("flywheel vel", flyWheelLeadEncoder.GetVelocity());
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
void Shooter::outerLaunch()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut + 0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(2050, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 2000;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut + 0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1900, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1850;
    }
}

void Shooter::innerLaunch()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut + 2, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1875, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1825;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevOut + 2, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1875, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1825;
    }
}

void Shooter::wall()
{
    if (isHigh)
    {
        //38
        shooterHood_pidController.SetReference(hoodrevOut + 6, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1750, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1700;
    }
    else if (!isHigh)
    {
        //38
        shooterHood_pidController.SetReference(hoodrevOut + 6, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1750, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1700;
    }
}

void Shooter::fender()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevIn-0.25, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1480;
    }
    else if (!isHigh)
    {
        shooterHood_pidController.SetReference(hoodrevIn-0.25, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1100, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1000;
    }
}

//currently not integrated into controller data
void Shooter::endOfTarmac()
{
    if (isHigh)
    {
        shooterHood_pidController.SetReference(-22, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        flyWheelLead_pidController.SetReference(1500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        readyShootLimit = 1450;
    }
    else if (!isHigh)
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

void Shooter::updateShootMode(const RobotData &robotData, ShooterData &shooterData) {
    // pressing a shoot button will set the robot to be in the associated shooting mode. if you press the button again, it will toggle that shoot mode off.

    if (robotData.controlData.saShooting) {
        if (shooterData.shootMode == shootMode_vision) {
            shooterData.shootMode = shootMode_none;
        } else { shooterData.shootMode = shootMode_vision; }
    }

    if (robotData.controlData.fenderShot) {
        if (shooterData.shootMode == shootMode_fender) {
            shooterData.shootMode = shootMode_none;
        } else { shooterData.shootMode = shootMode_fender; }
    }

    if (robotData.controlData.sideWallShot) {
        if (shooterData.shootMode == shootMode_sideWall) {
            shooterData.shootMode = shootMode_none;
        } else { shooterData.shootMode = shootMode_sideWall; }
    }

    if (robotData.controlData.wallLaunchPadShot) {
        if (shooterData.shootMode == shootMode_wallLaunchPad) {
            shooterData.shootMode = shootMode_none;
        } else { shooterData.shootMode = shootMode_wallLaunchPad; }
    }

    if (robotData.controlData.cornerLaunchPadShot) {
        if (shooterData.shootMode == shootMode_cornerLaunchPad) {
            shooterData.shootMode = shootMode_none;
        } else { shooterData.shootMode = shootMode_cornerLaunchPad; }
    }

    // interpret button data to toggle between shooting unassigned as ours or opponent's
    if (robotData.controlData.shootUnassignedAsOpponent) {
        shooterData.shootUnassignedAsOpponent = !shooterData.shootUnassignedAsOpponent;
    }

    // shut off shooting if all balls have exited (happens once upon ball count going to zero)
    // if (robotData.indexerData.indexerContents.size() == 0 && lastTickBallCount > 0 && shooterData.shootMode != shootMode_none /* probably redundant */) {
    //     shooterData.shootMode = shootMode_none;
    // }
    // lastTickBallCount = robotData.indexerData.indexerContents.size();
}