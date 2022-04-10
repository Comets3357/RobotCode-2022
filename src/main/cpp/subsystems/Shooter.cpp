#include "RobotData.h"

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * CLASS SPECIFIC INITS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::RobotInit(ShooterData &shooterData)
{
    flyWheelInit();
    shooterHoodInit();

    hoodRollerInit();
    shooterTurretInit();

    flyWheel.Set(0);
    shooterHood.Set(0);

    isTurretStatic = false;

    //FOR TESTING
    // used for reading flywheel speeds from the dashboard
    frc::SmartDashboard::PutNumber("flywheel speed", 0);

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

    shooterHood.BurnFlash();

}

void Shooter::flyWheelInit()
{
    // fly wheel motor init
    flyWheel.RestoreFactoryDefaults();
    flyWheel.SetInverted(true);
    flyWheel.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flyWheel.SetSmartCurrentLimit(70);

    readyShootLimit = 1200;

    //PIDS
    //for closer range
    flyWheelLead_pidController.SetP(0, 0);
    flyWheelLead_pidController.SetI(0, 0);
    flyWheelLead_pidController.SetD(0, 0);
    flyWheelLead_pidController.SetIZone(0, 0);
    flyWheelLead_pidController.SetFF(0.00021, 0);
    flyWheelLead_pidController.SetOutputRange(0, 1, 0);

    //for farther range
    flyWheelLead_pidController.SetP(0, 1); 
    flyWheelLead_pidController.SetI(0, 1);
    flyWheelLead_pidController.SetD(0, 1); 
    flyWheelLead_pidController.SetIZone(0, 1);
    flyWheelLead_pidController.SetFF(0.000215, 1); 
    flyWheelLead_pidController.SetOutputRange(0, 1, 1);

    flyWheelLead_pidController.SetP(0, 2); 
    flyWheelLead_pidController.SetI(0, 2);
    flyWheelLead_pidController.SetD(0, 2); 
    flyWheelLead_pidController.SetIZone(0, 2);
    flyWheelLead_pidController.SetFF(0.00022, 2); 
    flyWheelLead_pidController.SetOutputRange(0, 1, 2);

    //flyWheel.EnableVoltageCompensation()

    flyWheel.BurnFlash();  
    //flyWheel.EnableExternalUSBControl(true);
          
}

void Shooter::hoodRollerInit()
{
    //hood roller
    hoodRoller.RestoreFactoryDefaults();
    hoodRoller.SetInverted(false);
    hoodRoller.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    hoodRoller.SetSmartCurrentLimit(45);

    // //PIDS
    hoodRoller_pidController.SetP(0.00001);
    hoodRoller_pidController.SetI(0);
    hoodRoller_pidController.SetD(0);
    hoodRoller_pidController.SetIZone(0);
    hoodRoller_pidController.SetFF(0.0000955);
    hoodRoller_pidController.SetOutputRange(-1, 1);
    hoodRoller.BurnFlash(); 

}

void Shooter::shooterTurretInit()
{
    //Turret
    shooterTurret.RestoreFactoryDefaults();
    shooterTurret.SetInverted(true);
    shooterTurret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shooterTurret.SetSmartCurrentLimit(15);

    //PIDS
    shooterTurret_pidController.SetP(0.2); 
    shooterTurret_pidController.SetI(0);
    shooterTurret_pidController.SetD(0);
    shooterTurret_pidController.SetIZone(0);
    shooterTurret_pidController.SetFF(0);
    shooterTurret_pidController.SetOutputRange(-0.6,0.6);

    shooterTurret.BurnFlash(); 

}

void Shooter::DisabledInit()
{
    shooterHood.Set(0);
    flyWheel.Set(0);
    hoodRoller.Set(0);

    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    shooterTurret.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

}

void Shooter::DisabledPeriodic(const RobotData &robotData, ShooterData &shooterData){
    updateData(robotData, shooterData);

}

void Shooter::EnabledInit(ControlData &controlData, ShooterData &shooterData)
{
    controlData.shootMode = shootMode_none;

    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shooterTurret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * PERIODIC AND DRIVER CONTROL FUNCTIONS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::RobotPeriodic(const RobotData &robotData, ShooterData &shooterData)
{
    updateData(robotData, shooterData);

    //if climbing, bring the turret forward and don't run any motors
    if(robotData.controlData.mode == mode_climb_manual || robotData.controlData.mode == mode_climb_sa){
        flyWheel.Set(0);
        shooterHood.Set(0);

        if(isZeroed_Turret){ //if the encoder zeroed properly set it facing forward otherwise don't do anything
            setTurret_Pos(turretMiddleDegrees, shooterData);
        }else{
           setTurret_Pos(robotData.shooterData.currentTurretAngle, shooterData);
        }

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

    // for reject code
    if(!robotData.indexerData.autoRejectTop){
        rejectInitialized = false;
        shooterData.readyReject = false;
        shooterData.desiredRejectAngle = 180;
    }

    frc::SmartDashboard::PutBoolean("should reject", robotData.indexerData.autoRejectTop);
    frc::SmartDashboard::PutBoolean("in limelight mode", robotData.controlData.shootMode == shootMode_vision);
    frc::SmartDashboard::PutBoolean("shoot? at al?", robotData.controlData.autoRejectOpponentCargo);

    //Semi auto turret functionality
    saTurret(robotData, shooterData);

    if(robotData.timerData.secSinceInit > 1 && robotData.timerData.secSinceInit < 2){
        if(encoderPluggedInHood(shooterData)){
            shooterHoodEncoderRev.SetPosition(HoodabsoluteToREV(shooterHoodEncoderAbs.GetOutput()));
            isZeroed_Hood = true;
        }else{
            isZeroed_Hood = false;
        }

        if(encoderPluggedInTurret(shooterData)){
            shooterTurretEncoderRev.SetPosition(turretAbsoluteToREV(shooterTurretEncoderAbs.GetOutput()));
            isZeroed_Turret = true;
        }else{
            isZeroed_Turret = false;
        }
    }

    shooterHood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    shooterHood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    shooterHood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, hoodrevIn -1);
    shooterHood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, hoodrevOut +1);

    shooterTurret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    shooterTurret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    shooterTurret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, turretFullRotationRev_C );
    shooterTurret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, turretFullRotationRev_CCW );
    

    //SHOOTING LOGIC
    if(robotData.indexerData.autoRejectTop && robotData.controlData.autoRejectOpponentCargo){
        reject(robotData, shooterData);
        isTurretStatic = false;
    } else if(robotData.controlData.shootMode == shootMode_vision){ // Aiming with limelight
        isTurretStatic = false;

        //if the difference between the current velocity and the desired velocity is greater than a certain amount give it straight 100% vbus to kick start it
        //then once it's reached a certain amount below the target velocity switch to a pid to get than final desired rpm 
        if(robotData.limelightData.desiredVel - flyWheelLeadEncoder.GetVelocity() > 350){
            flyWheel.Set(1); //give it full power
        }else{
            if(robotData.intakeData.usingIntake){
                flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 2);

            }else{
                if(robotData.limelightData.distanceOffset >= 9*12){
                    flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1);
                }else{
                    flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0);
                }
            }

        }
    
        //sets the hood roller speed as normal
        hoodRoller_pidController.SetReference(robotData.limelightData.desiredHoodRollerVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

        //sets the hood to the desired location, once you're close stop it from moving to decrease jitter
        if(isZeroed_Hood){
            if(std::abs(hoodAngletoRev(robotData.limelightData.desiredHoodPos) - shooterHoodEncoderRev.GetPosition()) <= 1){
                shooterHood.Set(0);
            }else{
                shooterHood_pidController.SetReference(hoodAngletoRev(robotData.limelightData.desiredHoodPos), rev::CANSparkMaxLowLevel::ControlType::kPosition);
            }
        }else{
            shooterHood_pidController.SetReference(shooterHoodEncoderRev.GetPosition(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        }
        

        //once it's a high enough velocity its ready for indexer to run
        if (shooterData.readyShoot == false && (getWheelVel() > (robotData.limelightData.desiredVel - 30)) /**&& (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 3)**/)
        //if you're not in readyShoot yet and the wheel velocity is above 30 under the desire velocity, readyShoot will become true
        {
            shooterData.readyShoot = true;
        }
        else if (shooterData.readyShoot == true && (getWheelVel() < (robotData.limelightData.desiredVel - 100)) /**&& (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 3)**/)
        // if you're already in readyShoot, you'll only exit readyShoot if the wheel velocity drops below 100 below the desired velocity
        {
            shooterData.readyShoot = false;
        }

        // //CODE FOR TUNING SHOTS, TESTING CODE
        // double flywheelSpeed = frc::SmartDashboard::GetNumber("flywheel speed", 0);
        // flyWheelLead_pidController.SetReference(flywheelSpeed, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0);
        // hoodRoller_pidController.SetReference(flywheelSpeed*3.5, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

        // if (shooterData.readyShoot == false && (getWheelVel() > (flywheelSpeed - 30)))
        // //if you're not in readyShoot yet and the wheel velocity is above 30 under the desire velocity, readyShoot will become true
        // {
        //     shooterData.readyShoot = true;
        // }
        // else if (shooterData.readyShoot == true && (getWheelVel() < (flywheelSpeed - 100)))
        // // if you're already in readyShoot, you'll only exit readyShoot if the wheel velocity drops below 100 below the desired velocity
        // {
        //     shooterData.readyShoot = false;
        // }
    
//FIXED SHOTS 
//want the turret to stay forward during these so yes to turret static
    }else if(robotData.controlData.shootMode == shootMode_cornerLaunchPad){ //FROM THE CLOSER LAUNCH PAD
        innerLaunch(robotData);
        checkReadyShoot(shooterData);

        isTurretStatic = true;

    }
    else if (robotData.controlData.shootMode == shootMode_wallLaunchPad) //FROM THE FARTHER LAUNCH PAD
    {
        outerLaunch(robotData);
        checkReadyShoot(shooterData);

        isTurretStatic = true;
    }
    else if(robotData.controlData.shootMode == shootMode_fender) //FROM THE FENDER FIXED SHOT
    {
        fender(robotData);
        checkReadyShoot(shooterData);

        isTurretStatic = false;
    } 
    else if (robotData.controlData.shootMode == shootMode_sideWall) //FROM THE SIDE WALL FIXED SHOT
    {
        wall(robotData);
        checkReadyShoot(shooterData);

        isTurretStatic = true;
    } 
    else //IF NO SHOOTING DON'T DO ANYTHING
    {
        shooterData.readyShoot = false;

        flyWheel.Set(0);
        hoodRoller.Set(0);

        //if the hood is too far out bring it in then stop the hood from running 

        if(isZeroed_Hood){
            if(shooterHoodEncoderRev.GetPosition() < -3){
                shooterHood_pidController.SetReference(-2, rev::CANSparkMaxLowLevel::ControlType::kPosition);
            }else{
                shooterHood.Set(0);
            }
        }else{
            shooterHood_pidController.SetReference(shooterHoodEncoderRev.GetPosition(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        }
        

        //if nothing is happening then update the isTurretStatic value based on the button control
        //can this be outside the statement?
        isTurretStatic = robotData.controlData.staticTurret;

    }
}

void Shooter::manual(const RobotData &robotData, ShooterData &shooterData)
{
    
    if(robotData.controlData.mShooterWheelForward){ //manual wheel forward
        flyWheelLead_pidController.SetReference(2000, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        hoodRoller_pidController.SetReference(5500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

    }else if(robotData.controlData.mShooterWheelBackward){ //wheel backwards
        flyWheel.Set(-0.6);
        hoodRoller.Set(-0.8);

    }else{ //stops flywheel
        flyWheel.Set(0); 
        hoodRoller.Set(0);

    }

    
    shooterHood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    shooterHood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

    shooterTurret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    shooterTurret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

    //manual turret
    if(robotData.controlData.mTurret >= 0.1 || robotData.controlData.mTurret <= -0.1){ //accounts for deadzone
        shooterTurret.Set(robotData.controlData.mTurret*.5);
    }else{
        shooterTurret.Set(0);
    }
   
    //hood to joystick controls
     if(robotData.controlData.mHood >= 0.1 || robotData.controlData.mHood <= -0.01){ //accounts for deadzone
        shooterHood.Set(-robotData.controlData.mHood*.2);
    }else{
        shooterHood.Set(0);
    }

    //zeros hood pos
    if(robotData.controlData.mZeroHood)
    {
        shooterHoodEncoderRev.SetPosition(0);
        isZeroed_Hood = true;
    }
    if(robotData.controlData.mZeroTurret)
    {
        shooterTurretEncoderRev.SetPosition(turretMiddleRev);
        isZeroed_Turret = true;
    }

}


/**
 * sa turret control
 * contains code for setting field oriented position based on joystick control 
 * constant tracking of target if target is visible
 */
void Shooter::saTurret(const RobotData &robotData, ShooterData &shooterData){

    if(robotData.controlData.shootMode == shootMode_fender || !isZeroed_Turret)
    {
        setTurret_Pos(shooterData.currentTurretAngle, shooterData);
    }
    else if (isTurretStatic)
    {
        //if static turret bring to front and dont move
        //this is set in the semiauto function
        setTurret_Pos(turretMiddleDegrees, shooterData);
    }
    else if(robotData.controlData.usingTurretDirection)
    { //controls turret using field oriented control and joystick
        turretControlTurn(robotData.controlData.saTurretDirectionController, robotData, shooterData);
    } else if (robotData.indexerData.autoRejectTop && robotData.controlData.autoRejectOpponentCargo){
        turretControlTurn(shooterData.desiredRejectAngle, robotData, shooterData);
    }
    else
    {
        // use arbitrary feed forward for PID
        if (shooterData.currentTurretAngle > turretMiddleDegrees + 45) {
            arbFF = 0.02;
        } else if (shooterData.currentTurretAngle < turretMiddleDegrees - 45) {
            arbFF = -0.02;
        } else {
            arbFF = 0.0;
        }

        frc::SmartDashboard::PutNumber("arbFF", arbFF);

        if(robotData.limelightData.validTarget) // all the if statements not needed if the jitter is taken care of in limelight
        { //if you can see a target
            setTurret_Pos(robotData.limelightData.desiredTurretAngle + averageTurretGyroOffset(robotData, shooterData), shooterData);
            
        }
    }
}

/**
 * turns the turret to a position dicated by the joystick control using field oriented location
 * @param controlTurretDirection direction given by joystick control (gives 0-360 degrees)
 */

void Shooter::turretControlTurn(float controlTurretDirection, const RobotData &robotData, ShooterData &shooterData){
    float robotDirection = robotData.drivebaseData.odometryYaw; //in degrees 
    float turretTurnPos = (controlTurretDirection - robotDirection) + turretMiddleDegrees;
    turretTurnPos = (int)turretTurnPos % 360;

    frc::SmartDashboard::PutNumber("turret position 2", std::abs(shooterData.currentTurretAngle - (turretTurnPos + 360)));
    frc::SmartDashboard::PutNumber("turret position 1", std::abs(shooterData.currentTurretAngle - turretTurnPos));

    if (turretTurnPos > 0 && turretTurnPos < 90)
    {
        if (std::abs(shooterData.currentTurretAngle - (turretTurnPos + 360)) < std::abs(shooterData.currentTurretAngle - turretTurnPos))
        {
            turretTurnPos += 360;
        }
    }

    frc::SmartDashboard::PutNumber("turret final position", turretTurnPos);
    frc::SmartDashboard::PutNumber("robot position", robotData.drivebaseData.odometryYaw);

    setTurret_Pos(turretTurnPos, shooterData);

    //turretTurnPos = (controlTurretDirection - robotDirection) + turretMiddleDegrees; //calculates turret pos based on what we know to be the center of the bot
    
    //is this code necessary??? I don't think it should ever be over or under?????? but its good ig
    // if(turretTurnPos < 0 || turretTurnPos > turretFullRotationDegrees){
    //     if(turretTurnPos < 0){
    //         turretTurnPos += 360;
    //     }else if(turretTurnPos > turretFullRotationDegrees){
    //         turretTurnPos -=360;
    //     }
    // }
    
    //if its possible to have 2 positions from you're desired location
    // if(turretTurnPos > 360){
    //     float turretTurnPos2;

    //     turretTurnPos2 = turretTurnPos - 360; //second position

    //     //checks to see which of the two values is closer to the current turret value and go to that position
    //     if(std::abs(robotData.shooterData.currentTurretAngle-turretTurnPos) < std::abs(robotData.shooterData.currentTurretAngle-turretTurnPos2)){
    //         setTurret_Pos(turretTurnPos, shooterData);
    //     }else{
    //         setTurret_Pos(turretTurnPos2, shooterData);
    //     }
    // }else{
    //     setTurret_Pos(turretTurnPos, shooterData);
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
    shooterData.currentTurretAngle = turretRevtoAngle(shooterTurretEncoderRev.GetPosition());

    //turret 
    // frc::SmartDashboard::PutNumber("shooter Turret ABS", shooterTurretEncoderAbs.GetOutput());
    // frc::SmartDashboard::PutNumber("shooter Turret REV", shooterTurretEncoderRev.GetPosition());
    frc::SmartDashboard::PutNumber("turret angle", shooterData.currentTurretAngle);
    // frc::SmartDashboard::PutNumber("Average gyro offset", averageTurretGyroOffset(robotData, shooterData));

    //hood
    // frc::SmartDashboard::PutNumber("shooter hood abs", shooterHoodEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("shooter hood rev", shooterHoodEncoderRev.GetPosition());
    frc::SmartDashboard::PutNumber("HOOD ANGLE", hoodRevtoAngle(shooterHoodEncoderRev.GetPosition()));
    frc::SmartDashboard::PutNumber("desired hood pos", robotData.limelightData.desiredHoodPos);

    //flywheel
    frc::SmartDashboard::PutNumber("flywheel vel", flyWheelLeadEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("desired flywheel vel", robotData.limelightData.desiredVel);

    // frc::SmartDashboard::PutBoolean("isZeroed Hood", isZeroed_Hood);
    // frc::SmartDashboard::PutBoolean("isZeroed Turret", isZeroed_Turret);


    //hood roller
    //frc::SmartDashboard::PutNumber("hood roller vel", hoodRollerEncoderRev.GetVelocity());
    //frc::SmartDashboard::PutNumber("desired hood roller", robotData.limelightData.desiredHoodRollerVel);
    //frc::SmartDashboard::PutNumber("DISTANE OFFEST", robotData.controlData.saDistanceOffset);

}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * COMMON FUNCTIONS
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 **/


/**
 * @returns flywheel velocity in rpm
 **/
double Shooter::getWheelVel(){
    return flyWheelLeadEncoder.GetVelocity();
}

/**
 * @return converts shooter hood encoder values from angles (degrees) to the values of the absolute encoder
 * HOOD
 **/
double Shooter::HoodconvertFromAngleToAbs(double angle)
{
    double slope = (hoodabsOut - hoodabsIn)/(hoodAngleOut - hoodAngleIn);
    double b = hoodabsIn - (slope*hoodAngleIn);
    return ((angle*slope) + b);
}

/**
 * @return converts from the absolute encoder values to angles (degrees)
 * HOOD
 **/
double Shooter::HoodconvertFromAbsToAngle(double abs)
{
    double slope = (hoodAngleOut - hoodAngleIn)/(hoodabsOut - hoodabsIn);
    double b = hoodAngleIn - (slope*hoodabsIn);
    return ((abs*slope) + b);
}

double Shooter::hoodRevtoAngle(double value){
    double slope = (hoodAngleOut - hoodAngleIn)/(hoodrevOut - hoodrevIn);
    double b = hoodAngleIn - (slope*hoodrevIn);
    return ((value*slope) + b);
}

double Shooter::hoodAngletoRev(double value){
    double slope = (hoodrevOut - hoodrevIn)/(hoodAngleOut - hoodAngleIn);
    double b = hoodrevIn - (slope*hoodAngleIn);
    return ((value*slope) + b);
}

/**
 * @return converts from the absolute encoder values to ones the rev motor can read
 * constantly updates the rev position in disabled 
 * HOOD
 **/
double Shooter::HoodabsoluteToREV(double value){
    double slope = (hoodrevOut - hoodrevIn)/(hoodabsOut - hoodabsIn);
    double b = hoodrevIn - (slope*hoodabsIn);
    return ((value*slope) + b);
}

/**
 * @return converts shooter turret encoder values from angles (degrees) to the values of the absolute encoder
 * TURRET
 **/
double Shooter::turretConvertFromAngleToAbs(double angle)
{
    double slope = (turretFullRotationAbs_CCW - turretFullRotationAbs_C )/(turretFullRotationDegrees - turretZeroDegrees);
    double b = turretFullRotationAbs_C - (slope*turretZeroDegrees);
    return ((angle*slope) + b);
}

/**
 * @return converts from the absolute encoder values to angles (degrees)
 * TURRET
 **/
double Shooter::turretConvertFromAbsToAngle(double abs)
{
    double slope = (turretFullRotationDegrees - turretZeroDegrees)/(turretFullRotationAbs_CCW - turretFullRotationAbs_C);
    double b = turretZeroDegrees - (slope*turretFullRotationAbs_C);
    return ((abs*slope) + b);
}

double Shooter::turretRevtoAngle(double rev){
    double slope = (turretFullRotationDegrees - turretZeroDegrees)/(turretFullRotationRev_CCW - turretFullRotationRev_C);
    double b = turretZeroDegrees - (slope*turretFullRotationRev_C);
    return ((rev*slope) + b);
}

/**
 * @return converts from the absolute encoder values to ones the rev motor can read
 * constantly updates the rev position in disabled 
 * TURRET
 **/
double Shooter::turretAbsoluteToREV(double value){
    double slope = (turretFullRotationRev_CCW - turretFullRotationRev_C)/(turretFullRotationAbs_CCW - turretFullRotationAbs_C);
    double b = turretFullRotationRev_C - (slope*turretFullRotationAbs_C);
    return ((value*slope) + b);
}

/**
 * Creates linear interpolation between the offset of the turret (added position) and the rate of rotation of the drivebase through gyro
 * @param value is the current rate of rotation from gyro
 * gets added on in the sa turret function
 **/

double Shooter::turretGyroOffset(double value){
    double slope = (turretGyroOffset2 - turretGyroOffset1)/(rotationalRate2 - rotationalRate1);
    double b = turretGyroOffset1 - (slope*rotationalRate1);
    return ((value*slope) + b);
}

// double Shooter::getFieldRelativeToRobotRelativeTurret(const RobotData &robotData, ShooterData &shooterData){
    
// }

/**
 * @return the angle of the turret relative to the field, 0-360. 
 * 0 is opponent wall, 180 is our wall, CCW pos
 **/
double Shooter::getFieldRelativeTurretAngle(const RobotData &robotData, ShooterData &shooterData){
    // 90 gets turret to robot on the same zero as robot to field
    return ((int)(shooterData.currentTurretAngle + 90 + robotData.drivebaseData.odometryYaw) % 360);
}

/**
 * @return sets the turret to turn to face the target when shooting USING POSITIONS
 * @param pos is the desired angle position we want to turn to IN DEGREES
 * TURRET
 **/
void Shooter::setTurret_Pos(double pos, ShooterData &shooterData){
    shooterTurret_pidController.SetReference(turretAbsoluteToREV(turretConvertFromAngleToAbs(pos)), rev::CANSparkMax::ControlType::kPosition, 0, arbFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut);
    frc::SmartDashboard::PutNumber("arbFF", arbFF);
}

/**
 * @returns the avg offset from rate of rotation of the last 5 cycles to make the data smoother while shooting
 * used in parallel with the turret gyro offset function
 * NOT FULLY TESTED
 */
double Shooter::averageTurretGyroOffset(const RobotData &robotData, ShooterData &shooterData){
    double offsetPos = turretGyroOffset(robotData.gyroData.rotationalRate); 
    double total = 0;

    //if size is less then 6 keep adding updated offset positions until the deque is full
    if(robotData.shooterData.offsetPos.size() < 6){
        shooterData.offsetPos.push_back(offsetPos);
    }else{ //once it's full run through the deque and add it to the total
        for(size_t i = 0; i < robotData.shooterData.offsetPos.size(); i++){
            total += robotData.shooterData.offsetPos.at(i);
        }

        //make sure to remove the first value and add an updated speed to the end
        shooterData.offsetPos.pop_front();
        shooterData.offsetPos.push_back(offsetPos);
    }

    //return the average of those speeds
    shooterData.avgTurretOffsetPos = total/6;
    return shooterData.avgTurretOffsetPos;
}



/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * FIXED SHOTS FOR BOTH HIGHER AND LOWER currently only higher
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::outerLaunch(const RobotData &robotData)
{
    if(isZeroed_Hood){
        shooterHood_pidController.SetReference(outerLaunchHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }else{
        shooterHood_pidController.SetReference(shooterHoodEncoderRev.GetPosition(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }

    if(outerLaunchVel - flyWheelLeadEncoder.GetVelocity() > 350){
        flyWheel.Set(1); //give it full power
    }else{
        if(robotData.intakeData.usingIntake){
            flyWheelLead_pidController.SetReference(outerLaunchVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 2);

        }else{
            flyWheelLead_pidController.SetReference(outerLaunchVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 2);
            
        }

    }
  
    hoodRoller_pidController.SetReference(outerLaunchVel*3.5, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

    readyShootLimit = outerLaunchVel - 30;
    
}

void Shooter::innerLaunch(const RobotData &robotData)
{
   
    if(isZeroed_Hood){
        shooterHood_pidController.SetReference(innerLaunchHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }else{
        shooterHood_pidController.SetReference(shooterHoodEncoderRev.GetPosition(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }

    if(innerLaunchVel - flyWheelLeadEncoder.GetVelocity() > 350){
        flyWheel.Set(1); //give it full power
    }else{
        if(robotData.intakeData.usingIntake){
            flyWheelLead_pidController.SetReference(innerLaunchVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 2);

        }else{
            flyWheelLead_pidController.SetReference(innerLaunchVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1);
            
        }

    }

    hoodRoller_pidController.SetReference(innerLaunchVel*3.5, rev::CANSparkMaxLowLevel::ControlType::kVelocity);


    readyShootLimit = innerLaunchVel - 30;

    
}

void Shooter::wall(const RobotData &robotData)
{
    if(isZeroed_Hood){
        shooterHood_pidController.SetReference(wallHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);

    }else{
        shooterHood_pidController.SetReference(shooterHoodEncoderRev.GetPosition(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }       

    if(wallVel - flyWheelLeadEncoder.GetVelocity() > 350){
        flyWheel.Set(1); //give it full power
    }else{
        if(robotData.intakeData.usingIntake){
            flyWheelLead_pidController.SetReference(wallVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 2);

        }else{
            flyWheelLead_pidController.SetReference(wallVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1);
            
        }

    }

    hoodRoller_pidController.SetReference(wallVel*3.5, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

    readyShootLimit = wallVel - 30;
    
}

void Shooter::fender(const RobotData &robotData)
{
    if(isZeroed_Hood){
        shooterHood_pidController.SetReference(fenderHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);

    }else{
        shooterHood_pidController.SetReference(shooterHoodEncoderRev.GetPosition(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }

    if(fenderVel - flyWheelLeadEncoder.GetVelocity() > 350){
        flyWheel.Set(1); //give it full power
    }else{
        if(robotData.intakeData.usingIntake){
            flyWheelLead_pidController.SetReference(fenderVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1);

        }else{
            flyWheelLead_pidController.SetReference(fenderVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0);
            
        }

    }

    hoodRoller_pidController.SetReference(fenderVel*3, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

    readyShootLimit = fenderVel - 30;
    
}


void Shooter::reject(const RobotData &robotData, ShooterData &shooterData){
    shooterHood_pidController.SetReference(hoodrevOut + 1, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    setShooterWheel(800, 0);
    readyShootLimit = 800;

    if(!rejectInitialized){
        if(std::abs(getFieldRelativeTurretAngle(robotData, shooterData) - shooterData.desiredRejectAngle)  < 5){
            if(robotData.limelightData.validTarget){
                shooterData.desiredRejectAngle = 0; // on the other side 
                rejectInitialized = true;
            } else {
                shooterData.desiredRejectAngle = 180;
                rejectInitialized = true;
            }
        } /* else {
            turretControlTurn(shooterData.desiredRejectAngle, robotData, shooterData);
        } */
    } else { // you've figured out which side you're on

        // turretControlTurn(shooterData.desiredRejectAngle, robotData, shooterData); // FIX
        if(getWheelVel() > readyShootLimit - 30 && getWheelVel() < readyShootLimit + 30){
            shooterData.readyShoot = true;
        }

        //you don't see a valid target
        if(shooterData.desiredRejectAngle == 180){
            if(!robotData.limelightData.validTarget && (std::abs(getFieldRelativeTurretAngle(robotData, shooterData) - shooterData.desiredRejectAngle)  < 5)){
                shooterData.readyReject = true;
            } else {
                shooterData.readyReject = false;
            }
        } else if(shooterData.desiredRejectAngle == 0){
            if(!robotData.limelightData.validTarget && (getFieldRelativeTurretAngle(robotData, shooterData) < shooterData.desiredRejectAngle + 5 || getFieldRelativeTurretAngle(robotData, shooterData) > shooterData.desiredRejectAngle - 5 + 360)){
                shooterData.readyReject = true;
            } else {
                shooterData.readyReject = false;
            }
        }
    }
}


// checks to see if the flywheel is up to speed
void Shooter::checkReadyShoot(ShooterData &shooterData){
    //once it's a high enough velocity its ready for indexer to run
    if (shooterData.readyShoot == false && (getWheelVel() > readyShootLimit)) /**&& (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 3)**/
    //if you're not in readyShoot yet and the wheel velocity is above 30 under the desire velocity, readyShoot will become true
    {
        shooterData.readyShoot = true;
    }
    else if(shooterData.readyShoot == true && (getWheelVel() < (readyShootLimit - 70))) /**&& (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 3)**/
    // if you're already in readyShoot, you'll only exit readyShoot if the wheel velocity drops below 100 below the desired velocity
    {
        shooterData.readyShoot = false;
    }
}

/**
 * @param speed is the speed you want to set the flywheel to
 * @param pidSlot is 0 (for closer shots) or 1 (farther back shots has more power)
 */
void Shooter::setShooterWheel(double speed, double pidSlot){
    flyWheelLead_pidController.SetReference(speed, rev::CANSparkMaxLowLevel::ControlType::kVelocity, pidSlot);

}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * BENCH TEST CODE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::TestPeriodic(const RobotData &robotData, ShooterData &shooterData){
    //frc::SmartDashboard::PutBoolean("Shooter abs encoder working", encoderPluggedInHood(shooterData));
    frc::SmartDashboard::PutBoolean("Shooter abs encoder reading in correct range", encoderInRange(shooterData));
    frc::SmartDashboard::PutBoolean("Shooter hit bottom dead stop?", shooterData.bottomDeadStop);
    frc::SmartDashboard::PutBoolean("Shooter hit top dead stop?", shooterData.topDeadStop);
    frc::SmartDashboard::PutNumber("Shooter Abs Encoder Value", shooterHoodEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("Shooter min extend expected encoder value", hoodabsIn);
    frc::SmartDashboard::PutNumber("Shooter max extend expected encoder value", hoodabsOut);
    frc::SmartDashboard::PutNumber("Shooter hood power", shooterData.benchTestShooterHoodSpeed);
    frc::SmartDashboard::PutNumber("Fly Wheel Speed", shooterData.benchTestFlyWheelSpeed);

    checkDeadStop(shooterData);

    //runs the bench test sequence
    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && robotData.controlData.manualBenchTest){ //checks if we're testing shooter
        if (false){ //checks if the encoder is working
            if (robotData.benchTestData.stage == 0){
                //run hood forwards
                shooterData.benchTestShooterHoodSpeed = -.07; //sets the speed of the hood
                shooterData.benchTestFlyWheelSpeed = 0; //sets the speed of the fly wheel
            } else if (robotData.benchTestData.stage == 1){
                //run hoods backwards
                shooterData.benchTestShooterHoodSpeed = .07;
                shooterData.benchTestFlyWheelSpeed = 0;
            } else if (robotData.benchTestData.stage == 2){
                //run fly wheel
                shooterData.benchTestShooterHoodSpeed = 0;
                shooterData.benchTestFlyWheelSpeed = .25;
            } else if (robotData.benchTestData.PIDMode){ //tests in pid mode
                if (robotData.benchTestData.stage == 3){
                    shooterData.benchTestFlyWheelSpeed = 0;
                    // bring hood out
                    shooterHood_pidController.SetReference(hoodrevOut, rev::CANSparkMaxLowLevel::ControlType::kPosition);
                } else if (robotData.benchTestData.stage == 4){
                    shooterData.benchTestFlyWheelSpeed = 0;
                    // bring hood in
                    shooterHood_pidController.SetReference(hoodrevIn, rev::CANSparkMaxLowLevel::ControlType::kPosition);
                } else {
                    shooterData.benchTestShooterHoodSpeed = 0;
                    shooterData.benchTestFlyWheelSpeed = 0;
                }
            }
        }

        //sets the speed of the motors according to the variables set in the above if statement ^ (unless the hood hit a dead stop)
        if (!shooterData.topDeadStop && !shooterData.bottomDeadStop){
            shooterHood.Set(shooterData.benchTestShooterHoodSpeed);
        } else {
            shooterHood.Set(0);
        }

        flyWheel.Set(shooterData.benchTestFlyWheelSpeed);
        //flyWheel.Set(shooterData.benchTestFlyWheelSpeed);
    } else {
        shooterData.benchTestShooterHoodSpeed = 0; //if not testing shooter, then the speed of the motors is set to 0
        shooterData.benchTestFlyWheelSpeed = 0;
    }
}

//checks if the encoder is plugged in and giving an output
bool Shooter::encoderPluggedInHood(ShooterData &shooterData){

    if (shooterHoodEncoderAbs.GetOutput() > 0.01 && shooterHoodEncoderAbs.GetOutput() < 1)  { //checks if the output of the abs encoder is actually reading a signal
        //uses mode to find most constant abs value to return 
        //doesn't work because the encoder abs value changes too much to find a mode
        //updates encoder values
        // if (tickCount > 48){
        //     double number = hoodAbsValues[0];
        //     double mode = number;
        //     int count = 1;
        //     int countMode = 1;

        //     std::sort(std::begin(hoodAbsValues),std::end(hoodAbsValues));

        //     for(int i=0; i<49; i++){
        //         if (hoodAbsValues[i] == number) 
        //         { // count occurrences of the current number
        //             count++;
        //         }else{ // now this is a different number
        //             if (count > countMode) 
        //             {
        //                 countMode = count; // mode is the biggest ocurrences
        //                 mode = number;
        //             }
        //             count = 1; // reset count for the new number
        //             number = hoodAbsValues[i];
        //         }
        //     }

        //     // float maxFreq = 0;
        //     // float mostFrequent = -1;

        //     // for(int i = 0; i < 49; i++){
        //     //     int countFreq = 1;
        //     //     for(int j = 0; j < 49; j++){
        //     //         if(hoodAbsValues[j] == hoodAbsValues[i]){
        //     //             countFreq ++;
        //     //         }
        //     //     }

        //     //     if(maxFreq < countFreq){
        //     //         maxFreq = countFreq;
        //     //         mostFrequent = hoodAbsValues[i];
        //     //     }
        //     //     else if(maxFreq == countFreq){
        //     //         mostFrequent = std::min(mostFrequent, hoodAbsValues[i]);
        //     //     }
        //     // }

        //     shooterData.mode = mode;

        //     //shooterHoodEncoderRev.SetPosition(HoodabsoluteToREV(mode));
        //     tickCount = (tickCount + 1) % 50;
        // } else {
        //     float hoodAbsValue = 1000*shooterHoodEncoderAbs.GetOutput();
        //     hoodAbsValue = std::floor(hoodAbsValue);
        //     hoodAbsValues[tickCount] = hoodAbsValue/1000.0;
        //     tickCount = (tickCount + 1) % 50;
        // }

        return true; //returns true to indicate that the encoder is functioning
    } else {
        return false;
    }

}

//checks if the encoder is plugged in and giving an output
bool Shooter::encoderPluggedInTurret(const ShooterData &shooterData){
    if (shooterTurretEncoderAbs.GetOutput() > 0.01) { //checks if the output of the abs encoder is actually reading a signal
        return true;
    } else {
        return false;
    }
}

//checks if the encoder is reading presumably correct values (i.e. values in the expected range)
bool Shooter::encoderInRange(const ShooterData &shooterData){
    if (shooterData.benchTestShooterHoodSpeed > 0 && shooterHoodEncoderAbs.GetOutput() > hoodabsIn + .02){ //out of range
        return false;
    } else if (shooterData.benchTestShooterHoodSpeed < 0 && shooterHoodEncoderAbs.GetOutput() < hoodabsOut - .02){ //out of range
        return false;
    } else {
        return true; //everything else is within range, so return true
    }
}

//checks if the motor has hit a dead stop
void Shooter::checkDeadStop(ShooterData &shooterData){
    if (shooterData.benchTestShooterHoodSpeed < 0 && shooterHoodEncoderAbs.GetOutput() < hoodabsOut + .005){
        shooterData.topDeadStop = true;
        shooterData.bottomDeadStop = false;
    } else if (shooterData.benchTestShooterHoodSpeed > 0 && shooterHoodEncoderAbs.GetOutput() > hoodabsIn - .005){
        shooterData.topDeadStop = false;
        shooterData.bottomDeadStop = true;
    } else {
        shooterData.topDeadStop = false;
        shooterData.bottomDeadStop = false;
    }
}




