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

    flyWheelLead.Set(0);
    shooterHood.Set(0);

    isTurretStatic = false;

    //FOR TESTING
    //used for reading flywheel speeds from the dashboard
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

    shooterHood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    shooterHood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    shooterHood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, hoodrevIn -2);
    shooterHood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, hoodrevOut +1);

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
    flyWheelLead_pidController.SetP(0.00001, 0); //0.002
    flyWheelLead_pidController.SetI(0, 0);
    flyWheelLead_pidController.SetD(0, 0); //0.005
    flyWheelLead_pidController.SetIZone(0, 0);
    flyWheelLead_pidController.SetFF(0.000219, 0); //0.0002
    flyWheelLead_pidController.SetOutputRange(-1,1, 0);

    flyWheelLead_pidController.SetP(0.00001, 1); //0.002
    flyWheelLead_pidController.SetI(0, 1);
    flyWheelLead_pidController.SetD(0, 1); //0.005
    flyWheelLead_pidController.SetIZone(0, 1);
    flyWheelLead_pidController.SetFF(0.000219, 1); //0.0002
    flyWheelLead_pidController.SetOutputRange(-1,1, 1);

    flyWheelLead.BurnFlash();            
}

void Shooter::hoodRollerInit()
{
    // fly wheel LEAD motor init
    hoodRoller.RestoreFactoryDefaults();
    hoodRoller.SetInverted(false);
    hoodRoller.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    hoodRoller.SetSmartCurrentLimit(45);

    // //PIDS
    hoodRoller_pidController.SetP(0.00001,0); //0.002
    hoodRoller_pidController.SetI(0,0);
    hoodRoller_pidController.SetD(0,0); //0.005
    hoodRoller_pidController.SetIZone(0,0);
    hoodRoller_pidController.SetFF(0.0000955,0); //0.0002
    hoodRoller_pidController.SetOutputRange(-1,1,0);
    hoodRoller.BurnFlash();   

    // //PIDS
    hoodRoller_pidController.SetP(0.00001,1); //0.002
    hoodRoller_pidController.SetI(0,1);
    hoodRoller_pidController.SetD(0,1); //0.005
    hoodRoller_pidController.SetIZone(0,1);
    hoodRoller_pidController.SetFF(0.0000955,1); //0.0002
    hoodRoller_pidController.SetOutputRange(-1,1,1);
}

void Shooter::shooterTurretInit(){
    // fly wheel LEAD motor init
    shooterTurret.RestoreFactoryDefaults();
    shooterTurret.SetInverted(true);
    shooterTurret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shooterTurret.SetSmartCurrentLimit(15);


    //PIDS
    shooterTurret_pidController.SetP(0.12); 
    shooterTurret_pidController.SetI(0);
    shooterTurret_pidController.SetD(0);
    shooterTurret_pidController.SetIZone(0);
    shooterTurret_pidController.SetFF(0);
    shooterTurret_pidController.SetOutputRange(-1,1);
    shooterTurret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    shooterTurret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    shooterTurret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, turretFullRotationRev_C );
    shooterTurret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, turretFullRotationRev_CCW );

    shooterTurret.BurnFlash(); 

}

void Shooter::DisabledInit()
{
    shooterHood.Set(0);
    flyWheelLead.Set(0);
    hoodRoller.Set(0);

    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    shooterTurret.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

}

void Shooter::DisabledPeriodic(const RobotData &robotData, ShooterData &shooterData){
    updateData(robotData, shooterData);
    encoderPluggedInTurret(shooterData);
    encoderPluggedInHood(shooterData);


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

    if(robotData.controlData.mode == mode_climb_manual || robotData.controlData.mode == mode_climb_sa){
        flyWheelLead.Set(0);
        shooterHood.Set(0);

        setTurret_Pos(turretMiddleDegrees, shooterData);

    }else{
        if (robotData.controlData.mode == mode_teleop_manual)
        {
            manual(robotData, shooterData);
        }
        else if (robotData.controlData.mode == mode_teleop_sa)
        {
            semiAuto(robotData, shooterData);
        }

        //manual(robotData, shooterData);

    }

}

void Shooter::semiAuto(const RobotData &robotData, ShooterData &shooterData){

    // for reject code
    if(!robotData.indexerData.autoRejectTop){
        rejectInitialized = false;
        shooterData.readyReject = false;
        desiredAngle = 180;
    }

    frc::SmartDashboard::PutBoolean("should reject", robotData.indexerData.autoRejectTop);
    frc::SmartDashboard::PutBoolean("in limelight mode", robotData.controlData.shootMode == shootMode_vision);
    frc::SmartDashboard::PutBoolean("shoot? at al?", robotData.controlData.autoRejectOpponentCargo);

    saTurret(robotData, shooterData);
    //if nothing is happening then update the isTurretStatic value based on the button control
    //can this be outside the statement?
    isTurretStatic = robotData.controlData.staticTurret;


    //SHOOTING LOGIC
    if(robotData.indexerData.autoRejectTop && !robotData.controlData.autoRejectOpponentCargo){
        reject(robotData, shooterData);
        isTurretStatic = false;
    } else if(robotData.controlData.shootMode == shootMode_vision){ // Aiming with limelight

        //set the hood and flywheel using pids to the desired values based off the limelight code
        //checks battery voltage and increases velocity if it doesn't have enough power

        double flywheelSpeed = frc::SmartDashboard::GetNumber("flywheel speed", 0);
        
        // if(frc::DriverStation::GetBatteryVoltage() > 12.6){
        //     flyWheelLead_pidController.SetReference(flywheelSpeed, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        //     hoodRoller_pidController.SetReference(flywheelSpeed*2.75, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        // }else{
        //     flyWheelLead_pidController.SetReference(flywheelSpeed + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        //     hoodRoller_pidController.SetReference(flywheelSpeed*2.75 + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity);        
        // } 

        
        if(frc::DriverStation::GetBatteryVoltage() > 12.6){
            if(robotData.limelightData.distanceOffset >= 15){
                flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity ,1);
                hoodRoller_pidController.SetReference(robotData.limelightData.desiredHoodRollerVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1);
            }else{
                flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity ,0);
                hoodRoller_pidController.SetReference(robotData.limelightData.desiredHoodRollerVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0);
            }
           
        }else{
            if(robotData.limelightData.distanceOffset < 15){
                flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1);
                hoodRoller_pidController.SetReference(robotData.limelightData.desiredHoodRollerVel + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 1); 
            }else{
                flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0);
                hoodRoller_pidController.SetReference(robotData.limelightData.desiredHoodRollerVel + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0);  
            }
                  
        }    


        if(std::abs(HoodabsoluteToREV(HoodconvertFromAngleToAbs(robotData.limelightData.desiredHoodPos) - shooterHoodEncoderRev.GetPosition())) <= 1){
            shooterHood.Set(0);
        }else{
            shooterHood_pidController.SetReference(HoodabsoluteToREV(HoodconvertFromAngleToAbs(robotData.limelightData.desiredHoodPos)), rev::CANSparkMaxLowLevel::ControlType::kPosition);

        }

        // if (shooterData.readyShoot == false && (getWheelVel() > (flywheelSpeed - 30)))
        // //if you're not in readyShoot yet and the wheel velocity is above 30 under the desire velocity, readyShoot will become true
        // {
        //     shooterData.readyShoot = true;
        // }
        // //else
        // else if (shooterData.readyShoot == true && (getWheelVel() < (flywheelSpeed - 100)))
        // // if you're already in readyShoot, you'll only exit readyShoot if the wheel velocity drops below 100 below the desired velocity
        // {
        //     shooterData.readyShoot = false;
        // }

        // //once it's a high enough velocity and turret is in place its ready for indexer to run
        if (shooterData.readyShoot == false && (getWheelVel() > (robotData.limelightData.desiredVel - 30)) /**&& (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 3)**/)
        //if you're not in readyShoot yet and the wheel velocity is above 30 under the desire velocity, readyShoot will become true
        {
            shooterData.readyShoot = true;
        }
        //else
        else if (shooterData.readyShoot == true && (getWheelVel() < (robotData.limelightData.desiredVel - 100)) /**&& (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 3)**/)
        // if you're already in readyShoot, you'll only exit readyShoot if the wheel velocity drops below 100 below the desired velocity
        {
            shooterData.readyShoot = false;
        }
    
//FIXED SHOTS 
    } 
    else if(robotData.controlData.shootMode == shootMode_cornerLaunchPad){ //FROM THE CLOSER LAUNCH PAD
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

        isTurretStatic = true;
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

        flyWheelLead.Set(0);
        hoodRoller.Set(0);

        

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
        hoodRoller_pidController.SetReference(5500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);


    }else if(robotData.controlData.mShooterWheelBackward){ //wheel backwards
        flyWheelLead_pidController.SetReference(-2000, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        hoodRoller_pidController.SetReference(-5500, rev::CANSparkMaxLowLevel::ControlType::kVelocity);

    }else{
        flyWheelLead.Set(0); //stops flywheel
        hoodRoller.Set(0);

    }

    //manual turret
    if(robotData.controlData.mTurret >= 0.015 || robotData.controlData.mTurret <= -0.015){ //accounts for deadzone
        shooterTurret.Set(robotData.controlData.mTurret*.5);
    }else{
        shooterTurret.Set(0);
    }
    //hood to joystick controls
     if(robotData.controlData.mHood >= 0.01 || robotData.controlData.mHood <= -0.01){ //accounts for deadzone
        shooterHood.Set(-robotData.controlData.mHood*.2);
    }else{
        shooterHood.Set(0);
    }

    //zeros hood pos
    if(robotData.controlData.mZeroHood)
    {
        shooterHoodEncoderRev.SetPosition(0);
    }
    if(robotData.controlData.mZeroTurret)
    {
        shooterTurretEncoderRev.SetPosition(0);
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
    //frc::SmartDashboard::PutNumber("shooter Turret ABS", shooterTurretEncoderAbs.GetOutput());
    //frc::SmartDashboard::PutNumber("shooter Turret REV", shooterTurretEncoderRev.GetPosition());

    frc::SmartDashboard::PutNumber("shooter hood abs", shooterHoodEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("shooter hood rev", shooterHoodEncoderRev.GetPosition());

    //frc::SmartDashboard::PutNumber("hood encoder connection?", encoderPluggedInHood(shooterData));



    //frc::SmartDashboard::PutBoolean("shooter ready shoot", shooterData.readyShoot);
    frc::SmartDashboard::PutNumber("HOOD ANGLE", HoodconvertFromAbsToAngle(shooterHoodEncoderAbs.GetOutput()));
    frc::SmartDashboard::PutNumber("flywheel vel", flyWheelLeadEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("hood roller vel", hoodRollerEncoderRev.GetVelocity());

    frc::SmartDashboard::PutNumber("desired flywheel vel", robotData.limelightData.desiredVel);


    shooterData.currentTurretAngle = turretConvertFromAbsToAngle(shooterTurretEncoderAbs.GetOutput());
    
    //frc::SmartDashboard::PutNumber("turret angle", shooterData.currentTurretAngle);

    frc::SmartDashboard::PutNumber("Gyro offset", turretGyroOffset(robotData.gyroData.angularMomentum));
    //frc::SmartDashboard::PutNumber("control joystick", robotData.controlData.saTurretDirectionController);


    frc::SmartDashboard::PutNumber("desired hood pos", robotData.limelightData.desiredHoodPos);
    // frc::SmartDashboard::PutNumber("upper hub shot", robotData.controlData.upperHubShot);
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

/**
 * @return converts from the absolute encoder values to ones the rev motor can read
 * constantly updates the rev position in periodic 
 * HOOD
 **/
double Shooter::HoodabsoluteToREV(double value){
    double slope = (hoodrevOut - hoodrevIn)/(hoodabsOut - hoodabsIn);
    double b = hoodrevIn - (slope*hoodabsIn);
    return ((value*slope) + b);
}

/**
 * @return converts shooter hood encoder values from angles (degrees) to the values of the absolute encoder
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

/**
 * @return converts from the absolute encoder values to ones the rev motor can read
 * constantly updates the rev position in periodic 
 * TURRET
 **/
double Shooter::turretAbsoluteToREV(double value){
    double slope = (turretFullRotationRev_CCW - turretFullRotationRev_C)/(turretFullRotationAbs_CCW - turretFullRotationAbs_C);
    double b = turretFullRotationRev_C - (slope*turretFullRotationAbs_C);
    return ((value*slope) + b);
}

/**
 * RENAME THE VARIABLES TO MAKE SENSE CAUSE THEY AREN"T DESCRIPtiVR OR RIGHT
 * @param 
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
    shooterTurret_pidController.SetReference(turretAbsoluteToREV(turretConvertFromAngleToAbs(pos)), rev::CANSparkMax::ControlType::kPosition);
}



/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * FIXED SHOTS FOR BOTH HIGHER AND LOWER currently only higher
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::outerLaunch(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(outerLaunchHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        //checks battery voltage and adjusts the pid accordingly
        setShooterWheel(outerLaunchVel);

        readyShootLimit = outerLaunchVel - 30;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(outerLaunchHood_Low, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(outerLaunchVel_Low);

        readyShootLimit = outerLaunchVel_Low - 30;

    }
}

void Shooter::innerLaunch(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(innerLaunchHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(innerLaunchVel);

        readyShootLimit = innerLaunchVel - 30;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(innerLaunchHood_Low, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(innerLaunchVel_Low);

        readyShootLimit = innerLaunchVel_Low - 30;
    }
}

void Shooter::wall(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(wallHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(wallVel);

        readyShootLimit = wallVel - 30;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(wallHood_Low, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(wallVel_Low);

        readyShootLimit = wallVel_Low - 30;
    }
}

void Shooter::fender(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(fenderHood, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(fenderVel);

        readyShootLimit = fenderVel - 30;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(fenderHood_Low, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(fenderVel_Low);
        
        readyShootLimit = fenderVel_Low - 30;
    }
}

void Shooter::reject(const RobotData &robotData, ShooterData &shooterData){
    shooterHood_pidController.SetReference(hoodrevOut + 1, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    setShooterWheel(800);
    readyShootLimit = 800;

    if(!rejectInitialized){
        if(std::abs(getFieldRelativeTurretAngle(robotData, shooterData) - desiredAngle)  < 5){
            if(robotData.limelightData.validTarget){
                desiredAngle = 0; // on the other side 
                rejectInitialized = true;
            } else {
                desiredAngle = 180;
                rejectInitialized = true;
            }
        } else {
            turretControlTurn(desiredAngle, robotData, shooterData);
        }
    } else { // you've figured out which side you're on

        turretControlTurn(desiredAngle, robotData, shooterData);
        if(getWheelVel() > readyShootLimit - 30 && getWheelVel() < readyShootLimit + 30){
            shooterData.readyShoot = true;
        }

        //you don't see a valid target
        if(desiredAngle == 180){
            if(!robotData.limelightData.validTarget && (std::abs(getFieldRelativeTurretAngle(robotData, shooterData) - desiredAngle)  < 5)){
                shooterData.readyReject = true;
            } else {
                shooterData.readyReject = false;
            }
        } else if(desiredAngle == 0){
            if(!robotData.limelightData.validTarget && (getFieldRelativeTurretAngle(robotData, shooterData) < desiredAngle + 5 || getFieldRelativeTurretAngle(robotData, shooterData) > desiredAngle - 5 + 360)){
                shooterData.readyReject = true;
            } else {
                shooterData.readyReject = false;
            }
        }
    }
}

//for set position checking to see if flywheel is up to speed
void Shooter::checkReadyShoot(ShooterData &shooterData){
    if (getWheelVel() > readyShootLimit)
    {
        shooterData.readyShoot = true;
    }
    else
    {
        shooterData.readyShoot = false;
    }
}

//for checking voltage and setting the set shot wheel speed accordingly
void Shooter::setShooterWheel(double speed){
    if(frc::DriverStation::GetBatteryVoltage() > 12.5){
        flyWheelLead_pidController.SetReference(speed, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
    }else{
        flyWheelLead_pidController.SetReference(speed+20, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
    }
}



void Shooter::saTurret(const RobotData &robotData, ShooterData &shooterData){

    if(isTurretStatic){
        //if static turret bring to front and dont move 
        //NEED TO CHANGE TO FACING TOWARD CLIMB ARMS
        setTurret_Pos(turretZeroRev, shooterData);

    }else{ //ok cool we can move now

        if(robotData.controlData.usingTurretDirection){
            turretControlTurn(robotData.controlData.saTurretDirectionController, robotData, shooterData);

        }else{

            if(robotData.limelightData.validTarget == 1){
                //if youre within 2 degrees of the target you can stop turning (mitigates jerky movement)

                if(robotData.limelightData.distanceOffset < 7){
                    if(std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 10){
                        shooterTurret.Set(0);
                    }else{
                        //turn the turret to face the target
                        //accounts for if the robot is turning and adds more power
                        setTurret_Pos(robotData.limelightData.desiredTurretAngle+turretGyroOffset(robotData.gyroData.rotationalRate), shooterData);
                    }
                }else{
                    if(std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 2){
                        shooterTurret.Set(0);
                    }else{
                        //turn the turret to face the target
                        //accounts for if the robot is turning and adds more power
                        setTurret_Pos(robotData.limelightData.desiredTurretAngle+turretGyroOffset(robotData.gyroData.rotationalRate), shooterData);
                    } 
                }
                
            }
            
        }

    }

    //DECOMMISIONED CODE
    //case: you're spinning in circles and you reach one of your limits but youre still spinning so you lose sight of the target
    //turn the limelight to the last know position
    //this code only really works if the robot itself isn't moving anywhere crazy and you're just spinning in a circle

    // if(robotData.limelightData.validTarget == 1){ //valid target set the turret position to the desired one from limelight
    //     validTargetTurretPos = robotData.limelightData.desiredTurretAngle;

    //     setTurret_Pos(robotData.limelightData.desiredTurretAngle, shooterData);

    // }else if(robotData.limelightData.validTarget == 0){ //if you dont see a target,
    //     setTurret_Pos(validTargetTurretPos, shooterData);

    // }
    
}

/**
 * turns the turret to a position dicated by the joystick control using field oriented location
 * @param controlTurretDirection direction given by joystick control (gives 0-360 degrees)
 */

void Shooter::turretControlTurn(float controlTurretDirection, const RobotData &robotData, ShooterData &shooterData){
    //THIS WILL CHANGE NEED BRIANS CODE FOR ROBOT DIRECTION
    float robotDirection = robotData.drivebaseData.odometryYaw; //in degrees
    float turretTurnPos;

    turretTurnPos = (controlTurretDirection - robotDirection) + turretMiddleDegrees; //calculates turret pos based on what we know to be the center of the bot
    
    //is this code necessary??? I don't think it should ever be over or under??????
    if(turretTurnPos < 0 || turretTurnPos > turretFullRotationDegrees){
        if(turretTurnPos < 0){
            turretTurnPos += 360;
        }else if(turretTurnPos > turretFullRotationDegrees){
            turretTurnPos -=360;
        }
    }
    
    if(turretTurnPos > 360){
        float turretTurnPos2;

        turretTurnPos2 = turretTurnPos - 360;

        //checks to see which of the two values is closer to the current turret value and go to that position
        if(std::abs(robotData.shooterData.currentTurretAngle-turretTurnPos) < std::abs(robotData.shooterData.currentTurretAngle-turretTurnPos2)){
            setTurret_Pos(turretTurnPos, shooterData);
        }else{
            setTurret_Pos(turretTurnPos2, shooterData);
        }
    }else{
        setTurret_Pos(turretTurnPos, shooterData);

    }

    
}


/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * BENCH TEST CODE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * */
void Shooter::TestPeriodic(const RobotData &robotData, ShooterData &shooterData){
    frc::SmartDashboard::PutBoolean("Shooter abs encoder working", encoderPluggedInHood(shooterData));
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
        if (encoderPluggedInHood(shooterData) && encoderInRange(shooterData)){ //checks if the encoder is working
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

        flyWheelLead.Set(shooterData.benchTestFlyWheelSpeed);
        //flyWheelLead.Set(shooterData.benchTestFlyWheelSpeed);
    } else {
        shooterData.benchTestShooterHoodSpeed = 0; //if not testing shooter, then the speed of the motors is set to 0
        shooterData.benchTestFlyWheelSpeed = 0;
    }
}

//checks if the encoder is plugged in and giving an output
bool Shooter::encoderPluggedInHood(const ShooterData &shooterData){
    if (shooterHoodEncoderAbs.GetOutput() > 0.01) { //checks if the output of the abs encoder is actually reading a signal
        //updates encoder values
        if (tickCount > 40){
            shooterHoodEncoderRev.SetPosition(HoodabsoluteToREV(shooterHoodEncoderAbs.GetOutput()));
            tickCount = (tickCount + 1) % 50;
        } else {
            tickCount = (tickCount + 1) % 50;
        }
    
        return true; //returns true to indicate that the encoder is functioning
    } else {
        return false;
    }
}

//checks if the encoder is plugged in and giving an output
bool Shooter::encoderPluggedInTurret(const ShooterData &shooterData){
    if (shooterTurretEncoderAbs.GetOutput() > 0.01) { //checks if the output of the abs encoder is actually reading a signal
        //updates encoder values
        shooterTurretEncoderRev.SetPosition(turretAbsoluteToREV(shooterTurretEncoderAbs.GetOutput()));
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



