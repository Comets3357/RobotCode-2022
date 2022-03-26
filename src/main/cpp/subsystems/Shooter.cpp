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
    // frc::SmartDashboard::PutNumber("hood Roller speed", 0);
    // frc::SmartDashboard::PutNumber("flywheel speed", 0);

}

void Shooter::shooterHoodInit()
{
    shooterHood.RestoreFactoryDefaults();
    shooterHood.SetInverted(false);
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
    flyWheelLead_pidController.SetP(0.00001); //0.002
    flyWheelLead_pidController.SetI(0);
    flyWheelLead_pidController.SetD(0); //0.005
    flyWheelLead_pidController.SetIZone(0);
    flyWheelLead_pidController.SetFF(0.000219); //0.0002
    flyWheelLead_pidController.SetOutputRange(-1,1);
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
    hoodRoller_pidController.SetP(0.00001); //0.002
    hoodRoller_pidController.SetI(0);
    hoodRoller_pidController.SetD(0); //0.005
    hoodRoller_pidController.SetIZone(0);
    hoodRoller_pidController.SetFF(0.0000955); //0.0002
    hoodRoller_pidController.SetOutputRange(-1,1);
    hoodRoller.BurnFlash();           
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

    shooterTurret.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

}

void Shooter::EnabledInit(ControlData &controlData, ShooterData &shooterData)
{
    controlData.shootMode = shootMode_none;
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
        //shooterHood.Set(0);

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

    //updates rev encoder if abs encoder is working
    encoderPluggedIn();

    saTurret(robotData, shooterData);
    //if nothing is happening then update the isTurretStatic value based on the button control
    //can this be outside the statement?
    isTurretStatic = robotData.controlData.staticTurret;


    //SHOOTING LOGIC
    if(robotData.controlData.shootMode == shootMode_vision){ // Aiming with limelight

        // if(robotData.limelightData.validTarget == 1){ //valid target
        //     validTargetTurretPos = robotData.limelightData.desiredTurretAngle;
        //     setTurret_Pos(robotData.limelightData.desiredTurretAngle, shooterData);

        // }else{
        //     setTurret_Pos(validTargetTurretPos, shooterData);
        // }

        //set the hood and flywheel using pids to the desired values based off the limelight code
        //checks battery voltage and increases velocity if it doesn't have enough power

        // double flywheelSpeed = frc::SmartDashboard::GetNumber("flywheel speed", 0);
        // double hoodRollerSpeed = frc::SmartDashboard::GetNumber("hood Roller speed", 0);

        if(frc::DriverStation::GetBatteryVoltage() > 12.6){
            flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
            hoodRoller_pidController.SetReference(robotData.limelightData.desiredHoodRollerVel, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
        }else{
            flyWheelLead_pidController.SetReference(robotData.limelightData.desiredVel + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
            hoodRoller_pidController.SetReference(robotData.limelightData.desiredHoodRollerVel + 20, rev::CANSparkMaxLowLevel::ControlType::kVelocity);        
        }    

        shooterHood_pidController.SetReference(HoodabsoluteToREV(HoodconvertFromAngleToAbs(robotData.limelightData.desiredHoodPos)), rev::CANSparkMaxLowLevel::ControlType::kPosition);

        //once it's a high enough velocity and turret is in place its ready for indexer to run
        if (shooterData.readyShoot == false && (getWheelVel() > (robotData.limelightData.desiredVel - 30)) && (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 2))
        //if you're not in readyShoot yet and the wheel velocity is above 30 under the desire velocity, readyShoot will become true
        {
            shooterData.readyShoot = true;
        }
        //else
        else if (shooterData.readyShoot == true && (getWheelVel() < (robotData.limelightData.desiredVel - 100)) && (std::abs(robotData.limelightData.desiredTurretAngle - robotData.shooterData.currentTurretAngle) <= 2))
        // if you're already in readyShoot, you'll only exit readyShoot if the wheel velocity drops below 100 below the desired velocity
        {
            shooterData.readyShoot = false;
        }
    
//FIXED SHOTS 
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
    if(robotData.controlData.mTurret >= 0.01 || robotData.controlData.mTurret <= -0.01){ //accounts for deadzone
        shooterTurret.Set(robotData.controlData.mTurret*.5);
    }else{
        shooterTurret.Set(0);
    }
    //hood to joystick controls
     if(robotData.controlData.mHood >= 0.01 || robotData.controlData.mHood <= -0.01){ //accounts for deadzone
        shooterHood.Set(robotData.controlData.mHood*.2);
    }else{
        shooterHood.Set(0);
    }

    //zeros hood pos
    if(robotData.controlData.mZeroHood)
    {
        shooterTurretEncoderRev.SetPosition(0);
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
    frc::SmartDashboard::PutNumber("shooter Turret ABS", shooterTurretEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("shooter Turret REV", shooterTurretEncoderRev.GetPosition());

    //frc::SmartDashboard::PutNumber("shooter hood abs", shooterHoodEncoderAbs.GetOutput());
    //frc::SmartDashboard::PutNumber("shooter hood rev", shooterHoodEncoderRev.GetPosition());


    frc::SmartDashboard::PutBoolean("shooter ready shoot", shooterData.readyShoot);
    frc::SmartDashboard::PutNumber("HOOD ANGLE", HoodconvertFromAbsToAngle(shooterHoodEncoderAbs.GetOutput()));
    frc::SmartDashboard::PutNumber("flywheel vel", flyWheelLeadEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("desired flywheel vel", robotData.limelightData.desiredVel);

    shooterData.currentTurretAngle = turretConvertFromAbsToAngle(shooterTurretEncoderAbs.GetOutput());
    
    frc::SmartDashboard::PutNumber("turret angle", shooterData.currentTurretAngle);
    frc::SmartDashboard::PutNumber("REEEEE", turretAbsoluteToREV(turretConvertFromAngleToAbs(robotData.limelightData.desiredTurretAngle)));

    frc::SmartDashboard::PutNumber("Gyro angle", (robotData.drivebaseData.currentPose.Rotation().Radians().to<double>())*(180/pi));

    // frc::SmartDashboard::PutNumber("desired hood pos", robotData.limelightData.desiredHoodPos);
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

double Shooter::turretGyroOffset(double value){
    double slope = (turretGyroOffsetMax - turretGyroOffsetMin)/(rotationalRateMax - rotationalRateMin);
    double b = turretGyroOffsetMin - (slope*rotationalRateMin);
    return ((value*slope) + b);
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
        shooterHood_pidController.SetReference(hoodrevOut, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(2040 + 15);

        readyShootLimit = 1990 + 15;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(hoodrevOut, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(2040 + 15);

        readyShootLimit = 1990 + 15;
    }
}

void Shooter::wall(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(-32.33, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(1860 - 15);

        readyShootLimit = 1830 - 15;;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(-32.33, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(1860 - 15);

        readyShootLimit = 1800 - 15;
    }
}

void Shooter::fender(const RobotData &robotData)
{
    if (robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(hoodrevIn-0.25, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(1690 - 15);

        readyShootLimit = 1670 - 15;
    }
    else if (!robotData.controlData.upperHubShot)
    {
        shooterHood_pidController.SetReference(hoodrevIn-0.25, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        setShooterWheel(1240 - 15);
        
        readyShootLimit = 1220 - 15;
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

void Shooter::DisabledPeriodic(const RobotData &robotData, ShooterData &shooterData){
    updateData(robotData, shooterData);
}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * BENCH TEST CODE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 **/

void Shooter::TestInit(){
    //sets pid stuff for bench test
    shooterHood_pidController.SetP(0.378, 0);
    shooterHood_pidController.SetOutputRange(-0.5, 0.5, 0);
}

void Shooter::TestPeriodic(const RobotData &robotData, ShooterData &shooterData){
    frc::SmartDashboard::PutBoolean("Shooter abs encoder working", encoderPluggedIn());
    frc::SmartDashboard::PutBoolean("Shooter abs encoder reading in correct range", encoderInRange(shooterData));
    frc::SmartDashboard::PutBoolean("Shooter hit bottom dead stop?", shooterData.bottomDeadStop);
    frc::SmartDashboard::PutBoolean("Shooter hit top dead stop?", shooterData.topDeadStop);
    frc::SmartDashboard::PutNumber("Shooter Abs Encoder Value", shooterHoodEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("Shooter min extend expected encoder value", hoodabsIn);
    frc::SmartDashboard::PutNumber("Shooter max extend expected encoder value", hoodabsOut);
    frc::SmartDashboard::PutNumber("Shooter hood power", shooterData.benchTestShooterHoodSpeed);
    frc::SmartDashboard::PutNumber("Shooter Fly Wheel Speed", shooterData.benchTestFlyWheelSpeed);

    checkDeadStop(shooterData);

    //runs the bench test sequence
    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && (robotData.controlData.manualBenchTest || robotData.controlData.autoBenchTest)){ //checks if we're testing shooter
        if (encoderPluggedIn() && encoderInRange(shooterData)){ //checks if the encoder is working
            if (robotData.benchTestData.stage == 0){
                //run hood forwards
                if (!robotData.benchTestData.PIDMode){
                    shooterData.benchTestShooterHoodSpeed = -.07; //sets the speed of the hood
                    shooterData.benchTestFlyWheelSpeed = 0; //sets the speed of the fly wheel
                } else {
                    shooterData.benchTestFlyWheelSpeed = 0;
                    shooterHood_pidController.SetReference(hoodrevOut, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0); //runs the hood out
                }
            } else if (robotData.benchTestData.stage == 1){
                //run hoods backwards
                if (!robotData.benchTestData.PIDMode){
                    shooterData.benchTestShooterHoodSpeed = .07;
                    shooterData.benchTestFlyWheelSpeed = 0;
                } else {
                    shooterData.benchTestFlyWheelSpeed = 0;
                    shooterHood_pidController.SetReference(hoodrevIn, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0); //runs the hood in
                }
            } else if (robotData.benchTestData.stage == 2){
                //run fly wheel
                shooterData.benchTestShooterHoodSpeed = 0;
                shooterData.benchTestFlyWheelSpeed = .25; //change back to .25 when done testing
            } else {
                shooterData.benchTestShooterHoodSpeed = 0; //if the stage isn't within 0 to 2, then speeds get set to 0
                shooterData.benchTestFlyWheelSpeed = 0;
                shooterHood.Set(0);
                flyWheelLead.Set(0);
            }
        } else {
            shooterData.benchTestShooterHoodSpeed = 0; //if encoders don't work, then set the speeds to 0
            shooterData.benchTestFlyWheelSpeed = 0;
            shooterHood.Set(0);
            flyWheelLead.Set(0);
        }

        //if statement to make sure the speed doesn't interfere with PID mode
        if (!robotData.benchTestData.PIDMode){
            //sets the speed of the motors according to the variables set in the above if statement ^ (unless the hood hit a dead stop)
            if (!shooterData.topDeadStop && !shooterData.bottomDeadStop){
                shooterHood.Set(shooterData.benchTestShooterHoodSpeed);
            } else {
                shooterHood.Set(0);
            }
        }

        flyWheelLead.Set(shooterData.benchTestFlyWheelSpeed);
        //flyWheelLead.Set(shooterData.benchTestFlyWheelSpeed);
    } else {
        shooterData.benchTestShooterHoodSpeed = 0; //if not testing shooter, then speeds get set to 0
        shooterData.benchTestFlyWheelSpeed = 0;
        shooterHood.Set(0);
        flyWheelLead.Set(0);
    }
}

//checks if the encoder is plugged in and giving an output
bool Shooter::encoderPluggedIn(){
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
    if (shooterData.benchTestShooterHoodSpeed < 0 && shooterHoodEncoderAbs.GetOutput() < hoodabsOut + .01){
        shooterData.topDeadStop = true;
        shooterData.bottomDeadStop = false;
    } else if (shooterData.benchTestShooterHoodSpeed > 0 && shooterHoodEncoderAbs.GetOutput() > hoodabsIn - .01){
        shooterData.topDeadStop = false;
        shooterData.bottomDeadStop = true;
    } else {
        shooterData.topDeadStop = false;
        shooterData.bottomDeadStop = false;
    }
}