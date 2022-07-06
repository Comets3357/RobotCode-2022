    #include "subsystems/Climb.h"
#include "RobotData.h"

// 1 overtild the bot so that there cannot be any bounceback
// 2 wait until the bot is swinging forward before tilting the bot towards the bar
// 3 when pulling off of the bar only pull when the gyro senses that the arms are on the bar
// 4 shorten the extent when going up to the next bar so it will lock when moving onto the bar
// 5 wait until the bot stops swinging

void Climb::RobotInit()
{
    //do other init stuff (probably more)
    climbArms.RestoreFactoryDefaults();
    climbElevator.RestoreFactoryDefaults();

    //do other init stuff (probably more)
    climbArms.RestoreFactoryDefaults();
    climbElevator.RestoreFactoryDefaults();

    //sets the inversion of the motors
    climbArms.SetInverted(false);
    climbElevator.SetInverted(false);
    climbArms.SetSmartCurrentLimit(45);
    climbElevator.SetSmartCurrentLimit(80);
    //climbElevator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,140);
    //climbArms.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,250);

    //motor idlemode
    climbElevator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    climbArms.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    //resets encoders
    climbElevatorEncoder.SetPosition(0);
    climbArmsEncoder.SetPosition(0);

    //PIDS
    climbElevator_pidController.SetP(0.18, 0); //off bar
    climbElevator_pidController.SetOutputRange(-elevatorSpeed, elevatorSpeed, 0);
    climbElevator_pidController.SetP(1, 1); //on bar temp
    climbElevator_pidController.SetOutputRange(-elevatorSpeed, elevatorSpeed, 1);

    climbArms_pidController.SetP(0.25, 0);
    climbArms_pidController.SetOutputRange(-armsSpeed, armsSpeed, 0);
    climbArms_pidController.SetP(0.5, 1);
    climbArms_pidController.SetOutputRange(-armsSpeed, armsSpeed, 1);

    climbElevator.BurnFlash();
    climbArms.BurnFlash();
    
}

void Climb::RobotPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    //updates
    updateData(robotData, climbData);

    //checks if the robot is in climb mode
    if (robotData.controlData.mode == mode_climb_sa || robotData.controlData.mode == mode_climb_manual)
    {
        if (robotData.controlData.mode == mode_climb_manual)
        { //updates whether or not the robot is in manual or semiAuto mode
            manual(robotData, climbData);
        }else{
            //checks is turret is facing forward
            if((std::abs(turretMiddleDegrees - robotData.shooterData.currentTurretAngle) >= 30)) //if you're centered forward you can climb
            {
                //sets powers to 0 if the mode is changed out of climb mode
                climbElevator.Set(0);
                climbArms.Set(0);
            }else{
                semiAuto(robotData, climbData);
            }
        }
        
        
    } 
    else
    {
        //sets powers to 0 if the mode is changed out of climb mode
        climbElevator.Set(0);
        climbArms.Set(0);
    }

    //softlimit max for elevator
    if (climbElevatorEncoder.GetPosition() <= -145 && climbElevator.Get() < 0)
    {
        //sets the elevator power to 0 if it is above the max
        climbElevator.Set(0);
    }

    if (robotData.controlData.mode == mode_climb_sa || robotData.controlData.mode == mode_climb_manual) {
        //runs if the climb zeros
        if (robotData.controlData.climbZeroing)
        {
            //toggles zeroing when the button is pressed so it can be stopped
            climbData.zeroing = !climbData.zeroing;
        }

        //runs after the zeroing button is pressed
        // frc::SmartDashboard::PutBoolean("climbZeroing", climbData.zeroing);
        if (climbData.zeroing)
        {
            if (climbArmsAbs.GetOutput() > 0.03) {
                //sets the elevator going down really slow
                climbElevator.Set(0.1);
                climbArms.Set(0.2);
                //stops the motor and ends the zeroing when the limit switch changes
                if (!elevatorLimit.Get())
                {
                    //sets elevator power to 0 when done zeroing
                    climbElevator.Set(0);
                    //resets encoder because it is zeroed
                    climbElevatorEncoder.SetPosition(0);
                }
                if (climbArmsAbs.GetOutput() >= climbArmsZero)
                {
                    climbArms.Set(0);
                    climbArmsEncoder.SetPosition(0);
                }
                if (!elevatorLimit.Get() && climbArmsAbs.GetOutput() >= climbArmsZero)
                {
                    climbData.zeroing = false;
                }
            }

        }
    }

    // if (climbArmsAbs.GetOutput() > 0.811 && climbArms.Get() > 0)
    // {
    //     climbArms.Set(0);
    // }
}

//manual
void Climb::manual(const RobotData &robotData, ClimbData &climbData)
{
    //manualy sets the elevator with limit. I use the limit switch as the bottom limit
    if ((climbElevatorEncoder.GetPosition() <= -145 && climbElevator.Get() < 0) || (!elevatorLimit.Get() && climbElevator.Get() > 0))
    {
        //sets power to 0 if it is outside of its deadzone
        climbElevator.Set(0); //control elevator with left stick
    }
    else 
    {
        //deadzone
        if (robotData.controllerData.sLYStick < -0.08 || robotData.controllerData.sLYStick > 0.08)
        {
            //sets the motor power to joystick when joystick is outside of the deadzone
            climbElevator.Set(-robotData.controllerData.sLYStick); //control elevator with left stick); //sets the power to 0 so the elevator stops moving
        }
        else
        {
            //sets power to 0 when nothing is supposed to happen
            climbElevator.Set(0);
        }
    }
    
    

    //manualy sets the arms with limit. The bottom limit is gon because an absolute encode will be there eventually
    if ((climbArmsEncoder.GetPosition() <= -250 && climbArms.Get() < 0) || (climbArms.Get() > 0 && climbArmsAbs.GetOutput() > climbArmsZero))
    {    
        //sets climbarms to zero when outside of limit
        climbArms.Set(0); //control arms with right stick
    }
    else 
    {
        //dead zone
        if (robotData.controllerData.sRYStick < -0.08 || robotData.controllerData.sRYStick > 0.08)
        {
            //sets arm power to joystick when joystick is outside of deadzone
            climbArms.Set(-robotData.controllerData.sRYStick); //sets the power to 0 so the arms stop moving
        }
        else
        {
            //sets power to 0 when nothing is supposed to happen
            climbArms.Set(0);
        }
    }

    if (robotData.controlData.mClimbZeroElevatorRev)
    {
        climbElevatorEncoder.SetPosition(0);
    }

    if (robotData.controlData.mClimbZeroPivotArmsRev)
    {
        climbArmsEncoder.SetPosition(0);
    }
}

void Climb::semiAuto(const RobotData &robotData, ClimbData &climbData)
{

    //listens for climb initiation button and does somthing if it needs to
    climbInit(robotData, climbData);
    //this is the climb sequence where the bot will climb autonomously
    runSequence(robotData, climbData);
    //this will cancel the sequence or pause it
    cancelSequence(robotData, climbData);
    
}

void Climb::climbInit(const RobotData &robotData, ClimbData &climbData)
{
    if (robotData.controlData.saClimbInit && !climbInitiating) //if the climbInit button is pressed then the climb will go up if the climb is down and go down if it is already up. (toggle)
    {
        climbUp = !climbUp; //sets the climbUp to the opposite direction
        climbInitiating = true; //sets the climbInitiating variable to true to 
    }

    if (climbInitiating && climbUp)
    {
        ChangeElevatorSpeed(1,0);
        RunElevatorToPos(140,0,0); //runs the climb up when button is pressed
    }
    else if (climbInitiating && !climbUp)
    { 
        ChangeElevatorSpeed(1,0);
        RunElevatorToPos(0,0,0); //runs the climb back down if you press it again
    }
}

void Climb::cancelSequence(const RobotData &robotData, ClimbData &climbData) //cancels the climb sequence
{

    if (robotData.controlData.saPauseSequence) //checks for the pause button to be pressed
    {
        executeSequence = false; //press a button, semiAuto code stops
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    }

    if (robotData.controlData.saCancelSequence)
    {
        executeSequence = false; //press a button, semiAuto code stops
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
        stage = 0; //resets the stage because it is a cancel
        climbData.bar = 2; //resets bar because it is a cancel
    }
}

//runs climb sequence
void Climb::runSequence(const RobotData &robotData, ClimbData &climbData)
{

    //sets variables to go up to 3rd bar
    if (robotData.controlData.saClimbHeightSequence)
    {//3rd bar
        //stage = 0;//for testing
        executeSequence = true; //press right center button, semiAuto code runs
        climbData.climbing = true;
        targetBar = 3; //reaches to bar 3
        ChangeElevatorSpeed(elevatorSpeed, 0);
    }//sets variables to go up to 4th bar
    else if (robotData.controlData.saClimbTraversalSequence)
    {//4th bar
        //stage = 0;//for testing
        executeSequence = true; //press left center button, semiAuto code runs
        climbData.climbing = true;
        targetBar = 4; //reaches to bar 4
        ChangeElevatorSpeed(elevatorSpeed, 0);
    }

    //starts going up to bar
    if (executeSequence && climbData.bar < 4)
    { //checks if you want to run the sequence, and also if you're already at bar 4, then you can't run it
        if (stage == 0) ChangeElevatorSpeed(elevatorSpeed, 1);
        else if (stage == 1) RunArmsAndElevatorToPos(-1,1,0,0,1); // get value in manual
        else if (stage == 2) ChangeElevatorSpeed(0.5, 1);
        else if (stage == 3) RunElevatorToPos(20,1,0); //test 3
        else if (stage == 4) ChangeElevatorSpeed(1,1);
        else if (stage == 5) RunArmsToPos(20,1,1);
        //top bar transfer
        if (climbData.bar == targetBar-1)
        {
            if (stage == 6) RunArmsAndElevatorToPos(120,0,65,1,1);
            else if (stage == 7) WaitUntilGyro(1, 2, -33, 1);
            else if (stage == 8) RunElevatorToPos(141,1,1);
            else if (stage == 9) ChangeElevatorSpeed(elevatorSpeed,1);
            else if (stage == 10) ChangeArmSpeed(0.7,1);
            else if (stage == 11) TopTransfer();
            else if (stage == 12) {ChangeArmSpeed(1,1); lastTrasfer = false;}
            else if (stage == 13) RunArmsToPos(0,1,1);
            // else if (stage == 16) ChangeElevatorSpeed(0.6, 1);
            // else if (stage == 17) RunElevatorToPos(70,1,1);
            // else if (stage == 18) ChangeElevatorSpeed(elevatorSpeed, 1);
        }
        //transfer onto 3rd bar
        else 
        {
            if (stage == 6) RunArmsAndElevatorToPos(100,0,165,1,1);
            else if (stage == 7) WaitUntilGyro(-1, 1, -45, 1);
            else if (stage == 8) RunElevatorToPos(150,1,1);
            else if (stage == 9) RunArmsToPos(100,1,1);
            else if (stage == 10) RunElevatorToPos(110,1,1);
            else if (stage == 11) ChangeElevatorSpeed(elevatorSpeed, 1);
        }
        if (stage == 12)
        { //do it again if the bot isnt on the top bar
            //resets everything
            stage = 0;
            //adds 1 to bar because the robot has gone up
            climbData.bar++;
            //if the bar is at its target then stop else keep going
            if (climbData.bar == targetBar)
            {
                executeSequence = false;
            }

            //resets powers
            climbArms.Set(0);
            climbElevator.Set(0);
        }
    }
}

void Climb::RunArmsGyroLower()
{

}

void Climb::RunArmsGyroUpper()
{
    
}

void Climb::wait(int time)
{
    timer += 1;
    if (timer > time)
    {
        timer = 0;
        stage += 1;
    }
}

void Climb::WaitUntilArmsOnBar()
{
    if (climbArmsAbs.GetOutput() < 0.8)
    {
        stage += 1;
    }
}

//Runs the elevator to a specific location, specified in semiAuto
void Climb::RunElevatorToPos(int position, int stageAdd, int onBar)
{
    if (climbElevatorEncoder.GetPosition() > -position + 1 || climbElevatorEncoder.GetPosition() < -position - 1)
    {
        elevatorRunning = true;
        //only moves when angular rate is low to reduce swinging
        if (onBar==1){
            if (abs(angularRate) < 70)
            {
                climbElevator_pidController.SetReference(-position, rev::CANSparkMax::ControlType::kPosition, onBar);
            }
            else
            {
                climbElevator.Set(0);
                
            }
        } else{
            climbElevator_pidController.SetReference(-position, rev::CANSparkMax::ControlType::kPosition, onBar);
        }
        elevatorRunning = true;
        
    }
    else
    {
        elevatorRunning = false;
        climbInitiating = false;
        stage += stageAdd;
    }
}

//runs the arms to a specific location, specified in semiAuto
void Climb::RunArmsToPos(int position, int stageAdd, int onBar)
{

    if (climbArmsEncoder.GetPosition() > -position + 1 || climbArmsEncoder.GetPosition() < -position - 1)
    {
        armsRunning = true;
        climbArms_pidController.SetReference(-position, rev::CANSparkMax::ControlType::kPosition, onBar);
    }
    else
    {
        armsRunning = false;
        stage += stageAdd;
    }
}

//runs both elevator and arms
void Climb::RunArmsAndElevatorToPos(int elevatorPos, int elevatorBar, int armsPos, int armsBar, int stageAdd)
{
    RunElevatorToPos(elevatorPos, 0, elevatorBar);
    RunArmsToPos(armsPos, 0, armsBar);
    //waits until the arms and the elevator are done running
    if (!elevatorRunning && !armsRunning)
    {
        climbArms.Set(0);
        climbElevator.Set(0);
        stage+=stageAdd;
    }

}

//trasfers on top bar
void Climb::TopTransfer()
{

    //checks for angle where the bot pulls off bar
    if (angle < -44 || elevatorRunning)
    {
        climbArms.Set(0);
        ChangeElevatorSpeed(0.7, 0);
        RunElevatorToPos(110,1,0);
        lastTrasfer = true;
    } else if (!lastTrasfer){
        
        //runs arms down when not at the angle for transfer
        climbArms_pidController.SetReference(-200, rev::CANSparkMax::ControlType::kPosition, 1);
    }
}

void Climb::CheckArms()
{
    //checks to see if arms are in the right place
    if (climbArmsAbs.GetOutput() < 0.8)
    {
        stage += 1;
    } else 
    {
        //goes back if the arms are not in the right place
        stage -= 2;
    }
}

//checks for a specific gyro value before moving on
void Climb::WaitUntilGyro(int cmp, int arate, float gyroValue, int stageAdd)
{
    //checks greater than a value
    if (cmp == 1)
    {
        if (gyroValue < angle )
        {
            if (arate == 1 && angularRate > 0)
            {
                stage += stageAdd;
            }
            else if (arate == -1 && angularRate < 0)
            {
                stage += stageAdd;
            }
            if (arate == 2)
            {
                stage += stageAdd;
            }
        }
    }
    //checks less than a value
    else if (cmp == -1)
    {
        if (gyroValue > angle)
        {
            if (arate == 1 && angularRate > 0)
            {
                stage += stageAdd;
            }
            else if (arate == -1 && angularRate < 0)
            {
                stage += stageAdd;
            }
        }
    }
}

void Climb::ChangeElevatorSpeed(float speed, int stageAdd)
{
    //changes speed on elevator on bot PIDS
    climbElevator_pidController.SetOutputRange(-speed, speed, 0);
    climbElevator_pidController.SetOutputRange(-speed, speed, 1);
    stage += stageAdd;
}


void Climb::ChangeArmSpeed(float speed, int stageAdd)
{
    //changed speed on arms on both PIDS
    climbArms_pidController.SetOutputRange(-speed, speed, 0);
    climbArms_pidController.SetOutputRange(-speed, speed, 1);
    stage += stageAdd;
}

//zeros elevator
void Climb::ZeroElevator(float power, int stageAdd)
{
    //runs elevator if limit is not reached
    if (elevatorLimit.Get())
    {
        elevatorRunning = true;
        climbElevator.Set(abs(power));
        zeroingTimer += 1;
    }
    else 
    {
        climbElevator.Set(0);
        elevatorRunning = false;
        stage += stageAdd;
        zeroingTimer = 0;
    }
    //stops zeroing if going for too long
    if (zeroingTimer > 20)
    {
        climbElevator.Set(0);
        elevatorRunning = false;
        stage += stageAdd;
        zeroingTimer = 0;
    }
}


// updates encoder and gyro values
void Climb::updateData(const RobotData &robotData, ClimbData &climbData)
{
    angularRate = robotData.gyroData.angularMomentum;
    angle = robotData.gyroData.rawRoll;
    // climbData.elevatorAmp = climbElevator.GetOutputCurrent();
    // climbData.armsAmp = climbArms.GetOutputCurrent();
    // climbData.elevatorTemp = climbElevator.GetMotorTemperature();
    // climbData.armsTemp = climbArms.GetMotorTemperature();
    // climbData.elevatorPos = climbElevatorEncoder.GetPosition();
    // climbData.armsPos = climbArmsEncoder.GetPosition();
    // climbData.armsAbsPos = climbArmsAbs.GetOutput();
    climbData.stage = stage;
    // climbData.angle = angle;
    // climbData.angularRate = angularRate;
    // climbData.elevatorLimit = elevatorLimit.Get();
    
    frc::SmartDashboard::PutNumber("climb elevator rev encoder", climbElevatorEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("climb elevator limit switch", elevatorLimit.Get());
    // frc::SmartDashboard::PutNumber("elevator amps", elevatorAmperage);
    // frc::smartDashboard::PutNumber("Arms amps", armsAmperage);
    // frc::SmartDashboard::PutNumber("climb stage", stage);
    // frc::smartDashboard::PutBoolean("running sequence", executeSequence);
    frc::SmartDashboard::PutNumber("climb pivot rev encoder", climbArmsEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("climb pivot abs encoder", climbArmsAbs.GetOutput());
    // frc::smartDashboard::PutNumber("which bar is bot on bar", climbData.bar);
    // frc::smartDashboard::PutBoolean("zeroing", climbData.zeroing);
    // frc::smartDashboard::PutNumber("elevator motor temp", elevatorTemp);
    // frc::smartDashboard::PutNumber("arms temp", armsTemp);
    // frc::SmartDashboard::PutNumber("climb angle", angle);
    // frc::SmartDashboard::PutNumber("climb angle rate", angularRate);
    frc::SmartDashboard::PutNumber("angle", angle);
}



//sets powers to 0 when disabled
void Climb::DisabledInit()
{
    //sets motors to 0 for cuz disabled
    climbElevator.Set(0);
    climbArms.Set(0);
}


//runs update when disabled
void Climb::DisabledPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    updateData(robotData, climbData);
}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * BENCH TEST CODE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 **/

void Climb::TestInit(ClimbData &climbData){
    elevatorLimitSwitchWorking(climbData); //checks if the limits switch starts in false, which it's supposed to; if it doesn't start in false, then the bench test won't run
}

void Climb::TestPeriodic(const RobotData &robotData, ClimbData &climbData){
    //diagnosing issues with smart dashboard
    frc::SmartDashboard::PutNumber("Climb arms abs encoder value", climbArmsAbs.GetOutput());
    frc::SmartDashboard::PutNumber("Climb arms rev encoder value", climbArmsEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climb elevator rev encoder value", climbElevatorEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("Climb limit switch working", climbData.limitSwitchWorking);
    frc::SmartDashboard::PutBoolean("Climb elevator limit switch state", elevatorLimit.Get());
    frc::SmartDashboard::PutNumber("Climb arms speed", climbData.benchTestClimbArmsSpeed);
    frc::SmartDashboard::PutNumber("Climb elevator speed", climbData.benchTestClimbElevatorSpeed);
    frc::SmartDashboard::PutBoolean("Climb elevator hit top dead stop", climbData.elevatorUpperLimit);
    frc::SmartDashboard::PutBoolean("Climb elevator hit bottom dead stop", climbData.elevatorLowerLimit);
    frc::SmartDashboard::PutBoolean("Climb arms hit outer dead stop", climbData.armsLowerLimit);
    frc::SmartDashboard::PutBoolean("Climb arms hit inner dead stop", climbData.armsUpperLimit);

    //calls dead stop functions so the motors know when to stop
    checkElevatorDeadStop(climbData);
    checkArmsDeadStop(climbData);

    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && (robotData.controlData.manualBenchTest || robotData.controlData.autoBenchTest)){ //checks if we're testing climb
        if (climbData.limitSwitchWorking && encoderPluggedIn() && encoderInRange()){ //checks if the limit switch and abs encoder are working (doesn't run bench test if not working)
            if (robotData.benchTestData.stage == 0){
                //move climb arms forwards
                if (!robotData.benchTestData.PIDMode){
                    climbData.benchTestClimbArmsSpeed = -.1; //sets the arms speed
                    climbData.benchTestClimbElevatorSpeed = 0; //sets the elevator speed
                } else {
                    climbData.benchTestClimbElevatorSpeed = 0; //sets elevator speed to 0
                    climbData.benchTestClimbArmsSpeed = -.1; //sets the arms speed
                    climbArms_pidController.SetReference(-250, rev::CANSparkMax::ControlType::kPosition); //arms move according to pid
                }
            } else if (robotData.benchTestData.stage == 1){
                //move climb arms backwards
                if (!robotData.benchTestData.PIDMode){
                    climbData.benchTestClimbArmsSpeed = .1;
                    climbData.benchTestClimbElevatorSpeed = 0;
                } else {
                    climbData.benchTestClimbElevatorSpeed = 0;
                    climbData.benchTestClimbArmsSpeed = .1;
                    climbArms_pidController.SetReference(0, rev::CANSparkMax::ControlType::kPosition); //arms move according to pid
                }
            } else if (robotData.benchTestData.stage == 2){
                //move climb elevator up
                if (!robotData.benchTestData.PIDMode){
                    climbData.benchTestClimbArmsSpeed = 0;
                    climbData.benchTestClimbElevatorSpeed = -.1;
                } else {
                    climbData.benchTestClimbArmsSpeed = 0;
                    climbData.benchTestClimbElevatorSpeed = -.1;
                    climbElevator_pidController.SetReference(-140, rev::CANSparkMax::ControlType::kPosition); //elevator moves according to pid
                }
            } else if (robotData.benchTestData.stage == 3){
                //move climb elevator down
                if (!robotData.benchTestData.PIDMode){
                    climbData.benchTestClimbArmsSpeed = 0;
                    climbData.benchTestClimbElevatorSpeed = .1;
                } else {
                    climbData.benchTestClimbArmsSpeed = 0;
                    climbData.benchTestClimbElevatorSpeed = .1;
                    climbElevator_pidController.SetReference(0, rev::CANSparkMax::ControlType::kPosition); //elevator moves according to pid
                }
            } else {
                climbData.benchTestClimbArmsSpeed = 0; //if the stage isn't within 0 to 3, then speeds get set to 0
                climbData.benchTestClimbElevatorSpeed = 0;
                climbArms.Set(0);
                climbElevator.Set(0);
            }
        } else {
            climbData.benchTestClimbArmsSpeed = 0; //if sensors don't work, then speeds get set to 0
            climbData.benchTestClimbElevatorSpeed = 0;
            climbArms.Set(0);
            climbElevator.Set(0);
        }

        //if statement to make sure the speed doesn't interfere with PID mode
        if (!robotData.benchTestData.PIDMode){
            //uses the variables in the above ^ code to set the motor speeds (also checks if the motor has hit a dead stop, and if so, the motor stops)
            if (!climbData.elevatorUpperLimit && !climbData.elevatorLowerLimit){
                climbElevator.Set(climbData.benchTestClimbElevatorSpeed);
            } else {
                climbElevator.Set(0); //sets the speed to 0 if the motor is at a dead stop
            }

            if (!climbData.armsUpperLimit && !climbData.armsLowerLimit){
                climbArms.Set(climbData.benchTestClimbArmsSpeed);
            } else {
                climbArms.Set(0); //sets the speed to 0 if the motor is at a dead stop
            }
        }
    } else {
        climbData.benchTestClimbArmsSpeed = 0; //if not testing climb, then speeds get set to 0
        climbData.benchTestClimbElevatorSpeed = 0;
        climbArms.Set(0);
        climbElevator.Set(0);
    }

    //calls dead stop functions so the motors know when to stop - necessary to call it again for automatic bench test
    checkElevatorDeadStop(climbData);
    checkArmsDeadStop(climbData);
}

//sets the limits and sets variables to the limits to let the TestPeriodic function know when to stop running the motors
void Climb::checkElevatorDeadStop(ClimbData &climbData){
    if (climbElevatorEncoder.GetPosition() <= -140 && climbData.benchTestClimbElevatorSpeed < 0){
        climbData.elevatorUpperLimit = true;
        climbData.elevatorLowerLimit = false;
    } else if (!elevatorLimit.Get() && climbData.benchTestClimbElevatorSpeed > 0){
        climbData.elevatorUpperLimit = false;
        climbData.elevatorLowerLimit = true;
    } else {
        climbData.elevatorUpperLimit = false;
        climbData.elevatorLowerLimit = false;
    }
}

//sets the limits and sets variables to the limits to let the TestPeriodic function know when to stop running the motors
void Climb::checkArmsDeadStop(ClimbData &climbData){
    if (climbArmsEncoder.GetPosition() <= -250 && climbData.benchTestClimbArmsSpeed < 0){
        climbData.armsUpperLimit = false;
        climbData.armsLowerLimit = true;
    } else if (climbArmsEncoder.GetPosition() >= 0 && climbData.benchTestClimbArmsSpeed > 0){
        climbData.armsUpperLimit = true;
        climbData.armsLowerLimit = false;
    } else {
        climbData.armsUpperLimit = false;
        climbData.armsLowerLimit = false;
    }
}

//checks if the limit switch is working - only run in TestInit
void Climb::elevatorLimitSwitchWorking(ClimbData &climbData){
    if (elevatorLimit.Get()){
        climbData.limitSwitchWorking = false; //false to indicate that the limit switch isn't functioning
    } else {
        climbData.limitSwitchWorking = true;
    }
}

//checks to see if the encoder is reading zero because if it is that means the encoder was most likley unplugged and the current values are wrong and we don't want to run any motors
bool Climb::encoderPluggedIn(){
    if (climbArmsAbs.GetOutput() > 0.01){
        return true; //returns true to indicate that the encoder is functioning
    } else {
        return false;
    }
}

//checks if the encoder is reading values in the incorrect range, and if the values aren't reasonable, then the motors stop running in the bench test function
bool Climb::encoderInRange(){
    if (climbArmsAbs.GetOutput() < .727 - .05){
        return false;
    } else if (climbArmsAbs.GetOutput() > 0.811 + .05){
        return false;
    } else {
        return true;
    }
}