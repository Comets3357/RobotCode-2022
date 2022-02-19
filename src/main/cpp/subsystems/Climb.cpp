#include "subsystems/Climb.h"
#include "RobotData.h"

void Climb::RobotInit()
{
    //do other init stuff (probably more)
    climbArms.RestoreFactoryDefaults();
    climbElevator.RestoreFactoryDefaults();

    //do other init stuff (probably more)
    climbArms.RestoreFactoryDefaults();
    climbElevator.RestoreFactoryDefaults();

    //sets the inversion of the motors
    climbArms.SetInverted(true);
    climbElevator.SetInverted(true);
    climbArms.SetSmartCurrentLimit(45);
    climbElevator.SetSmartCurrentLimit(80);
    climbElevator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,140);
    climbArms.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,250);

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
    
}

//arms 0.25
//arms 0.5

void Climb::RobotPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    //updates
    updateData(robotData, climbData);

    //checks if the robot is in climb mode
    if (robotData.controlData.mode == mode_climb_sa || robotData.controlData.mode == mode_climb_manual)
    {
        //chacks if the robot is in manual
        if (robotData.controlData.mode == mode_climb_manual)
        { //updates whether or not the robot is in manual or semiAuto mode
            manual(robotData, climbData);
        }
        else
        {
            semiAuto(robotData, climbData);
        }
    } 
    else
    {
        //sets powers to 0 if the mode is changed out of climb mode
        climbElevator.Set(0);
        climbArms.Set(0);
    }

    //softlimit max for elevator
    if (climbElevatorEncoder.GetPosition() >= 145 && climbElevator.Get() > 0)
    {
        //sets the elevator power to 0 if it is above the max
        climbElevator.Set(0);
    }

    //runs if the climb zeros
    if (robotData.controlData.climbZeroing)
    {
        //toggles zeroing when the button is pressed so it can be stopped
        climbData.zeroing = !climbData.zeroing;
    }

    //runs after the zeroing button is pressed
    if (climbData.zeroing)
    {
        if (climbArmsAbs.GetOutput() > 0.03) {
            //sets the elevator going down really slow
            climbElevator.Set(-0.1);
            climbArms.Set(-0.2);
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

//manual
void Climb::manual(const RobotData &robotData, ClimbData &climbData)
{
    //manualy sets the elevator with limit. I use the limit switch as the bottom limit
    if (climbElevatorEncoder.GetPosition() >= 144 && robotData.controllerData.sLYStick > 0)
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
            climbElevator.Set(robotData.controllerData.sLYStick*0.4); //control elevator with left stick); //sets the power to 0 so the elevator stops moving
        }
        else
        {
            //sets power to 0 when nothing is supposed to happen
            climbElevator.Set(0);
        }
    }
    
    

    //manualy sets the arms with limit. The bottom limit is gon because an absolute encode will be there eventually
    if ((climbArmsEncoder.GetPosition() >= 250 && robotData.controllerData.sRYStick > 0))
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
            climbArms.Set(robotData.controllerData.sRYStick); //sets the power to 0 so the arms stop moving
        }
        else
        {
            //sets power to 0 when nothing is supposed to happen
            climbArms.Set(0);
        }
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
    if (robotData.controlData.saclimbInit && !climbInitiating) //if the climbInit button is pressed then the climb will go up if the climb is down and go down if it is already up. (toggle)
    {
        climbUp = !climbUp; //sets the climbUp to the opposite direction
        climbInitiating = true; //sets the climbInitiating variable to true to 
    }

    if (climbInitiating && climbUp)
    {
        RunElevatorToPos(140,0,0); //runs the climb up when button is pressed
    }
    else if (climbInitiating && !climbUp)
    { 
        RunElevatorToPos(0,0,0); //runs the climb back down if you press it again
    }
}

void Climb::cancelSequence(const RobotData &robotData, ClimbData &climbData) //cancels the climb sequence
{

    if (robotData.controlData.sapauseSequence) //checks for the pause button to be pressed
    {
        executeSequence = false; //press a button, semiAuto code stops
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    }

    if (robotData.controlData.sacancelSequence)
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
    if (robotData.controlData.saclimbHeightSequence)
    {//3rd bar
        //stage = 0;//for testing
        executeSequence = true; //press right center button, semiAuto code runs
        climbData.climbing = true;
        targetBar = 3; //reaches to bar 3
        ChangeElevatorSpeed(elevatorSpeed, 0);
    }
    //sets variables to go up to 4th bar
    else if (robotData.controlData.saclimbTraversalSequence)
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
        if (stage == 0) RunArmsAndElevatorToPos(0,1,0,0,1); //Elevator goes down to latch on 2nd/3rd bar
        else if (stage == 1) ZeroElevator(0.5,1);
        else if (stage == 2) RunArmsToPos(70,1,0); //Elevator goes up to latch the arms onto the bar with the elevator a little above
        else if (stage == 3) RunElevatorToPos(30,1,0); //Outer Arms pivot the robot so the elevator is facing the next bar
        else if (stage == 4) RunArmsAndElevatorToPos(100,0,220,1,1);
        else if (stage == 5) WaitUntilGyro(-1, -45, 1);
        else if (stage == 6) RunElevatorToPos(144.5,1,1);
        //else if (stage == 8) RunArmsToPos(200,1,1); //Elevator goes up to reach the next bar
        else if (stage == 7) RunArmsToPos(135,1,1);
        // else if (stage == 10) CheckGyroPosition(1, -50, -2, 1);
        else if (stage == 8) ChangeElevatorSpeed(0.5, 1); //Outer arms pivot back to latch the elevator onto the 3rd bar
        else if (stage == 9) RunElevatorToPos(100,1,1); //Elevator moves down to lift robot to the next bar. Outer arms move a little bit
        else if (stage == 10) ChangeElevatorSpeed(elevatorSpeed, 1);
        else if (stage == 11)
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

void Climb::CheckGyroPosition(int cmp, float gyroValue, int failAdd, int successAdd)
{
    if (cmp == 1)
    {
        if (gyroValue < angle)
        {
            stage += successAdd;
        }
        else 
        {
            stage += failAdd;
        }
    }
    else if (cmp == -1)
    {
        if (gyroValue > angle)
        {
            stage += successAdd;
        }
        else 
        {
            stage += failAdd;
        }
    }
    else if (cmp == 0){
        if (gyroValue == angle)
        {
            stage += successAdd;
        }
        else 
        {
            stage += failAdd;
        }

    }
}

void Climb::WaitUntilGyro(int cmp, float gyroValue, int stageAdd)
{
    if (cmp == 1)
    {
        if (gyroValue < angle)
        {
            stage += stageAdd;
        }
    }
    else if (cmp == -1)
    {
        if (gyroValue > angle)
        {
            stage += stageAdd;
        }
    }
    else if (cmp == 0){
        if (gyroValue == angle)
        {
            stage += stageAdd;
        }

    }
}

void Climb::ChangeElevatorSpeed(float speed, int stageAdd)
{
    climbElevator_pidController.SetOutputRange(-speed, speed, 0);
    climbElevator_pidController.SetOutputRange(-speed, speed, 1);
    stage += stageAdd;
}

//sets powers to 0 when disabled
void Climb::DisabledInit()
{
    //sets motors to 0 for cuz disabled
    climbElevator.Set(0);
    climbArms.Set(0);
}

// updates encoder and gyro values
void Climb::updateData(const RobotData &robotData, ClimbData &climbData)
{
    angularRate = robotData.gyroData.angularMomentum;
    angle = robotData.gyroData.rawRoll;
    
    frc::SmartDashboard::PutNumber("encoder value", climbElevatorEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("limitClimb", elevatorLimit.Get());
    frc::SmartDashboard::PutNumber("climb value", climbElevator.Get());
    frc::SmartDashboard::PutNumber("climb stage", angularRate);
    frc::SmartDashboard::PutBoolean("climbInitiationg", climbInitiating);
    frc::SmartDashboard::PutBoolean("executeSequence", executeSequence);
    frc::SmartDashboard::PutNumber("armEncoder", climbArmsEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("armsAbs", climbArmsAbs.GetOutput());
}

//runs update when disabled
void Climb::DisabledPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    updateData(robotData, climbData);
}

//Runs the elevator to a specific location, specified in semiAuto
void Climb::RunElevatorToPos(int position, int stageAdd, int onBar)
{
    if (climbElevatorEncoder.GetPosition() > position + 1 || climbElevatorEncoder.GetPosition() < position - 1)
    {
        elevatorRunning = true;
        if (onBar){
            if (abs(angularRate) < 40)
            {
                climbElevator_pidController.SetReference(position, rev::ControlType::kPosition, onBar);
            }
            else
            {
                climbElevator.Set(0);
                
            }
        } else{
            climbElevator_pidController.SetReference(position, rev::ControlType::kPosition, onBar);
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

    if (climbArmsEncoder.GetPosition() > position + 1 || climbArmsEncoder.GetPosition() < position - 1)
    {
        armsRunning = true;
        climbArms_pidController.SetReference(position, rev::ControlType::kPosition, onBar);
    }
    else
    {
        armsRunning = false;
        stage += stageAdd;
    }
}

void Climb::RunArmsAndElevatorToPos(int elevatorPos, int elevatorBar, int armsPos, int armsBar, int stageAdd)
{
    RunElevatorToPos(elevatorPos, 0, elevatorBar);
    RunArmsToPos(armsPos, 0, armsBar);
    if (!elevatorRunning && !armsRunning)
    {
        climbArms.Set(0);
        climbElevator.Set(0);
        stage+=stageAdd;
    }

}

void Climb::ZeroElevator(float power, int stageAdd)
{
    if (elevatorLimit.Get())
    {
        elevatorRunning = true;
        climbElevator.Set(-abs(power));
    }
    else 
    {
        climbElevator.Set(0);
        elevatorRunning = false;
        stage += stageAdd;
    }
}