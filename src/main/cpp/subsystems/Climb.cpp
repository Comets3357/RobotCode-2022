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
    //climbElevator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,0);
    climbElevator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,140);
    //climbArms.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,0);
    climbArms.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,250);

    //motor idlemode
    climbElevator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    climbArms.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    //resets encoders
    climbElevatorEncoder.SetPosition(0);
    climbArmsEncoder.SetPosition(0);

    //PIDS
    climbElevator_pidController.SetP(0.18, 0); //off bar
    climbElevator_pidController.SetOutputRange(-0.5, 0.5, 0);
    climbElevator_pidController.SetP(1, 1); //on bar temp
    climbElevator_pidController.SetOutputRange(-0.5, 0.5, 1);

    climbArms_pidController.SetP(0.25, 0);
    climbArms_pidController.SetOutputRange(-0.8, 0.8, 0);
    climbArms_pidController.SetP(0.5, 1);
    climbArms_pidController.SetOutputRange(-0.8, 0.8, 1);
    
}

//arms 0.25
//arms 0.5

void Climb::RobotPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    
    updateData(robotData, climbData);
    //manual(robotData, climbData);
    //semiAuto(robotData, climbData);

    //RunElevatorToPos(10,0,0);
    if (robotData.controlData.mode == mode_climb_sa || robotData.controlData.mode == mode_climb_manual)
    {
    //if (stage == 0) RunElevatorToPos(0.1, 1, 0);

    //climbElevator_pidController.SetReference(50, rev::ControlType::kPosition, 0);
        if (robotData.controlData.mode == mode_climb_manual)
        { //updates whether or not the robot is in manual or semiAuto mode
            manual(robotData, climbData);
        }
        else {
            semiAuto(robotData, climbData);
        }
    } 
    else
    {
        climbElevator.Set(0);
        climbArms.Set(0);
    }

    if (climbElevatorEncoder.GetPosition() >= 145 && climbElevator.Get() > 0)
    {
        climbElevator.Set(0);
    }

    //runs if the climb zeros
    if (robotData.controlData.climbZeroing)
    {
        climbData.zeroing = !climbData.zeroing;
    }

    //runs after the zeroing button is pressed
    if (climbData.zeroing)
    {
        //sets the elevator going down really slow
        climbElevator.Set(-0.1);
        //stops the motor and ends the zeroing when the limit switch changes
        if (!elevatorLimit.Get())
        {
            climbElevator.Set(0);
            climbData.zeroing = false;
            //resets encoder
            climbElevatorEncoder.SetPosition(0);
        }

    }
}

void Climb::manual(const RobotData &robotData, ClimbData &climbData)
{
    //manualy sets the elevator with limit. I use the limit switch as the bottom limit
    if (climbElevatorEncoder.GetPosition() >= 144 && robotData.controllerData.sLYStick > 0)
    {
        climbElevator.Set(0); //control elevator with left stick
    }
    else 
    {
        //deadzone
        if (robotData.controllerData.sLYStick < -0.08 || robotData.controllerData.sLYStick > 0.08)
        {
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
        climbArms.Set(0); //control arms with right stick
    }
    else 
    {
        //dead zone
        if (robotData.controllerData.sRYStick < -0.08 || robotData.controllerData.sRYStick > 0.08)
        {
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
    //elevator up = negative
    //elevator down = positive
    //arms away is positive
    //arms otherway is negative

    //elevator position 1000 is extended to bar height
    //arms position 0 is straight up

    climbInit(robotData, climbData);
    //cancelSequence(robotData, climbData);
    runSequence(robotData, climbData);
    
}
void Climb::climbInit(const RobotData &robotData, ClimbData &climbData)
{
    //if the climbInit button is pressed then the climb will go up if the climb is down and go down if it is already up. (toggle)
    if (robotData.controlData.saclimbInit && !climbInitiating)
    {
        //sets the climbUp to the opposite direction
        climbUp = !climbUp;
        //sets the climbInitiating variable to true to 
        climbInitiating = true;
    }

    //runs climb up
    if (climbInitiating && climbUp)
    {
        RunElevatorToPos(140,0,0);
    //runs climb down
    }
    else if (climbInitiating && !climbUp)
    { 
        RunElevatorToPos(0,0,0);
    }
}

//cancels the climb sequence
void Climb::cancelSequence(const RobotData &robotData, ClimbData &climbData)
{
    if (robotData.controlData.sacancelSequence)
    {
        executeSequence = false; //press a button, semiAuto code stops
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    }
}

//runs climb sequence
void Climb::runSequence(const RobotData &robotData, ClimbData &climbData)
{

    //sets variables to go up to 3rd bar
    if (robotData.controlData.saclimbHeightSequence)
    {//3rd bar
        stage = 0;
        executeSequence = true; //press right center button, semiAuto code runs
        climbData.climbing = true;
        targetBar = 3; //reaches to bar 3
        climbData.bar = 2;
    }
    //sets variables to go up to 4th bar
    else if (robotData.controlData.saclimbTraversalSequence)
    {//4th bar
        stage = 0;
        executeSequence = true; //press left center button, semiAuto code runs
        climbData.climbing = true;
        targetBar = 4; //reaches to bar 4
        climbData.bar = 2;
    }

    //starts going up to bar
    if (executeSequence && climbData.bar < 4)
    { //checks if you want to run the sequence, and also if you're already at bar 4, then you can't run it
        if (stage == 0) RunArmsAndElevatorToPos(0,1,0,0,1); //Elevator goes down to latch on 2nd/3rd bar
        else if (stage == 1) zeroElevator(0.5,1);
        else if (stage == 2) RunArmsToPos(40,1,0); //Elevator goes up to latch the arms onto the bar with the elevator a little above
        else if (stage == 3) RunElevatorToPos(30,1,0); //Outer Arms pivot the robot so the elevator is facing the next bar
        else if (stage == 4) RunArmsAndElevatorToPos(100,0,200,1,1);
        else if (stage == 5) RunElevatorToPos(145,1,1); //Elevator goes up to reach the next bar
        else if (stage == 6) RunArmsToPos(140,1,1); //Outer arms pivot back to latch the elevator onto the 3rd bar
        else if (stage == 7) RunElevatorToPos(90,1,1); //Elevator moves down to lift robot to the next bar. Outer arms move a little bit
        else if (stage == 8)
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
    frc::SmartDashboard::PutNumber("encoder value", climbElevatorEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("limitClimb", elevatorLimit.Get());
    frc::SmartDashboard::PutNumber("climb value", climbElevator.Get());
    frc::SmartDashboard::PutNumber("climb stage", angularRate);
    frc::SmartDashboard::PutBoolean("climbInitiationg", climbInitiating);
    frc::SmartDashboard::PutBoolean("executeSequence", executeSequence);
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
        if (abs(angularRate) < 45)
        {
            climbElevator_pidController.SetReference(position, rev::ControlType::kPosition, onBar);
        }
        else
        {
            climbElevator.Set(0);
            
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

void Climb::zeroElevator(float power, int stageAdd)
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