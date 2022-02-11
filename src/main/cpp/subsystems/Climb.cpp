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
    climbArms.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,0);
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
    climbArms_pidController.SetOutputRange(-0.5, 0.5, 0);
    climbArms_pidController.SetP(0.5, 1);
    climbArms_pidController.SetOutputRange(-0.5, 0.5, 1);
    
}

//arms 0.25
//arms 0.5

void Climb::RobotPeriodic(const RobotData &robotData, ClimbData &climbData){
    
    updateData(robotData, climbData);

    if (robotData.controlData.mode == mode_climb_sa || robotData.controlData.mode == mode_climb_manual){
    //if (stage == 0) RunElevatorToPos(0.1, 1, 0);

        if (robotData.controlData.mode == mode_climb_manual){ //updates whether or not the robot is in manual or semiAuto mode
            manual(robotData, climbData);
        } else {
            //semiAuto(robotData, climbData);
        }
    } else {
        climbElevator.Set(0);
        climbArms.Set(0);
    }
    // if (robotData.controlData.climbMode){

    //     if (robotData.controlData.manualMode){ //updates whether or not the robot is in manual or semiAuto mode
    //         manual(robotData, climbData);
    //     } else {
    //         //semiAuto(robotData, climbData);
    //     }
    // }

    //if the limit switch is read, then the power is set to 0 and the encoder is set to 0
    // if (elevatorLimit.Get() && climbElevator.Get() <= 0 && !climbData.zeroing){
    //     climbElevator.Set(0);
    //     climbElevatorEncoder.SetPosition(0);
    // }

    if (robotData.controlData.climbZeroing){
        climbData.zeroing = !climbData.zeroing;
    }

    if (climbData.zeroing)
    {
        climbElevator.Set(-0.03);
        if (!elevatorLimit.Get()){
            climbElevator.Set(0);
            climbData.zeroing = false;
            climbElevatorEncoder.SetPosition(0);
        }

    }
}

void Climb::manual(const RobotData &robotData, ClimbData &climbData){
    //manualy sets the elevator with limit
    if (climbElevatorEncoder.GetPosition() >= 144 && robotData.controllerData.sLYStick > 0){
        climbElevator.Set(0); //control elevator with left stick
    } else {
        if (robotData.controllerData.sLYStick < -0.08 || robotData.controllerData.sLYStick > 0.08){
        climbElevator.Set(robotData.controllerData.sLYStick*0.4); //control elevator with left stick); //sets the power to 0 so the elevator stops moving
        }
        else{
            climbElevator.Set(0);
        }
    }
    
    

    //manualy sets the arms with limit
    if ((climbArmsEncoder.GetPosition() <= 0 && robotData.controllerData.sRYStick < 0) || (climbArmsEncoder.GetPosition() >= 250 && robotData.controllerData.sRYStick > 0)){
        
        climbArms.Set(0); //control arms with right stick
    } else {
        if (robotData.controllerData.sRYStick < -0.08 || robotData.controllerData.sRYStick > 0.08){
            climbArms.Set(robotData.controllerData.sRYStick); //sets the power to 0 so the arms stop moving
        }
        else{
            climbArms.Set(0);
        }
    }
    

    //manualy sets the arms with limit
    // if (climbArmsEncoder.GetPosition() > -1000 && climbArmsEncoder.GetPosition() < 2000){
    //     climbArms.Set(robotData.controllerData.sRYStick); //control arms with right stick
    // } else {
    //     climbArms.Set(0); //sets the power to 0 so the arms stop moving
    // }
    //climbArms.Set(robotData.controlData.mArmPivot*.3);
}

void Climb::semiAuto(const RobotData &robotData, ClimbData &climbData){
    //elevator up = negative
    //elevator down = positive
    //arms away is positive
    //arms otherway is negative

    //elevator position 1000 is extended to bar height
    //arms position 0 is straight up

    climbInit(robotData, climbData);
    cancelSequence(robotData, climbData);
    runSequence(robotData, climbData);
    
}
void Climb::climbInit(const RobotData &robotData, ClimbData &climbData){
    //if the climbInit button is pressed then the climb will go up if the climb is down and go down if it is already up. (toggle)
    if (robotData.controlData.saclimbInit && !climbInitiating){
        //sets the climbUp to the opposite direction
        climbUp = !climbUp;
        //sets the climbInitiating variable to true to 
        climbInitiating = true;
    }

    //runs climb up
    if (climbInitiating && climbUp){
        RunElevatorToPos(140,0,0);
    //runs climb down
    } else if (climbInitiating && !climbUp){ 
        RunElevatorToPos(0,0,0);
    }
}
void Climb::cancelSequence(const RobotData &robotData, ClimbData &climbData){
    if (robotData.controlData.sacancelSequence){
        executeSequence = false; //press a button, semiAuto code stops
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    }
}
void Climb::runSequence(const RobotData &robotData, ClimbData &climbData){
    if (robotData.controlData.saclimbHeightSequence){//3rd bar
        stage = 0;
        executeSequence = true; //press right center button, semiAuto code runs
        climbData.climbing = true;
        targetBar = 3; //reaches to bar 3
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    } else if (robotData.controlData.saclimbTraversalSequence){//4th bar
        stage = 0;
        executeSequence = true; //press left center button, semiAuto code runs
        climbData.climbing = true;
        targetBar = 4; //reaches to bar 4
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    }

    if (executeSequence && climbData.bar == 3){ //checks if you want to run the sequence, and also if you're already at bar 4, then you can't run it
        if (stage == 0) RunArmsToPos(0,1,0); //resets the arms position to perpendicular in case they aren't
        else if (stage == 1) RunElevatorToPos(0,1,0); //Elevator goes down to latch on 2nd/3rd bar
        else if (stage == 2) RunArmsToPos(50,1,0); //Elevator goes up to latch the arms onto the bar with the elevator a little above
        else if (stage == 3) RunElevatorToPos(20,1,0); //Outer Arms pivot the robot so the elevator is facing the next bar
        else if (stage == 4) RunArmsAndElevatorToPos(140,0,200,1,1); //Elevator goes up to reach the next bar
        else if (stage == 5) RunArmsToPos(180,1,1); //Outer arms pivot back to latch the elevator onto the 3rd bar
        else if (stage == 6) RunElevatorToPos(100,1,1); //Elevator moves down to lift robot to the next bar. Outer arms move a little bit
        else if (stage == 7) RunArmsAndElevatorToPos(40,1,0,0,1); //Outer arms release from 2nd bar
        else if (stage == 8) RunElevatorToPos(0,1,1); //Elevators goes up a little to let the outer arms pivot to the other side of the bar
        else if (stage == 9) RunArmsToPos(50,0,1);
        else if (stage == 9) RunElevatorToPos(50,0,1); //Outer arms pivot the the other side of the bar
        else if (stage == 10){ //do it again if the bot isnt on the top bar
            stage = 0;
            executeSequence = false;
            climbArms.Set(0);

        }
    }
}

void Climb::DisabledInit(){
    //sets motors to 0 for cuz disabled
    climbElevator.Set(0);
    climbArms.Set(0);
}

// updates encoder and gyro values
void Climb::updateData(const RobotData &robotData, ClimbData &climbData){
    frc::SmartDashboard::PutNumber("encoder value", climbElevatorEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("limitClimb", elevatorLimit.Get());
    frc::SmartDashboard::PutNumber("climb value", climbElevator.Get());
}

void Climb::DisabledPeriodic(const RobotData &robotData, ClimbData &climbData)
{
    updateData(robotData, climbData);
}

//Runs the elevator to a specific location, specified in semiAuto
void Climb::RunElevatorToPos(int position, int stageAdd, int onBar){
    // //Checks which direction the motor will be spinning. This uses the variable running to only set the direction 1 time.
    // if (!elevatorRunning && climbElevatorEncoder.GetPosition() > position){
    //     //sets the direction that the motor will be spinning based on where the ideal position is from the current position
    //     elevatorDirection = true;
    //     //sets running to true so direction is not set after this moment
    //     elevatorRunning = true;
    // } else {
    //     //sets the direction that the motor will be spinning based on where the ideal position is from the current position
    //     elevatorDirection = false;
    //     //sets running to true so direction is not set after this moment
    //     elevatorRunning = true;
    // }

    // //this sets the power of the motor based on the direction chosen
    // if (climbElevatorEncoder.GetPosition() > position && !elevatorDirection){
    //     //sets the power of the motor to a negative value based on the direction needed to get to the destination
    //     climbElevator.Set(-abs(power));
    // } else {
    //     //sets the power to 0 because the job is done
    //     climbElevator.Set(0);
    //     //sets the stage to the next stage so it can move to the next step
    //     stage+=stageAdd;
    //     //sets the climbInitiating to false incase this function is used for climb initiation
    //     climbInitiating = false;
    //     elevatorRunning = false;
    // }

    // if (climbElevatorEncoder.GetPosition() < position && elevatorDirection){
    //     //sets the power of the motor to a positive value based on the direction needed to get to the destination
    //     climbElevator.Set(abs(power));
    // } else {
    //     //sets the power to 0 because the job is done
    //     climbElevator.Set(0);
    //     //sets the stage to the next stage so it can move to the next step
    //     stage+=stageAdd;
    //     //sets the climbInitiating to false incase this function is used for climb initiation
    //     climbInitiating = false;
    //     elevatorRunning = false;
    // }

    if (climbElevatorEncoder.GetPosition() > position + 1 || climbElevatorEncoder.GetPosition() < position - 1){
        elevatorRunning = true;
        // if (//angular moment < 1){
        //     climbElevator_pidController.SetReference(position, rev::ControlType::kPosition, onBar);
        // } else {
        //     climbElevator.Set(0);
        // }
        elevatorRunning = true;
        climbElevator_pidController.SetReference(position, rev::ControlType::kPosition, onBar);
    } else {
        elevatorRunning = false;
        stage += stageAdd;
    }
}

//runs the arms to a specific location, specified in semiAuto
void Climb::RunArmsToPos(int position, int stageAdd, int onBar){
    // //Checks which direction the motor will be spinning. This uses the variable running to only set the direction 1 time.
    // if (!armsRunning && climbArmsEncoder.GetPosition() > position){
    //     //sets the direction that the motor will be spinning based on where the ideal position is from the current position
    //     armsDirection = true;
    //     //sets running to true so direction is not set after this moment
    //     armsRunning = true;
    // } else {
    //     //sets the direction that the motor will be spinning based on where the ideal position is from the current position
    //     armsDirection = false;
    //     //sets running to true so direction is not set after this moment
    //     armsRunning = true;
    //     armsRunning = false;
    // }

    // //this sets the power of the motor based on the direction chosen
    // if (climbArmsEncoder.GetPosition() > position && !armsDirection){
    //     //sets the power of the motor to a negative value based on the direction needed to get to the destination
    //     climbArms.Set(-abs(power));
    // } else {
    //     //sets the power to 0 because the job is done
    //     climbArms.Set(0);
    //     //sets the stage to the next stage so it can move to the next step
    //     stage+=stageAdd;
    //     //sets the climbInitiating to false incase this function is used for climb initiation
    //     climbInitiating = false;
    //     armsRunning = false;
    // }

    // if (climbArmsEncoder.GetPosition() < position && armsDirection){
    //     //sets the power of the motor to a positive value based on the direction needed to get to the destination
    //     climbArms.Set(abs(power));
    // } else {
    //     //sets the power to 0 because the job is done
    //     climbArms.Set(0);
    //     //sets the stage to the next stage so it can move to the next step
    //     stage+=stageAdd;
    //     //sets the climbInitiating to false incase this function is used for climb initiation
    //     climbInitiating = false;
    //     armsRunning = false;
    // }

    if (climbArmsEncoder.GetPosition() > position + 1 || climbArmsEncoder.GetPosition() < position - 1){
        armsRunning = true;
        climbArms_pidController.SetReference(position, rev::ControlType::kPosition, onBar);
    } else {
        armsRunning = false;
        stage += stageAdd;
    }
}

void Climb::RunArmsAndElevatorToPos(int elevatorPos, int elevatorBar, int armsPos, int armsBar, int stageAdd){
    RunElevatorToPos(elevatorPos, 0, elevatorBar);
    RunArmsToPos(armsPos, 0, armsBar);
    if (!elevatorRunning && !armsRunning){
        climbArms.Set(0);
        climbElevator.Set(0);
        stage+=stageAdd;
    }

}


void Climb::zeroElevator(float power, int stageAdd){
    if (!elevatorLimit.Get()){
        elevatorRunning = true;
        climbElevator.Set(abs(power));
    } else {
        climbElevator.Set(0);
        elevatorRunning = false;
        stage += stageAdd;
    }
}