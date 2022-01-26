#include "RobotData.h"

void Climb::RobotInit(){
    //sets the inversion of the motors
    climbArms.SetInverted(false);
    climbElevator.SetInverted(false);

    climbElevatorEncoder.SetPosition(0);
    climbArmsEncoder.SetPosition(0);

}

void Climb::RobotPeriodic(const RobotData &robotData, ClimbData &climbData){
    updateData(robotData, climbData);
    if (robotData.controlData.manualMode)
    {
        manual(robotData, climbData);
    }
    else
    {
        semiAuto(robotData, climbData);
    }

}

void Climb::manual(const RobotData &robotData, ClimbData &climbData){

    //manualy sets the elevator with limit
    if (climbElevatorEncoder.GetPosition() > -1000 && climbElevatorEncoder.GetPosition() < 2000){
        climbElevator.Set(robotData.controllerData.sLYStick);
    } else {
        climbElevator.Set(0);
    }

    //manualy sets the arms with limit
    if (climbArmsEncoder.GetPosition() > -1000 && climbArmsEncoder.GetPosition() < 2000){
        climbArms.Set(robotData.controllerData.sRYStick);
    } else {
        climbArms.Set(0);
    }

}

void Climb::semiAuto(const RobotData &robotData, ClimbData &climbData){

    //elevator up = negative
    //elevator down = positive
    //arms away is positive
    //arms otherway is negative

    //elevator position 1000 is extended to bar height
    //arms position 0 is straight up

    //press a, arms pivot away, press b, arms pivot in
    if (robotData.controllerData.sABtn && !executeSemiAuto) climbArms.Set(-0.5);
    else if (robotData.controllerData.sBBtn && !executeSemiAuto) climbArms.Set(0.5);
    else if (!executeSemiAuto) climbArms.Set(0); //sets the power to zero to make it stop moving

    //press x, elevator goes down, press y, elevator goes up
    if (robotData.controllerData.sXBtn && !executeSemiAuto) climbElevator.Set(0.5);
    else if (robotData.controllerData.sYBtn && !executeSemiAuto) climbElevator.Set(-0.5);
    else if (!executeSemiAuto) climbElevator.Set(0); //sets the power to zero to make it stop moving

    if (robotData.controllerData.sLStickBtn){
        executeSemiAuto = false; //press a button, semiAuto code stops
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    }

    if (robotData.controllerData.sRCenterBtn){
        executeSemiAuto = true; //press right center button, semiAuto code runs
        targetBar = 3; //reaches to bar 3
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    } else if (robotData.controllerData.sLCenterBtn){
        executeSemiAuto = true; //press left center button, semiAuto code runs
        targetBar = 4; //reaches to bar 4
        climbElevator.Set(0); //sets the power to zero to make it stop moving
        climbArms.Set(0); //sets the power to zero to make it stop moving
    }

    if (executeSemiAuto && bar < 4){ //checks if you want to run the sequence, and also if you're already at bar 4, then you can't run it
        if (stage == 0) RunArmsToPos(0,1); //resets the arms position to perpendicular in case they aren't
        else if (stage == 1) RunClimbToPos(800,1); //Elevator goes down to latch on 2nd bar
        else if (stage == 2) RunClimbToPos(1200,1); //Elevator goes up to latch the arms onto the bar with the elevator a little above
        else if (stage == 3) RunArmsToPos(-500,1); //Outer Arms pivot the robot so the elevator is facing the 3rd bar
        else if (stage == 4) RunClimbToPos(1750,1); //Elevator goes up to reach the 3rd bar
        else if (stage == 5) RunArmsToPos(-400,1); //Outer arms pivot back to latch the elevator onto the 3rd bar
        else if (stage == 6) RunClimbToPos(1000,1); //Elevator moves down to lift robot to the 3rd bar. Outer arms move a little bit
        else if (stage == 7) RunArmsToPos(-100,1); //Outer arms release from 2nd bar
        else if (stage == 8) RunClimbToPos(800,1); //Elevators goes up a little to let the outer arms pivot to the other side of the bar
        else if (stage == 9) RunArmsToPos(100,1); //Outer arms pivot the the other side of the elevator
        else if (stage == 10) RunClimbToPos(1100,1); //Elevator goes down a little so the hooks are above the bar
        else if (stage == 11) RunArmsToPos(0,1); //Outer arms go forward a little to align them with the bar
        else if (stage == 12 && bar < targetBar){ //do it again if the bot isnt on the top bar
            stage = 0; //sets the stage to 0 to go to the next bar
            bar++;
        } else {
            stage = 0;
            executeSemiAuto = false;
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
    
}

void Climb::RunClimbToPos(int position, float power){
    //Checks which direction the motor will be spinning. This uses the variable running to only set the direction 1 time.
    if (!elevatorRunning && climbElevatorEncoder.GetPosition() > position){
        //sets the direction that the motor will be spinning based on where the ideal position is from the current position
        elevatorDirection = true;
        //sets running to true so direction is not set after this moment
        elevatorRunning = true;
    } else {
        //sets the direction that the motor will be spinning based on where the ideal position is from the current position
        elevatorDirection = false;
        //sets running to true so direction is not set after this moment
        elevatorRunning = true;
    }

    //this sets the power of the motor based on the direction chosen
    if (climbElevatorEncoder.GetPosition() > position && !elevatorDirection){
        //sets the power of the motor to a negative value based on the direction needed to get to the destination
        climbElevator.Set(-abs(power));
    } else {
        //sets the power to 0 because the job is done
        climbElevator.Set(0);
        //sets the stage to the next stage so it can move to the next step
        stage++;
    }

    if (climbElevatorEncoder.GetPosition() < position && elevatorDirection){
        //sets the power of the motor to a positive value based on the direction needed to get to the destination
        climbElevator.Set(abs(power));
    } else {
        //sets the power to 0 because the job is done
        climbElevator.Set(0);
        //sets the stage to the next stage so it can move to the next step
        stage++;
    }
}

void Climb::RunArmsToPos(int position, float power){
    //Checks which direction the motor will be spinning. This uses the variable running to only set the direction 1 time.
    if (!armsRunning && climbArmsEncoder.GetPosition() > position){
        //sets the direction that the motor will be spinning based on where the ideal position is from the current position
        armsDirection = true;
        //sets running to true so direction is not set after this moment
        armsRunning = true;
    } else {
        //sets the direction that the motor will be spinning based on where the ideal position is from the current position
        armsDirection = false;
        //sets running to true so direction is not set after this moment
        armsRunning = true;
    }

    //this sets the power of the motor based on the direction chosen
    if (climbArmsEncoder.GetPosition() > position && !armsDirection){
        //sets the power of the motor to a negative value based on the direction needed to get to the destination
        climbArms.Set(-abs(power));
    } else {
        //sets the power to 0 because the job is done
        climbArms.Set(0);
        //sets the stage to the next stage so it can move to the next step
        stage++;
    }

    if (climbArmsEncoder.GetPosition() < position && armsDirection){
        //sets the power of the motor to a positive value based on the direction needed to get to the destination
        climbArms.Set(abs(power));
    } else {
        //sets the power to 0 because the job is done
        climbArms.Set(0);
        //sets the stage to the next stage so it can move to the next step
        stage++;
    }
}