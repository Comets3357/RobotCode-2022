#include "common/LEDs.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <unistd.h>
#include "RobotData.h"
#include "Robot.h"

void LEDs::RobotInit(){

}

void LEDs::TeleopInit(){

}

void LEDs::RobotPeriodic(const RobotData &robotData){
    //tells the roborio to write to device address 1 and write a value of x to the device that is registered
    if (robotData.controllerData.sBBtnToggled){
        x++;
    }

    if (x==6){
        x = 1;
    }

    success = !arduino.Write(1, x);

    frc::SmartDashboard::PutBoolean("Print success", success); //this prints true if the write was successful, and false if it aborted
    frc::SmartDashboard::PutNumber("X: ", x);
}