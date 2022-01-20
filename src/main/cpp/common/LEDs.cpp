#include "common/LEDs.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotData.h"
#include "Robot.h"
#include "controller/Controller.h"

void LEDs::RobotInit(){

}

void LEDs::TeleopInit(){

}

void LEDs::RobotPeriodic(const RobotData &robotData){
    //when B is pressed, the colorCode variable changes, and the new value is written to the arduino.
    if (robotData.controllerData.sBBtnToggled){
        colorCode++;
    }

    //the code writes the value of colorCode to device address 1 (the arduino), which then color codes the LEDs based upon the value
    //if the write is successful, success = true
    success = !arduino.Write(1, colorCode);

    //this prints true if the write was successful, and false if it aborted
    frc::SmartDashboard::PutBoolean("success", success);

    //the number corresponding to the color
    //1 = red, 2 = green, 3 = blue, 4 = yellow, 5 = white
    frc::SmartDashboard::PutNumber("colorCode: ", colorCode);
}