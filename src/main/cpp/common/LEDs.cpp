#include <frc/smartdashboard/SmartDashboard.h>
#include "common/LEDs.h"
#include "Robot.h"

void LEDs::RobotInit(){

}

void LEDs::TeleopInit(){

}

void LEDs::RobotPeriodic(const RobotData &robotData){
    //this makes the robot LEDs be different colors depending on the mode

    //writes the value of colorCode to device address 1 (the left arduino), which then color codes the LEDs based upon the value
    //if the write is successful, success = true
    if (robotData.shooterData.readyShoot){
        leftColorCode = 5;
        success = !leftArduino.Write(1, leftColorCode);
    } else if (robotData.controlData.mode == mode_teleop_manual){
        leftColorCode = 4; //teleop manual mode
        success = !leftArduino.Write(1, leftColorCode);
    } else if (robotData.controlData.mode == mode_climb_sa){
        leftColorCode = 3; //climb semiauto mode
        success = !leftArduino.Write(1, leftColorCode);
    } else if (robotData.controlData.mode == mode_climb_manual){
        leftColorCode = 2; //climb manual mode
        success = !leftArduino.Write(1, leftColorCode);
    } else if (robotData.controlData.mode == mode_teleop_sa){
        leftColorCode = 1; //teleop semiauto mode
        success = !leftArduino.Write(1, leftColorCode);
    }

    //Writes to the right arduino/LED strip; should display indexer contents
    if (robotData.indexerData.indexerContents.at(0) == CargoColor::cargo_Red && robotData.indexerData.indexerContents.at(1) == CargoColor::cargo_Blue){
        rightColorCode = 0;
        success = !rightArduino.Write(2, rightColorCode);
    } else if (robotData.indexerData.indexerContents.at(0) == CargoColor::cargo_Blue && robotData.indexerData.indexerContents.at(1) == CargoColor::cargo_Red){
        rightColorCode = 1;
        success = !rightArduino.Write(2, rightColorCode);
    } else if (robotData.indexerData.indexerContents.at(0) == CargoColor::cargo_Red && robotData.indexerData.indexerContents.at(1) == CargoColor::cargo_Unknown){
        rightColorCode = 2;
        success = !rightArduino.Write(2, rightColorCode);
    } else if (robotData.indexerData.indexerContents.at(0) == CargoColor::cargo_Blue && robotData.indexerData.indexerContents.at(1) == CargoColor::cargo_Unknown){
        rightColorCode = 3;
        success = !rightArduino.Write(2, rightColorCode);
    } else if (robotData.indexerData.indexerContents.at(0) == CargoColor::cargo_Unknown && robotData.indexerData.indexerContents.at(1) == CargoColor::cargo_Red){
        rightColorCode = 4;
        success = !rightArduino.Write(2, rightColorCode);
    } else if (robotData.indexerData.indexerContents.at(0) == CargoColor::cargo_Unknown && robotData.indexerData.indexerContents.at(1) == CargoColor::cargo_Blue){
        rightColorCode = 5;
        success = !rightArduino.Write(2, rightColorCode);
    } else if (robotData.indexerData.indexerContents.at(0) == CargoColor::cargo_Unknown && robotData.indexerData.indexerContents.at(1) == CargoColor::cargo_Unknown){
        rightColorCode = 6;
        success = !rightArduino.Write(2, rightColorCode);
    }

    //prints true if the write was successful, and false if it aborted
    frc::SmartDashboard::PutBoolean("Arduino write successful?", success);
}