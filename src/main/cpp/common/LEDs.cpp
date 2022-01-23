#include <frc/smartdashboard/SmartDashboard.h>
#include "common/LEDs.h"
#include "Robot.h"
#include <frc/util/Color.h>

void LEDs::RobotInit(){

}

void LEDs::TeleopInit(){

}

void LEDs::RobotPeriodic(const RobotData &robotData){

    /* worth noting that nothing in this file has been tested on the robot */

    //this makes the robot LEDs be different colors depending on the mode
    if (robotData.controlData.manualMode && !robotData.controlData.climbMode){
        colorCode = 4; //teleop manual mode
        //writes the value of colorCode to device address 1 (the arduino), which then color codes the LEDs based upon the value
        //if the write is successful, success = true
        success = !arduino.Write(1, colorCode);
    } else if (!robotData.controlData.manualMode && robotData.controlData.climbMode){
        colorCode = 3; //climb semiauto mode
        success = !arduino.Write(1, colorCode);
    } else if (robotData.controlData.manualMode && robotData.controlData.climbMode){
        colorCode = 2; //climb manual mode
        success = !arduino.Write(1, colorCode);
    } else if (!robotData.controlData.manualMode && !robotData.controlData.climbMode){
        colorCode = 1; //teleop semiauto mode
        success = !arduino.Write(1, colorCode);
    }

    // // indexer arduino LED color sensor stuff (1st slot (closest to intake))
    // if (robotData.colorSensorData.currentColor == frc::Color::kBlue){ //should work once branches are merged (theoretically)
    //     indexerColorCodeB = indexerColorCodeA;
    //     indexerColorCodeA = 2;
    //     success = !indexerArduinoA.Write(2, indexerColorCodeA); //blue
    // } else if (robotData.colorSensorData.currentColor == frc::Color::kRed){
    //     indexerColorCodeB = indexerColorCodeA;
    //     indexerColorCodeA = 1;
    //     success = !indexerArduinoA.Write(2, indexerColorCodeA); //red
    // } else {
    //     indexerColorCodeB = indexerColorCodeA;
    //     indexerColorCodeA = 4;
    //     success = !indexerArduinoA.Write(2, indexerColorCodeA); //black (aka off)
    // }

    // // indexer arduino LED color sensor stuff (2nd slot (closest to shooter))
    // if (indexerColorCodeB == 2){
    //     success = !indexerArduinoB.Write(3, indexerColorCodeB); //blue
    // } else  if (indexerColorCodeB == 1){
    //     success = !indexerArduinoB.Write(3, indexerColorCodeB); //red
    // } else {
    //     success = !indexerArduinoB.Write(3, indexerColorCodeB); //black (aka off)
    // }

    // // if the shooter is ready to shoot, then the LEDs for it turn white; if not, the LEDs turn black (off)
    // if (robotData.shooterData.readyShoot){ //should work once branches are merged
    //     shooterColorCode = 3;
    //     success = !shooterArduino.Write(4, shooterColorCode); //white
    // } else {
    //     shooterColorCode = 4;
    //     success = !shooterArduino.Write(4, shooterColorCode); //black (aka off)
    // }

    //prints true if the write was successful, and false if it aborted
    frc::SmartDashboard::PutBoolean("success", success);
}