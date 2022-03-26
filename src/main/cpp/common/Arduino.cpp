#include <frc/smartdashboard/SmartDashboard.h>
#include "common/Arduino.h"
#include "Robot.h"
#include <time.h>

void Arduino::RobotPeriodic(const RobotData &robotData){
    // this makes the robot LEDs be different colors depending on the mode
    // writes the value of colorCode to device address 1 (the left arduino), which then color codes the LEDs based upon the value
    if (robotData.shooterData.readyShoot){
        colorCode = 5; //ready to shoot
    } else if (robotData.controlData.mode == Mode::mode_teleop_manual){
        colorCode = 4; //teleop manual mode
    } else if (robotData.controlData.mode == Mode::mode_climb_sa){
        colorCode = 3; //climb semiauto mode
    } else if (robotData.controlData.mode == Mode::mode_climb_manual){
        colorCode = 2; //climb manual mode
    } else if (robotData.controlData.mode == Mode::mode_teleop_sa){
        colorCode = 1; //teleop semiauto mode
    }

    char value[1] = {(char)colorCode};

    if (lastColorCode != colorCode){
        arduino.Write(value, 1);
    }

    lastColorCode = colorCode;

    frc::SmartDashboard::PutNumber("Arduino bytes received", arduino.GetBytesReceived());
    frc::SmartDashboard::PutNumber("Arduino colorCode for LEDs", colorCode);
    frc::SmartDashboard::PutBoolean("isDisabled", false);

    if (arduino.GetBytesReceived() >= 1){
        arduino.Read(colors,1);
        arduino.Reset();

    }
}

void Arduino::DisbledPeriodic(){
    char value[1] = {'0'};
    arduino.Write(value, 1);
    if (lastColorCode != 0)
    {
        arduino.Write(value, 1);
    }
    lastColorCode = 0;
    colorCode = 0;
    frc::SmartDashboard::PutBoolean("isDisabled", true);
}