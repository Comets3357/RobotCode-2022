#include <frc/smartdashboard/SmartDashboard.h>
#include "common/LEDs.h"
#include "Robot.h"
#include <time.h>

void LEDs::RobotPeriodic(const RobotData &robotData){
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
    //arduino.Write(value, 1);
    if (lastColorCode != colorCode)
    {
        arduino.Write(value, 1);
        frc::SmartDashboard::PutNumber("Idk", arduino.Write(value, 1));
    }
    lastColorCode = colorCode;


    //prints the corresponding number of the color for the LEDs
    frc::SmartDashboard::PutNumber("ColorCode for LEDs", colorCode);
    char colors[3];
    if (arduino.GetBytesReceived() >= 3)
    {
        arduino.Read(colors,3);
    }

    frc::SmartDashboard::PutRaw("colors", colors);
    
}

void LEDs::DisbledPeriodic()
{
    char value[1] = {'0'};
    //arduino.Write(value, 1);
    if (lastColorCode != 0)
    {
        arduino.Write(value, 1);
        frc::SmartDashboard::PutNumber("Idk", arduino.Write(value, 1));
    }
    lastColorCode = 0;
    colorCode = 0;
}