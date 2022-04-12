#include <frc/smartdashboard/SmartDashboard.h>
// #include "common/Arduino.h"
#include "Robot.h"
#include <time.h>

void Arduino::RobotInit()
{
    try{
        arduino = new frc::SerialPort(9600, frc::SerialPort::Port::kUSB);    }
    catch (...)
    {
        ArduinoWorks = false;
    }
    frc::SmartDashboard::PutNumber("LEDNUMBER", 0);

}

void Arduino::RobotPeriodic(const RobotData &robotData, ArduinoData &arduinoData){
    frc::SmartDashboard::PutBoolean("ARDUINOWORKS", ArduinoWorks);
    if (frc::DriverStation::IsEnabled() && ArduinoWorks) {
        //if (ArduinoWorks){
        realBallCount = robotData.jetsonData.ballCount;
        // realBallCount = frc::SmartDashboard::GetNumber("LEDNUMBER", 0);
        if (realBallCount > ballCount)
        {
            ballCount = realBallCount;
            changeTimer = 0;
        }
        else if (realBallCount < ballCount)
        {
            changeTimer += 1;
        } else{
            changeTo = 0;
            changeTimer = 0;
        }
        if (changeTimer >= 10)
        {
            ballCount = realBallCount;
            changeTimer = 0;
        }
        // realBallCount = frc::SmartDashboard::GetNumber("LEDNUMBER", 0);
        // this makes the robot LEDs be different colors depending on the mode
        // writes the value of colorCode to device address 1 (the left arduino), which then color codes the LEDs based upon the value
        if (robotData.controlData.mode == Mode::mode_teleop_manual){
            mode = 4; //teleop manual mode
        } else if (robotData.controlData.mode == Mode::mode_climb_sa){
            mode = 3; //climb semiauto mode
        } else if (robotData.controlData.mode == Mode::mode_climb_manual){
            mode = 2; //climb manual mode
        } else if (robotData.controlData.mode == Mode::mode_teleop_sa){
            mode = 1; //teleop semiauto mode
        }
        //colorCode = 0; //uncomment for reveal video
        colorCode = (ballCount * 10) + mode + 10;
        // char value[1] = {(char)(colorCode)};
        char value[1] = {(char)(colorCode)};

        try {
            if (lastColorCode != colorCode){
                arduino->Write(value, 1);
            }
        } catch (...)
        {
            failedTransfers += 1;
        }
        
        

        lastColorCode = colorCode;
        lastBallCount = ballCount;

        // frc::SmartDashboard::PutNumber("Color", (int)colors[0]);
        // frc::SmartDashboard::PutNumber("Arduino colorCode for LEDs", colorCode);
        arduinoData.ColorData = (int)colors[0];
        // frc::SmartDashboard::PutBoolean("isDisabled", false);

        try{
            if (arduino->GetBytesReceived() >= 1){
                arduino->Read(colors,1);
                arduino->Reset();
            }
        } catch (...)
        {
            failedTransfers += 1;
        }
        

    }

    
}

void Arduino::DisabledPeriodic(){
    if (ArduinoWorks){
    colorCode = 0;
    char value[1] = {(char)colorCode};

    try {
        if (lastColorCode != colorCode){
            arduino->Write(value, 1);
        }
    } catch (...){
        failedTransfers += 1;
    }

    lastColorCode = colorCode;
    }
}