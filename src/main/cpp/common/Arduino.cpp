#include <frc/smartdashboard/SmartDashboard.h>
// #include "common/Arduino.h"
#include "Robot.h"
#include <time.h>

void Arduino::RobotInit()
{
    try{
        arduino = new frc::SerialPort(9600, frc::SerialPort::Port::kUSB);
    }
    catch (...)
    {
        failedTransfers += 1;
    }


}

void Arduino::RobotPeriodic(const RobotData &robotData, ArduinoData &arduinoData){

    if (frc::DriverStation::IsEnabled() && ArduinoWorks) {

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

        //colorCode = 0; //uncomment for reveal video

        char value[1] = {(char)colorCode};

        try { throw exception();
            if (lastColorCode != colorCode){
                arduino->Write(value, 1);
            }
        } catch (...)
        {
            failedTransfers += 1;
        }
        
        

        lastColorCode = colorCode;

        // frc::SmartDashboard::PutNumber("Color", (int)colors[0]);
        // frc::SmartDashboard::PutNumber("Arduino colorCode for LEDs", colorCode);
        arduinoData.ColorData = (int)colors[0];
        // frc::SmartDashboard::PutBoolean("isDisabled", false);

        try{ throw exception();
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

    try { throw exception();
        if (lastColorCode != colorCode){
            arduino->Write(value, 1);
        }
    } catch (...)
    {
        failedTransfers += 1;
    }
        
    

    lastColorCode = colorCode;
    }
}