#include <frc/smartdashboard/SmartDashboard.h>
#include "common/LEDs.h"
#include "Robot.h"

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
    //Basically just takes the color value of the ball and tells the LEDs what color to display
    //Also since the enum depends on the alliance you're on, it doubles the amount of code :(
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
        if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Alliance && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Opponent){
            rightColorCode = 0; //value corresponding to the alliance/opponent combination on the LED arduino side
            success = !rightArduino.Write(2, rightColorCode); //writes the value to the indexer arduino, which will display the color combination on the LEDs
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Opponent && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Alliance){
            rightColorCode = 1;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Alliance && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Unassigned){
            rightColorCode = 2;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Opponent && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Unassigned){
            rightColorCode = 3;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Unassigned && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Alliance){
            rightColorCode = 4;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Unassigned && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Opponent){
            rightColorCode = 5;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Unassigned && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Unassigned){
            rightColorCode = 6;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Alliance && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Alliance){
            rightColorCode = 7;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Opponent && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Opponent){
            rightColorCode = 8;
            success = !rightArduino.Write(2, rightColorCode);
        }
    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Opponent && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Alliance){
            rightColorCode = 0;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Alliance && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Opponent){
            rightColorCode = 1;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Opponent && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Unassigned){
            rightColorCode = 2;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Alliance && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Unassigned){
            rightColorCode = 3;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Unassigned && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Opponent){
            rightColorCode = 4;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Unassigned && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Alliance){
            rightColorCode = 5;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Unassigned && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Unassigned){
            rightColorCode = 6;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Opponent && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Opponent){
            rightColorCode = 7;
            success = !rightArduino.Write(2, rightColorCode);
        } else if (robotData.indexerData.indexerContents.at(0) == Cargo::cargo_Alliance && robotData.indexerData.indexerContents.at(1) == Cargo::cargo_Alliance){
            rightColorCode = 8;
            success = !rightArduino.Write(2, rightColorCode);
        }
    }

    //prints true if the write was successful, and false if it aborted
    frc::SmartDashboard::PutBoolean("Arduino write successful?", success);
}