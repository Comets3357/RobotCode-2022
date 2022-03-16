#include <frc/smartdashboard/SmartDashboard.h>
#include "common/LEDs.h"
#include "Robot.h"

void LEDs::RobotPeriodic(const RobotData &robotData){
    //this makes the robot LEDs be different colors depending on the mode
    //writes the value of colorCode to device address 1 (the left arduino), which then color codes the LEDs based upon the value
    //if the write is successful, success = true
    if (robotData.shooterData.readyShoot){
        colorCode = 5; //ready to shoot
        success = !arduino.Write((uint8_t *) colorCode, sizeof(uint8_t));
    } else if (robotData.controlData.mode == Mode::mode_teleop_manual){
        colorCode = 4; //teleop manual mode
        success = !arduino.Write((uint8_t *) colorCode, sizeof(uint8_t));
    } else if (robotData.controlData.mode == Mode::mode_climb_sa){
        colorCode = 3; //climb semiauto mode
        success = !arduino.Write((uint8_t *) colorCode, sizeof(uint8_t));
    } else if (robotData.controlData.mode == Mode::mode_climb_manual){
        colorCode = 2; //climb manual mode
        success = !arduino.Write((uint8_t *) colorCode, sizeof(uint8_t));
    } else if (robotData.controlData.mode == Mode::mode_teleop_sa){
        colorCode = 1; //teleop semiauto mode
        success = !arduino.Write((uint8_t *) colorCode, sizeof(uint8_t));
    }

    //prints the corresponding number of the color for the LEDs
    frc::SmartDashboard::PutNumber("ColorCode for LEDs", colorCode);

    //prints true if the write was successful, and false if it aborted
    frc::SmartDashboard::PutBoolean("Arduino write successful?", success);
}