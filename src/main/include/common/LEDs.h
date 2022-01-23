#pragma once

#include <frc/I2C.h>
#include "RobotData.h"

struct LEDsData {
    
};

class LEDs {
public:
    void RobotInit();
    void TeleopInit();
    void RobotPeriodic(const RobotData &robotData);

private:
    //in constructor port, deviceaddress
    frc::I2C arduino = frc::I2C(frc::I2C::Port::kOnboard, 1);
    frc::I2C indexerArduinoA = frc::I2C(frc::I2C::Port::kOnboard, 2);
    frc::I2C indexerArduinoB = frc::I2C(frc::I2C::Port::kOnboard, 3);
    frc::I2C shooterArduino = frc::I2C(frc::I2C::Port::kOnboard, 4);
    bool success = false;
    int colorCode = 5;
    int indexerColorCodeA = 0;
    int indexerColorCodeB = 0;
    int shooterColorCode = 0;
};