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
    frc::I2C leftArduino = frc::I2C(frc::I2C::Port::kOnboard, 1);
    frc::I2C rightArduino = frc::I2C(frc::I2C::Port::kOnboard, 2);
    bool success = false;
    int leftColorCode = 5;
    int rightColorCode = 6;
};