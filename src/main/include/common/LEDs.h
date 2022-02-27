#pragma once

#include <frc/I2C.h>
#include "RobotData.h"

struct LEDsData {
    
};

class LEDs {
public:
    void RobotPeriodic(const RobotData &robotData);

private:
    //in constructor port, deviceaddress
    frc::I2C arduino = frc::I2C(frc::I2C::Port::kOnboard, 1); //initializes arduino to device 1
    bool success = false;
    int colorCode = 0;
};