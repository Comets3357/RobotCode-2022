#pragma once

#include <frc/I2C.h>
#include "RobotData.h"

struct LEDsData{
    
};

class LEDs{ // the bracket goes on this line if you put it on the next line you are objectively wrong
public:
    void RobotInit();
    void TeleopInit();
    void RobotPeriodic(const RobotData &robotData);

private:
    //in constructor port, deviceaddress
    frc::I2C arduino = frc::I2C(frc::I2C::Port::kOnboard, 1);
    bool success = false;
    int colorCode = 5;
};