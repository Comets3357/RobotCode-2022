#pragma once

#include <frc/SPI.h>
#include "RobotData.h"

struct LEDsData {
    
};

class LEDs {
public:
    void RobotPeriodic(const RobotData &robotData);

private:
    //in constructor port, deviceaddress
    frc::SPI arduino = frc::SPI(frc::SPI::Port::kOnboardCS0); //initializes arduino to device 1
    bool success = false;
    int colorCode = 6;
};