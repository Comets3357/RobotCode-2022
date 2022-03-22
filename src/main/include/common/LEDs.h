#pragma once

#include "RobotData.h"
#include <frc/SerialPort.h>

struct LEDsData {
    
};

class LEDs {
    public:
        void RobotPeriodic(const RobotData &robotData);

    private:
        //in constructor port, deviceaddress
        frc::SerialPort arduino = frc::SerialPort(frc::SerialPort::kOnboard);
        int colorCode = 6;
};