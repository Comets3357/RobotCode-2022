#pragma once

#include "RobotData.h"
#include <frc/SerialPort.h>

struct LEDsData {
    
};

class LEDs {
    public:
        void RobotPeriodic(const RobotData &robotData);
        void DisbledPeriodic();

    private:
        //in constructor port, deviceaddress
        frc::SerialPort arduino = frc::SerialPort(9600, frc::SerialPort::Port::kUSB);

        int colorCode = 6;
        int lastColorCode = 6;
};