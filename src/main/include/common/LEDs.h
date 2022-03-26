#pragma once


#include <frc/SerialPort.h>

struct RobotData;

struct LEDsData {
    int ColorData = 0;
};

class LEDs {
    public:
        void RobotPeriodic(const RobotData &robotData, LEDsData &ledData);

    private:
        //in constructor port, deviceaddress
        frc::SerialPort arduino = frc::SerialPort(9600, frc::SerialPort::Port::kUSB);

        int colorCode = 6;
        int lastColorCode = 6;
        char colors[1];
};