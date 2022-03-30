#pragma once
#include <frc/SerialPort.h>

struct RobotData;

struct ArduinoData {
    int ColorData = 0;
};

class Arduino {
    public:
        void RobotPeriodic(const RobotData &robotData, ArduinoData &arduinoData);
        void DisabledPeriodic();

        int failedTransfers = 0;

    private:
        bool ArduinoWorks = true;
        // in constructor port, deviceaddress
        frc::SerialPort arduino = frc::SerialPort(9600, frc::SerialPort::Port::kUSB);

        int colorCode = 6;
        int lastColorCode = 6;
        char colors[1];
};