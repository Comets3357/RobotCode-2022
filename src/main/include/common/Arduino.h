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
        void RobotInit();

        int failedTransfers = 0;

        int lastBallCount = 0;
        int ballCount = 0;
        int mode = 0;

    private:
        bool ArduinoWorks = true;
        // in constructor port, deviceaddress
        frc::SerialPort *arduino;
        //frc::SerialPort arduino = frc::SerialPort(9600, frc::SerialPort::Port::kUSB);

        int colorCode = 6;
        int lastColorCode = 6;
        char colors[1];
};