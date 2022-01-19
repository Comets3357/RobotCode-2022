#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SPI.h>
#include "AHRS.h"

struct GyroData
{
    double rawYaw = 0;
    double rawPitch = 0;
    double rawRoll = 0;
};

class Gyro
{

public:
    void RobotInit();
    void AutonomousInit();
    void TeleopInit();
    void RobotPeriodic(GyroData &gyroData);

private:
    AHRS gyro{frc::SPI::Port::kMXP};

};