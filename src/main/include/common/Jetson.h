#pragma once

#include <frc/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

class Jetson
{
public:
    void RobotInit();
    void RobotPeriodic();
private:
    int currentAlliance;
    frc::DriverStation driverStation;
};