#pragma once

#include <frc/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

struct JetsonData
{
    int ballCount;
};

class Jetson
{
public:
    void RobotInit();
    void RobotPeriodic(JetsonData &jetsonData);
private:
    int currentAlliance;
};