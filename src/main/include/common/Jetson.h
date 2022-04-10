#pragma once

#include <frc/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

struct RobotData;

struct JetsonData
{
    double leftSkew;
    double rightSkew;
    double distanceFromBall;
    double angleOffBall;
    int ballCount;
};

class Jetson
{
public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, JetsonData &jetsonData);
private:
    int currentAlliance;

    double getSkew(double angle, double distance);

    int i = 0;
};