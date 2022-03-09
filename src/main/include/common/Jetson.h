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
};

class Jetson
{
public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, JetsonData &jetsonData);
    double getDistanceFromBall();
    double getAngleOffBall();
    double getSkew(double distance);
private:
    double distanceFromBall;
    double angleOffBall;
    int currentAlliance;
};