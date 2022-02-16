#pragma once
#include "Constants.h"
#include "common/VisionLookup.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/span.h>
#include <cmath>
#include <deque>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>


struct RobotData;

struct LimelightData
{
    double xOffset;
    double yOffset;
    int targetValue;
    bool validTarget;
    int pipeline; //for LED power

    int upperVal;
    int lowerVal;

    double upperValPos;
    double lowerValPos;

    double lowerValVel;
    double upperValVel;

    double desiredHoodPos;
    double desiredVel;
    double distanceToTarget;
    double correctDistance;

    //shooter corrections:
    double distanceOffset;
    double angleOffset;

    std::deque<double> distances;

    double avgDistance = 0;

};

class Limelight
{

public:
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup);
    double getHorizontalOffset();
    double getVerticalOffset();
    int getTarget();
    int getPipeline(double verticalOffset);


private:
    double distanceToTarget();
    double correctDistance(double angleOffset, double originalDistance);
    double getHoodPOS(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData);
    double getWheelVelocity(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData);
    void shooterOffset(const RobotData &robotData, LimelightData &limelightData);

    void averageDistance(const RobotData &robotData, LimelightData &limelightData);

    //network table for limelight
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

};