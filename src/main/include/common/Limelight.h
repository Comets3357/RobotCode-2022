#pragma once
#include "Constants.h"
#include "common/VisionLookup.h"


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

    double desiredHoodPos;
    double distanceToTarget;
    double correctDistance;

    //shooter corrections:
    double distanceOffset;
    double angleOffset;
};

class Limelight
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup);
    double getHorizontalOffset();
    double getVerticalOffset();
    int getTarget();
    int getPipeline(double verticalOffset);


private:
    double distanceToTarget();
    double correctDistance(double angleOffset, double originalDistance);
    double getHoodPOS(VisionLookup &visionLookup, LimelightData &limelightData);
    void shooterOffset(const RobotData &robotData, LimelightData &limelightData);



};