#pragma once

struct LimelightData
{
    double xOffset;
    double yOffset;
    int targetValue;
    bool validTarget;
    int pipeline; //for LED power
};

class Limelight
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData);
    double getHorizontalOffset();
    double getVerticalOffset();
    int getTarget();
    int getPipeline(double verticalOffset);

private:

};