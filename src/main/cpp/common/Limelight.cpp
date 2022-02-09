
#include "RobotData.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Limelight::RobotInit() {}

/**
 * @return horizontal offset angle from limelight
 */
double Limelight::getHorizontalOffset()
{
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the table
    return (table->GetNumber("tx", 0.0)) + 0.5;                                                         //offset
}

/**
 * @return vertical offset angle from limelight
 */
double Limelight::getVerticalOffset()
{
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the table
    return table->GetNumber("ty", 0.0);
}

/**
 * @return if a target is seen or not 0 or 1
 */
int Limelight::getTarget()
{
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the table
    return table->GetNumber("tv", 0.0);
}

/**
 * @param verticalOffset y offset from limelight
 * @return needed pipeline based off how close to the target the bot is
 */
int Limelight::getPipeline(double verticalOffset)
{

    int pipeline;

    if (verticalOffset > 14)
    {
        pipeline = 1;
    }
    else if (verticalOffset > 9)
    {
        pipeline = 2;
    }
    else if (verticalOffset > 6)
    {
        pipeline = 3;
    }
    else if (verticalOffset > 1)
    {
        pipeline = 4;
    }
    else{
        pipeline = 5;
    }

    //basically if you can see the target turn on the limelight otherwise don't
    return pipeline;
}

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the networktable

    //updating data
    limelightData.xOffset = getHorizontalOffset();
    limelightData.yOffset = getVerticalOffset();
    limelightData.targetValue = getTarget();
    limelightData.validTarget = table->GetNumber("tv", 0.0);
    table->PutNumber("ledMode", 1);
}

