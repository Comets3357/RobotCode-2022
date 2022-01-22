
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

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup)
{

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the networktable

    //updating data
    limelightData.xOffset = getHorizontalOffset();
    limelightData.yOffset = getVerticalOffset();
    limelightData.targetValue = getTarget();
    limelightData.validTarget = table->GetNumber("tv", 0.0);

    table->PutNumber("pipeline", getPipeline(robotData.limelightData.yOffset)); //set the pipeline based on y offset

    limelightData.desiredHoodPos = getHoodPOS(visionLookup, limelightData);

    frc::SmartDashboard::PutNumber("limelight y offset", robotData.limelightData.yOffset);
    frc::SmartDashboard::PutNumber("limelight x offset", robotData.limelightData.xOffset);

}

/**
 * @return the desired hood position using lookup table
 */
double Limelight::getHoodPOS(VisionLookup &visionLookup, LimelightData &limelightData){
    double distance = distanceToTarget();
    double orignalDistance = distance;
    limelightData.lowerVal = std::floor(distance/12); //lower value in ft
    limelightData.upperVal = limelightData.lowerVal +1; //upper value in ft

    //use lookup table to get the desired hood positions
    limelightData.lowerValPos = visionLookup.getValue(limelightData.lowerVal);
    limelightData.upperValPos = visionLookup.getValue(limelightData.upperVal);

    //if either of the int values are higher than the highest lookup table value,
    //set the values to the highest lookup table value
    if(limelightData.lowerVal > visionLookup.highestVal()){
        limelightData.lowerVal = visionLookup.highestVal();
    }

    if(limelightData.upperVal > visionLookup.highestVal()){
        limelightData.upperVal = visionLookup.highestVal();
    }

    //get the slope of the line between the upper and lower values
    double desiredSlope = (limelightData.upperValPos - limelightData.lowerValPos)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired position of hood for that small distance 
    //then add that to the desired position of the lower floored value
    return desiredSlope*(orignalDistance - limelightData.lowerVal*12)+limelightData.lowerValPos;

}

/**
 * @return distance from limelight to target in inches
 */
double Limelight::distanceToTarget(){ 
    double radianAngle = (getVerticalOffset()+limelightAngle)*(3.141592653589793238463/180);
    return((hubHeight-limelightMount)/std::tan(radianAngle));
}




