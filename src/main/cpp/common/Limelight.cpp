
#include "RobotData.h"

void Limelight::RobotInit(const RobotData &robotData) 
{
    
}

/**
 * @return vertical offset angle from limelight
 */
double Limelight::getHorizontalOffset()
{
    return table->GetNumber("tx", 0.0) * (3.141592653589793238463 / 180);
}

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup)
{

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the networktable

    //updating data
    limelightData.validTarget = table->GetNumber("tv", 0.0);
    limelightData.distanceToTarget = distanceToTarget();

    //moves the limelight data over to the actual position of the shooter
    shooterOffset(robotData, limelightData);

    //x offset will need to change to adjusted value for shooter
    limelightData.correctDistance = correctDistance(limelightData.angleOffset, limelightData.distanceOffset);

    limelightData.desiredHoodPos = getHoodPOS(visionLookup, limelightData);
    
    //returns offset distance and offset angle to robotdata

    frc::SmartDashboard::PutNumber("limelight y offset", robotData.limelightData.yOffset);
    frc::SmartDashboard::PutNumber("limelight x offset", robotData.limelightData.xOffset);
    frc::SmartDashboard::PutNumber("distance offset", robotData.limelightData.distanceOffset);
    frc::SmartDashboard::PutNumber("desired hood", robotData.limelightData.desiredHoodPos);
}

/**
 * @return distance from limelight to target in inches
 */
double Limelight::distanceToTarget(){ 
    double radianAngle = table->GetNumber("ty", 0.0);
    radianAngle = (radianAngle + limelightAngle) * (3.141592653589793238463 / 180);
    return((hubHeight-limelightMount)/std::tan(radianAngle));
}

/**
 * stores the distance and angle to the target of the shooter rather than the limelight (adds the offset)
 */
void Limelight::shooterOffset(const RobotData &robotData, LimelightData &limelightData){
    //get to radians
    double radianAngle = robotData.limelightData.xOffset;

    //find the values of the two sides of the right triangle
    double xValue = robotData.limelightData.distanceToTarget*std::sin(radianAngle);
    double yValue = robotData.limelightData.distanceToTarget*std::cos(radianAngle);
    
    double xValueOffset = 0;

    //depending on which side of the target we are on, either add the distance between the camera and limelight or subtract them
    if(robotData.limelightData.xOffset >= 0){
        xValueOffset = xValue+xcameraDistanceFromBot;
    }else{
        xValueOffset = xValue-xcameraDistanceFromBot;
    }

    //account for the fact that the limelight is further forward than the center shooter itself
    double yValueOffset = yValue + ycameraDistanceFromBot;

    //calculate the distance from the shooter to target using pythagorian theorem with the new x and y values
    limelightData.distanceOffset = std::sqrt(std::pow(yValueOffset,2)+std::pow(xValueOffset,2));

    //calculate the angle between the shooter since it is different from that given by the limelight
    limelightData.angleOffset = (1/std::sin(xValueOffset/limelightData.distanceOffset));
}

/**
 * @returns finds the corrected distance for when we turn the robot
 */
double Limelight::correctDistance(double angleOffset, double originalDistance)
{
    return (originalDistance + (shooterDistanceFromCenterOfBot - (std::cos(angleOffset)))*shooterDistanceFromCenterOfBot);
}

/**
 * @return the desired hood position using lookup table
 */
double Limelight::getHoodPOS(VisionLookup &visionLookup, LimelightData &limelightData){
    double distance = limelightData.correctDistance;
    double orignalDistance = distance;
    limelightData.lowerVal = std::floor(distance/12); //lower value in ft
    limelightData.upperVal = limelightData.lowerVal +1; //upper value in ft
    
    //if either of the int values are higher than the highest lookup table value,
    //set the values to the highest lookup table value
    if(limelightData.lowerVal > visionLookup.highestVal()){
        limelightData.lowerVal = visionLookup.highestVal();
    }

    if(limelightData.upperVal > visionLookup.highestVal()){
        limelightData.upperVal = visionLookup.highestVal();
    }

    //use lookup table to get the desired hood positions
    limelightData.lowerValPos = visionLookup.getValue(limelightData.lowerVal);
    limelightData.upperValPos = visionLookup.getValue(limelightData.upperVal);

    //get the slope of the line between the upper and lower values
    double desiredSlope = (limelightData.upperValPos - limelightData.lowerValPos)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired position of hood for that small distance 
    //then add that to the desired position of the lower floored value
    return desiredSlope*(orignalDistance - limelightData.lowerVal*12)+limelightData.lowerValPos;

}




