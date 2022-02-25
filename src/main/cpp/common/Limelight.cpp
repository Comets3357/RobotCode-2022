
#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup)
{
    //updating data
    limelightData.validTarget = table->GetNumber("tv", 0.0); //valid target or not
    limelightData.distanceToTarget = distanceToTarget(); 
    limelightData.xOffset =  table->GetNumber("tx", 0.0) * (pi/180); //RADIANS
    limelightData.yOffset =  table->GetNumber("ty", 0.0);
    limelightData.angleOffset = robotData.limelightData.angleOffset; //Degrees

    //turns off limelight if not shooting
    if(robotData.controlData.shootMode == shootMode_none){
         table->PutNumber("ledMode", 1);
    }else{
        table->PutNumber("ledMode", 0);
    }
    
    //table->PutNumber("ledMode", 0);

    //updates the angle to be in degrees rather than radians
    //the actual distance from the hub based on the turning of the drivebase
    //limelightData.correctDistance = correctDistance(limelightData.angleOffset, limelightData.distanceOffset);

    //moves the limelight data over to the actual position of the shooter
    shooterOffset(robotData, limelightData);
    //averages the distances provided by the limelight in order to make the shooting sequence smoother
    averageDistance(robotData, limelightData);

    //the desired hood and velocity for shooting from anywhere
    limelightData.desiredHoodPos = getHoodPOS(visionLookup, limelightData, robotData); //returns an angle
    limelightData.desiredVel = getWheelVelocity(visionLookup, limelightData, robotData); //returns rpm
    
    //printing data to the dashboard
    frc::SmartDashboard::PutNumber("distance offset", robotData.limelightData.distanceOffset);
    frc::SmartDashboard::PutNumber("angleOffset", limelightData.angleOffset);
    //frc::SmartDashboard::PutNumber("desired hood", robotData.limelightData.desiredHoodPos);
    //frc::SmartDashboard::PutNumber("final correct distance", robotData.limelightData.correctDistance);
}

/**
 * @return distance from limelight to target in inches
 */
double Limelight::distanceToTarget(){ 
    double radianAngle = table->GetNumber("ty", 0.0);
    radianAngle = (radianAngle + limelightAngle) * (pi / 180);
    return(((hubHeight + crosshairOffset)-limelightMount)/std::tan(radianAngle));

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
    limelightData.angleOffset = (std::asin(xValueOffset/limelightData.distanceOffset));
    limelightData.angleOffset *= (180/pi);
}

/**
 * @returns finds the corrected distance for when we turn the robot
 * NOT USED
 */
double Limelight::correctDistance(double angleOffset, double originalDistance)
{
    return (originalDistance + (shooterDistanceFromCenterOfBot - (std::cos(angleOffset)))*shooterDistanceFromCenterOfBot);
}

/**
 * @return the desired hood position using lookup table
 */
double Limelight::getHoodPOS(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData){
    double distance = limelightData.distanceOffset;
    //double distance = limelightData.avgDistance;
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

    
    //checks to see if the controldata for shooting in the highhub is true
    // if(robotData.shooterData.isHighGeneral){
    //     //use lookup table to get the desired hood positions
        

    // }else if(!robotData.shooterData.isHighGeneral){ //LOW HUB VALUES
    //     //use lookup table to get the desired hood positions
    //     limelightData.lowerValPos = visionLookup.getLowValue(limelightData.lowerVal);
    //     limelightData.upperValPos = visionLookup.getLowValue(limelightData.upperVal);
    // }

    //gets value from the lookup table
    limelightData.lowerValPos = visionLookup.getValue(limelightData.lowerVal);
    limelightData.upperValPos = visionLookup.getValue(limelightData.upperVal);

    //get the slope of the line between the upper and lower values
    double desiredSlope = (limelightData.upperValPos - limelightData.lowerValPos)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired position of hood for that small distance 
    //then add that to the desired position of the lower floored value
    return (desiredSlope*(orignalDistance - limelightData.lowerVal*12)+limelightData.lowerValPos);
}

/**
 * @return the desired flywheel velocity using lookup table
 */
double Limelight::getWheelVelocity(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData){
    double distance = limelightData.distanceOffset;
    //double distance = limelightData.avgDistance;
    double orignalDistance = distance;
    limelightData.lowerVal = std::floor(distance/12); //lower value in ft
    limelightData.upperVal = limelightData.lowerVal +1; //upper value in ft

    //if either of the int values are higher than the highest lookup table value,
    //set the values to the highest lookup table value
    if(limelightData.lowerVal > visionLookup.highestVelocity()){
        limelightData.lowerVal = visionLookup.highestVelocity();
    }

    if(limelightData.upperVal > visionLookup.highestVelocity()){
        limelightData.upperVal = visionLookup.highestVelocity();
    }
    
    // if(robotData.shooterData.isHighGeneral){
    //     //use lookup table to get the desired velocities
    //     limelightData.lowerValVel = visionLookup.getVelocity(limelightData.lowerVal);
    //     limelightData.upperValVel = visionLookup.getVelocity(limelightData.upperVal);

    // }else if(!robotData.shooterData.isHighGeneral){ //LOW HUB
    //     //use lookup table to get the desired velocities
    //     limelightData.lowerValVel = visionLookup.getLowVelocity(limelightData.lowerVal);
    //     limelightData.upperValVel = visionLookup.getLowVelocity(limelightData.upperVal);

    // }

    //gets value from the lookup table
    limelightData.lowerValVel = visionLookup.getVelocity(limelightData.lowerVal);
    limelightData.upperValVel = visionLookup.getVelocity(limelightData.upperVal);

    //get the slope of the line between the upper and lower values
    double desiredSlope = (limelightData.upperValVel - limelightData.lowerValVel)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired velocity for that small distance 
    //then add that to the desired position of the lower floored value
    return (desiredSlope*(orignalDistance - limelightData.lowerVal*12)+limelightData.lowerValVel);

}

/**
 * Returns the avg distance of the last 5 cycles to make the data smoother while shooting
 */
void Limelight::averageDistance(const RobotData &robotData, LimelightData &limelightData){
    double distance = robotData.limelightData.distanceOffset;
    double total = 0;

    //if size is less then 6 keep adding updated distances until the deque is full
    // if(robotData.limelightData.distances.size() < 6){
    //     limelightData.distances.push_back(distance);
    // }else{ //once it's full run through the deque and add it to the total
    //     for(int i = 0; (unsigned)i < robotData.limelightData.distances.size(); i ++){
    //         total += robotData.limelightData.distances.at(i);
    //     }

    //     //make sure to remove the first value and add an updated distance to the end
    //     limelightData.distances.pop_front();
    //     limelightData.distances.push_back(distance);
    // }

    //return the average of those distances
    limelightData.avgDistance = distance;

}
