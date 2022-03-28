
#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup)
{
    //updating data
    // int x = 0;
    // if(x > 40){
    //     limelightData.validTarget = table->GetNumber("tv", 0.0); //valid target or not
    //     x = (x+1)%50;
    // }else{
    //     x = (x+1)%50;
    // }

    limelightData.validTarget = table->GetNumber("tv", 0.0); //valid target or not

    limelightData.xOffset =  table->GetNumber("tx", 0.0) * (pi/180); //RADIANS
    limelightData.yOffset =  table->GetNumber("ty", 0.0);
    limelightData.distanceToTarget = distanceToTarget(); //the distance

    //turns off limelight if not shooting
    // if(robotData.controlData.shootMode == shootMode_none){
    //     table->PutNumber("pipeline", 1);
    // }else{
    //     table->PutNumber("pipeline", 0);
    // }

    table->PutNumber("ledMode", 0);
    table->PutNumber("pipeline", 0);

    
    

    //updates the angle to be in degrees rather than radians
    //the actual distance from the hub based on the turning of the drivebase
    //limelightData.correctDistance = correctDistance(limelightData.angleOffset, limelightData.distanceOffset);

    //takes into account the limelight's position offset from shooter
    shooterOffset(robotData, limelightData);

    //averages the distances provided by the limelight in order to make the shooting sequence smoother
    //averageDistance(robotData, limelightData);

    //the desired hood and velocity for shooting from anywhere
    if(robotData.limelightData.validTarget == 0){
        limelightData.desiredVel = 1300; //returns rpm
        limelightData.desiredHoodRollerVel = 1300*3.5;
    }else{
        limelightData.desiredVel = getWheelVelocity(visionLookup, limelightData, robotData); //returns rpm
        limelightData.desiredHoodRollerVel = getHoodRollerVel(limelightData, robotData);
    }

    limelightData.desiredHoodPos = getHoodPOS(visionLookup, limelightData, robotData); //returns an angle
    

    //TURRET 
    limelightData.turretDifference = -robotData.limelightData.angleOffset;

    limelightData.desiredTurretAngle = getTurretTurnAngle(limelightData, robotData); //position to go to to shoot

    if(robotData.controlData.mode == mode_teleop_sa){
        limelightData.distanceOffset = limelightData.distanceOffset + robotData.controlData.saDistanceOffset; //adds 6 inches everytime it's clicked
    }

    
    //printing data to the dashboard
    frc::SmartDashboard::PutNumber("distance offset", robotData.limelightData.distanceOffset);
    //frc::SmartDashboard::PutNumber("desired turret angle", limelightData.desiredTurretAngle);
    frc::SmartDashboard::PutNumber("desired hood", robotData.limelightData.desiredHoodPos);
    frc::SmartDashboard::PutNumber("desired hood roller", robotData.limelightData.desiredHoodRollerVel);
    //frc::SmartDashboard::PutNumber("final correct distance", robotData.limelightData.correctDistance);

    // if(robotData.controlData.mDistanceOffsetAdd){
    //     limelightData.distanceOffset +=3; //adds 3 inches everytime it's clicked
    // }else if(robotData.controlData.mDistanceOffsetSubtract){
    //     limelightData.distanceOffset -=3; //adds 3 inches everytime it's clicked

    // }

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

    //calculate the distance from the shooter to target using pythagorian theorem with the new x and y values (sorry for the spelling)
    limelightData.distanceOffset = std::sqrt(std::pow(yValueOffset,2)+std::pow(xValueOffset,2));

    //calculate the angle between the shooter since it is different from that given by the limelight
    limelightData.angleOffset = (std::asin(xValueOffset/limelightData.distanceOffset));
    limelightData.angleOffset *= (180/pi);
    frc::SmartDashboard::PutNumber("limelight angle diff real", limelightData.angleOffset);
}

/**
 * @returns finds the corrected distance for when we turn the robot
 * NOT USED
 */
// double Limelight::correctDistance(double angleOffset, double originalDistance)
// {
//     return (originalDistance + (shooterDistanceFromCenterOfBot - (std::cos(angleOffset)))*shooterDistanceFromCenterOfBot);
// }

/**
 * @return the desired hood position using lookup table
 */
double Limelight::getHoodPOS(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData){
    double distance = limelightData.distanceOffset;
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

    //get the slope of the line between the upper and lower values (interpolating)
    double desiredSlope = (limelightData.upperValPos - limelightData.lowerValPos)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired position of hood for that small distance 
    //then add that to the desired position of the lower floored value
    float desiredHood = (desiredSlope*(orignalDistance - limelightData.lowerVal*12)+limelightData.lowerValPos);
    if(desiredHood > hoodAngleOut){
        return hoodAngleOut;
    }else if(desiredHood < hoodAngleIn){
        return hoodAngleIn;
    }else{
        return desiredHood;
    }
}

/**
 * @return the desired flywheel velocity using lookup table
 */
double Limelight::getWheelVelocity(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData){
    double distance = limelightData.distanceOffset;
    double orignalDistance = distance;
    limelightData.lowerVal = std::floor(distance/12); //lower value in ft
    limelightData.upperVal = limelightData.lowerVal +1; //upper value in ft


    //if either of the distance values are lower than the lowest lookup table value,
    //set the values to the lowest lookup table value 

    if(limelightData.lowerVal < visionLookup.lowestVelocity()){
        limelightData.lowerVal = visionLookup.lowestVelocity();
    }

    if(limelightData.upperVal < visionLookup.lowestVelocity()){
        limelightData.upperVal = visionLookup.lowestVelocity();
    }

    //if either of the distance values are higher than the highest lookup table value,
    //set the values to the highest lookup table value 

    if(limelightData.lowerVal > visionLookup.highestVelocity()){
        limelightData.upperVal = visionLookup.highestVelocity();
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

    //get the slope of the line between the upper and lower values (interpolate)
    double desiredSlope = (limelightData.upperValVel - limelightData.lowerValVel)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired velocity for that small distance 
    //then add that to the desired position of the lower floored value
    return (desiredSlope*(orignalDistance - limelightData.lowerVal*12)+limelightData.lowerValVel);

}

/**
 * @return the desired hood roller velocity based off of the desired flywheel velocity
 */
double Limelight::getHoodRollerVel(LimelightData &limelightData, const RobotData &robotData){
    if(robotData.limelightData.distanceOffset >= 15){
        limelightData.hoodFlywheelRatio = 3.5;
    }else{
        limelightData.hoodFlywheelRatio = 3;
    }


    double flywheelVel = robotData.limelightData.desiredVel;
    return flywheelVel*limelightData.hoodFlywheelRatio;
    //return flywheelVel*3.5;
}

/**
 * @return how much the turret needs to turn in order to get to the target 
 * is constantly updating based on the current Turret Angle and the angle offset from the limelight
 */
double Limelight::getTurretTurnAngle(LimelightData &limelightData, const RobotData &robotData){
    float desired = robotData.limelightData.turretDifference + robotData.shooterData.currentTurretAngle;
    
    float snapshot;

    

    
    if(desired < 0 || desired > turretFullRotationDegrees){
        //so you're telling the turret to turn to turn to the unwrapped state, therefore, you are unwrapping
        limelightData.unwrapping = true; 

        if(desired < 0){
            desired += 360;
        }else if(desired > turretFullRotationDegrees){
            desired -=360;
        }

    }

    return desired;

    // if(limelightData.unwrapping){ //if you're trying to unwrap the turret, and the turret sees target and the difference is less than 45 from the hard stop
    //     if(std::abs(desired - turretZeroDegrees) < 45 || std::abs(desired - turretFullRotationDegrees) < 45){
    //         return snapshot;
    //     }else{

    //     }
    // }
    

}



/**
 * Returns the avg distance of the last 5 cycles to make the data smoother while shooting
 */
void Limelight::averageDistance(const RobotData &robotData, LimelightData &limelightData){
    // double distance = robotData.limelightData.desiredTurretAngle;
    // double total = 0;

    // //if size is less then 6 keep adding updated distances until the deque is full
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

    // //return the average of those distances
    // limelightData.avgDistance = distance;
}
