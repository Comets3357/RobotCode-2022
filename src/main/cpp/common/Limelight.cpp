#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup)
{

    limelightData.validTarget = table->GetNumber("tv", 0.0); //valid target or not
    limelightData.xOffset =  table->GetNumber("tx", 0.0) * (pi/180); //RADIANS
    limelightData.yOffset =  table->GetNumber("ty", 0.0); //DEGREES
    limelightDistance = distanceToTarget(); //the distance in INCHES. NOT TO BE USED 

    //SETS LEDS ON LIMELIGHT
    table->PutNumber("ledMode", 0);

    if(robotData.controlData.mode == mode_climb_manual || robotData.controlData.mode == mode_climb_sa){
        table->PutNumber("pipeline", 1);
    }else{
        table->PutNumber("pipeline", 0);
    }

    //takes into account the limelight's position offset from shooter
    shooterOffset(robotData, limelightData);

    if(robotData.controlData.mode == mode_teleop_sa){
        limelightData.distanceOffset = limelightData.distanceOffset + robotData.controlData.saDistanceOffset; //adds 6 inches everytime it's clicked
    }

    //the desired hood and velocity for shooting from anywhere
    if(robotData.limelightData.validTarget == 0){
        limelightData.desiredVel = 1250; //returns rpm
        limelightData.desiredHoodRollerVel = 1250*3.5;
    }else{
        limelightData.desiredVel = getWheelVelocity(visionLookup, limelightData, robotData); //returns rpm
        limelightData.desiredHoodRollerVel = getHoodRollerVel(limelightData, robotData);
    }

    //DESIRED HOOD
    limelightData.desiredHoodPos = getHoodPOS(visionLookup, limelightData, robotData); //returns an angle

    //TURRET DIFFERENCE
    limelightData.turretDifference = -robotData.limelightData.angleOffset; // turret turning is not consistent with limelight degrees off
    //DESIRED TURRET
    limelightData.desiredTurretAngle = getTurretTurnAngle(limelightData, robotData); //position to go to to shoot


    

    //printing data to the dashboard
    frc::SmartDashboard::PutNumber("distance offset", robotData.limelightData.distanceOffset/12);
    frc::SmartDashboard::PutNumber("desired turret", robotData.limelightData.desiredTurretAngle);
    frc::SmartDashboard::PutNumber("avg desired turret", robotData.limelightData.avgDesiredTurretAngle);

    //DECOMMISSIONED
    //updates the angle to be in degrees rather than radians
    //the actual distance from the hub based on the turning of the drivebase
    //limelightData.correctDistance = correctDistance(limelightData.angleOffset, limelightData.distanceOffset);
    //averages the distances provided by the limelight in order to make the shooting sequence smoother
    //averageDistance(robotData, limelightData);

}

/**
 * @return distance from LIMELIGHT to target in INCHES
 */
double Limelight::distanceToTarget(){ 
    double radianAngle = table->GetNumber("ty", 0.0);
    radianAngle = (radianAngle + limelightAngle) * (pi / 180); //converts to radians
    return(((hubHeight + crosshairOffset)-limelightMount)/std::tan(radianAngle));
}

/**
 * stores the distance and angle to the target of the shooter rather than the limelight (adds the offset)
 * distanceoffset in INCHES 
 * angle offset in DEGREES
 */
void Limelight::shooterOffset(const RobotData &robotData, LimelightData &limelightData){
    //get to radians
    double radianAngle = robotData.limelightData.xOffset;

    //find the values of the two sides of the right triangle
    double xValue = limelightDistance*std::sin(radianAngle);
    double yValue = limelightDistance*std::cos(radianAngle);
    
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
    //limelightData.distanceOffset = 5*12; //IN INCHES

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
    double distance = limelightData.distanceOffset/12; // original distance offset is in inches, and we are converting it to feet
    double originalDistance = distance;
    limelightData.lowerVal = std::floor(distance); //lower value in ft 
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
    //use lookup table to get the desired hood positions

    // }else if(!robotData.shooterData.isHighGeneral){ //LOW HUB VALUES
    //     //use lookup table to get the desired hood positions
    //     limelightData.lowerValPos = visionLookup.getLowValue(limelightData.lowerVal);
    //     limelightData.upperValPos = visionLookup.getLowValue(limelightData.upperVal);
    // }

    //gets value from the lookup table
    limelightData.lowerValPos = visionLookup.getValue(limelightData.lowerVal);
    limelightData.upperValPos = visionLookup.getValue(limelightData.upperVal);

    //get the slope of the line between the upper and lower values (interpolating)
    //position/inch
    double desiredSlope = (limelightData.upperValPos - limelightData.lowerValPos)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired position of hood for that small distance 
    //then add that to the desired position of the lower floored value
    float desiredHood = (desiredSlope*((originalDistance - limelightData.lowerVal)*12)+limelightData.lowerValPos);
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
    double distance = limelightData.distanceOffset/12;
    double originalDistance = distance;
    limelightData.lowerVal = std::floor(distance); //lower value in ft
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
    //position/inch
    double desiredSlope = (limelightData.upperValVel - limelightData.lowerValVel)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired velocity for that small distance 
    //then add that to the desired position of the lower floored value
    return ( (desiredSlope*((originalDistance - limelightData.lowerVal)*12) + limelightData.lowerValVel) );   // 320 for front!

}

/**
 * @return the desired hood roller velocity based off of the desired flywheel velocity
 */
double Limelight::getHoodRollerVel(LimelightData &limelightData, const RobotData &robotData){
    //if you're farther back get a faster flywheel hoodroller ratio
    if(robotData.limelightData.distanceOffset >= 15*12){
        limelightData.hoodFlywheelRatio = 3.5;
    }else{
        limelightData.hoodFlywheelRatio = 3.5;
    }

    double flywheelVel = robotData.limelightData.desiredVel;
    return flywheelVel*limelightData.hoodFlywheelRatio;
}

/**
 * @return how much the turret needs to turn in order to get to the target 
 * is constantly updating based on the current Turret Angle and the angle offset from the limelight
 */
double Limelight::getTurretTurnAngle(LimelightData &limelightData, const RobotData &robotData){

    float desired = robotData.limelightData.turretDifference + robotData.shooterData.currentTurretAngle;
    double theta = (180/pi)*(std::atan(1/robotData.limelightData.distanceOffset));

    if(desired < 0 || desired > turretFullRotationDegrees){ //if you're outside of the range, go through and add/subtract 360 to get in the range

        if(desired < 0){
            desired += 360;
        }else if(desired > turretFullRotationDegrees){
            desired -= 360;
        }

        //so you're telling the turret to turn to turn to the unwrapped state, therefore, you are unwrapping
        limelightData.unwrapping = true;
        //set the upwrapping value to the newly desired position 
        unwrappingVal = desired;

    }

    //if you see a target, and youre in youre range of unwrapped, set unwrapped to false
    if(robotData.limelightData.validTarget && robotData.shooterData.currentTurretAngle > (turretZeroDegrees + 90) && robotData.shooterData.currentTurretAngle < (turretFullRotationDegrees - 90)){ 
        limelightData.unwrapping = false;
    }

    //if you arent unwrapping update the returned value to the desired one thats constantly updated
    if(!limelightData.unwrapping){
        unwrappingVal = desired;
    }

    if (unwrappingVal < std::max(theta, 0.75))
    {
        unwrappingVal = 0;
    }

    return unwrappingVal;
}

// ROLLING AVG NOT NEEDED. IT WILL SLOW DOWN TURRET

/**
 * Returns the avg distance of the last 5 cycles to make the data smoother while shooting
 */ 
// void Limelight::averageDesiredTurret(const RobotData &robotData, LimelightData &limelightData){
//     double desiredTurretAngle = robotData.limelightData.desiredTurretAngle;
//     double total = 0;

//     if(robotData.limelightData.unwrapping){
//         limelightData.desiredTurretAngles.clear();
//     }

//     //if size is less then 6 keep adding updated distances until the deque is full
//     if(robotData.limelightData.desiredTurretAngles.size() < 6){
//         limelightData.desiredTurretAngles.push_back(desiredTurretAngle);
//     }else{ //once it's full run through the deque and add it to the total
//         for(size_t i = 0; i < robotData.limelightData.desiredTurretAngles.size(); i ++){
//             total += robotData.limelightData.desiredTurretAngles.at(i);
//         }

//         //make sure to remove the first value and add an updated distance to the end
//         limelightData.desiredTurretAngles.pop_front();
//         limelightData.desiredTurretAngles.push_back(desiredTurretAngle);
//     }

//     //return the average of those distances
//     limelightData.avgDesiredTurretAngle = total / ((double)robotData.limelightData.desiredTurretAngles.size());
// }
