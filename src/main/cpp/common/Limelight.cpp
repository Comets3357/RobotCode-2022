#include "RobotData.h"

void Limelight::AutonomousInit(LimelightData &limelightData){
    limelightData.unwrapping = false;
}

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup)
{
    double tempOffset;

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
    
    //Intermediate desired position and velocity
    limelightData.desiredHoodPos = getHoodPOS(visionLookup, limelightData, robotData); //returns an angle
    backwardDesiredVel = getWheelVelocity(visionLookup, limelightData, robotData); //returns rpm

    //the desired hood and velocity for shooting from anywhere
    if(robotData.limelightData.validTarget == 0){
        limelightData.desiredVel = 1250; //returns rpm
        limelightData.desiredHoodRollerVel = 1250*3.5;
    }else{
        limelightData.desiredVel = interpolationVel(limelightData, robotData);
        limelightData.desiredHoodRollerVel = getHoodRollerVel(limelightData, robotData);
    }

    // limelightData.desiredHoodPos = interpolationHood(limelightData, robotData);

    // tempOffset = limelightData.angleOffset;

    // if (tempOffset > 0)
    // {
    //     if (tempOffset < std::min((180/pi)*std::atan(12/limelightData.distanceOffset), (double)2))
    //     {
    //         limelightData.angleOffset = 0;
    //     }
    // }
    // else if (tempOffset < 0)
    // {
    //     if (std::abs(tempOffset) < std::min(std::abs((180/pi)*std::atan(12/limelightData.distanceOffset)), (double)6))
    //     {
    //         limelightData.angleOffset = 0;
    //     }
    // }
    //TURRET DIFFERENCE
    limelightData.turretDifference = -robotData.limelightData.angleOffset; // turret turning is not consistent with limelight degrees off

    if ((std::abs(limelightData.turretDifference)) < std::min((180/pi)*std::atan(8/limelightData.distanceOffset), (double)4))
    {
        limelightData.turretDifference = 0;
    }

    //DESIRED TURRET
    limelightData.desiredTurretAngle = getTurretTurnAngle(limelightData, robotData); //position to go to to shoot

    //printing data to the dashboard
    frc::SmartDashboard::PutNumber("distance offset", robotData.limelightData.distanceOffset/12);
    //frc::SmartDashboard::PutNumber("desired turret", robotData.limelightData.desiredTurretAngle);
    //frc::SmartDashboard::PutBoolean("Unwrapping", limelightData.unwrapping);

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

    //calculate the angle between the shooter since it is different from that given by the limelight
    limelightData.angleOffset = (std::asin(xValueOffset/limelightData.distanceOffset));
    limelightData.angleOffset *= (180/pi);
}

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

    //gets value from the lookup table
    limelightData.lowerValVel = visionLookup.getVelocity(limelightData.lowerVal);
    limelightData.upperValVel = visionLookup.getVelocity(limelightData.upperVal);

    //get the slope of the line between the upper and lower values (interpolate)
    //position/inch
    double desiredSlope = (limelightData.upperValVel - limelightData.lowerValVel)/12; 

    //multiply the difference in the distance and floored value by the slope to get desired velocity for that small distance 
    //then add that to the desired position of the lower floored value

    // if(robotData.limelightData.distanceOffset > 14*12){
    //     return ( (desiredSlope*((originalDistance - limelightData.lowerVal)*12) + limelightData.lowerValVel));   // 320 for front!
    // }else{
    //     return ( (desiredSlope*((originalDistance - limelightData.lowerVal)*12) + limelightData.lowerValVel));   // 320 for front!
    // }
    return ( (desiredSlope*((originalDistance - limelightData.lowerVal)*12) + limelightData.lowerValVel)) + 40;

}

/**
 * @return the desired hood roller velocity based off of the desired flywheel velocity
 */
double Limelight::getHoodRollerVel(LimelightData &limelightData, const RobotData &robotData){
    float hoodFlywheelRatio;

    //if you're farther back get a faster flywheel hoodroller ratio
    if(robotData.limelightData.distanceOffset >= 15*12){
        hoodFlywheelRatio = 3.5;
    }else{
        hoodFlywheelRatio = 3.5;
    }

    double flywheelVel = robotData.limelightData.desiredVel;
    return flywheelVel*hoodFlywheelRatio;
}

/**
 * @return how much the turret needs to turn in order to get to the target 
 * is constantly updating based on the current Turret Angle and the angle offset from the limelight
 */
double Limelight::getTurretTurnAngle(LimelightData &limelightData, const RobotData &robotData){
    
    
    float desired = robotData.limelightData.turretDifference + robotData.shooterData.currentTurretAngle;

    if((desired < 0 || desired > turretFullRotationDegrees) && frc::DriverStation::IsEnabled()){ //if you're outside of the range, go through and add/subtract 360 to get in the range

        if(desired < 0){
            desired += 430;
        }else if(desired > turretFullRotationDegrees){
            desired -= 430;
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

    return unwrappingVal;
}

double Limelight::interpolationVel(LimelightData &limelightData, const RobotData &robotData)
{
    //take in the desired value from front and from back
    //take those two values and the current position of the turret
    
    double velBackwards = backwardDesiredVel;
    double velFowards = velBackwards+forwardVelOffset;
    
    // if (limelightData.distanceOffset < 8) {
        // backwardDesiredVel += 10;
    // }

    // if(robotData.limelightData.distanceOffset > change*12){
    //     return backwardDesiredVel;
    // }else{
    double slope = 0;
    double turretAngle = robotData.shooterData.currentTurretAngle;

    if(((robotData.shooterData.currentTurretAngle <= turretMiddleDegrees) && (robotData.shooterData.currentTurretAngle >= turretBackwardsDegrees_C)) || (robotData.shooterData.currentTurretAngle >= turretBackwardsDegrees_CCW))
    { //on the right side of the turret   
        // double slope = (velFowards - velBackwards)/(turretMiddleDegrees - turretBackwardsDegrees_C);
        
        if ((robotData.shooterData.currentTurretAngle >= turretBackwardsDegrees_CCW) || (robotData.shooterData.currentTurretAngle <= (turretMiddleDegrees - turretBackwardsDegrees_C) / 2))
        {
            if (turretAngle > turretBackwardsDegrees_CCW)
            {
                turretAngle -= (turretBackwardsDegrees_CCW - turretBackwardsDegrees_C);
            }
            slope = (-0.5007 + (0.04477 * (turretAngle)) - (0.0002661 * (pow(turretAngle, 2))));
        }
        else 
        {
            slope = (velFowards - velBackwards)/(turretMiddleDegrees - turretBackwardsDegrees_C);
        }

        return slope*((int)(turretAngle - turretBackwardsDegrees_C)%360) + velBackwards; 
    }
    else
    { //left side of the robot
        // double slope = (velFowards - velBackwards)/(turretMiddleDegrees - turretBackwardsDegrees_CCW);

        if ((robotData.shooterData.currentTurretAngle <= turretBackwardsDegrees_C) || (robotData.shooterData.currentTurretAngle >= (((turretBackwardsDegrees_CCW - turretMiddleDegrees) / 2) + turretMiddleDegrees)))
        {
            if (robotData.shooterData.currentTurretAngle <= turretBackwardsDegrees_C)
            {
                turretAngle += (turretBackwardsDegrees_CCW - turretBackwardsDegrees_C);
            }
            slope = (0.0002384 * (pow(turretAngle, 2))) - (0.1628 * turretAngle) + 26.46;
        }
        else
        {
            slope = (velFowards - velBackwards)/(turretMiddleDegrees - turretBackwardsDegrees_CCW); 
        }

    // slope and turretangle - turretbackdegrees will always produce a positive number
    return slope*((int)(turretAngle - turretBackwardsDegrees_CCW)%360) + velBackwards;
        
    }
}

double Limelight::interpolationHood(LimelightData &limelightData, const RobotData &robotData){
    double hoodBackwards = backwardDesiredHood;
    double hoodFowards = hoodBackwards;

    if(robotData.limelightData.distanceOffset > change*12){
        hoodFowards = hoodBackwards + forwardHoodOffsetFar;
        //hoodBackwards -= 0.3;
    }else{
        hoodFowards = hoodBackwards + forwardHoodOffsetClose;  
    }

    double newHood;

    if(((robotData.shooterData.currentTurretAngle <= turretMiddleDegrees) && (robotData.shooterData.currentTurretAngle >= turretBackwardsDegrees_C)) || (robotData.shooterData.currentTurretAngle >= turretBackwardsDegrees_CCW)){ //on the right side of the turret   
        double slope = (hoodFowards - hoodBackwards)/(turretMiddleDegrees - turretBackwardsDegrees_C);
        newHood = slope*((int)(robotData.shooterData.currentTurretAngle - turretBackwardsDegrees_C)%360) + hoodBackwards; 
    }else{ //left side of the robot
        double slope = (hoodFowards - hoodBackwards)/(turretMiddleDegrees - turretBackwardsDegrees_CCW);
        newHood = slope*((int)(robotData.shooterData.currentTurretAngle - turretBackwardsDegrees_CCW)%360) + hoodBackwards;
    }

    if(newHood < hoodAngleIn){
        return hoodAngleIn;
    }else if(newHood > hoodAngleOut){
        return hoodAngleOut;
    }else{
        return newHood;
    }

}
