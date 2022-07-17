
#include "RobotData.h"
#include "common/VisionLookup.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

void VisionLookup::RobotInit() {}

void VisionLookup::RobotPeriodic(const RobotData &robotData, VisionLookupData &visionLookupData)
{
    // hood angle map HIGH HUB
    // key is the distance in feet
    // assigned value is the hood position in angle measurment(degrees)
    visionMap[4] = 24.00;
    visionMap[5] = 25.90;
    visionMap[6] = 26.45; 
    visionMap[7] = 28; 
    visionMap[8] = 31.0; 
    visionMap[9] = 31.50; 
    visionMap[10] = 32.90; // 7.5 feet in real life with bumpers (edge of tarmac) - back of robot / HOOD ROLLER PID tuned here
    visionMap[11] = 33.70; // 8.5 feet in real life with bumpers (side field) - back of robot
    visionMap[12] = 35.20; 
    visionMap[13] = 36.30; 
    visionMap[14] = 37.10; 
    visionMap[15] = 37.90; //  12 feet in real life with bumpers (close protected zone) - back of robot
    visionMap[16] = 38.40; 
    visionMap[17] = 38.90; 
    visionMap[18] = 40.9; 
    visionMap[19] = 42; 
    visionMap[20] = 42.5;
    visionMap[21] = 43;
    // visionMap[22] = 43;

    // velocity map
    // key is the distance in feet
    // assigned value is the desired flywheel velocity in rpm
    velocityMap[4] = 1110;
    velocityMap[5] = 1120; 
    velocityMap[6] = 1140; 
    velocityMap[7] = 1180; 
    velocityMap[8] = 1200; 
    velocityMap[9] =  1225; 
    velocityMap[10] =  1270;  
    velocityMap[11] = 1310; 
    velocityMap[12] =  1340; 
    velocityMap[13] = 1390;
    velocityMap[14] = 1420; 
    velocityMap[15] = 1460; 
    velocityMap[16] = 1510; 
    velocityMap[17] = 1560; 
    velocityMap[18] = 1610; 
    velocityMap[19] = 1660;
    velocityMap[20] = 1740;
    velocityMap[21] = 1770;
    // velocityMap[22] = 1860;

}

/**
 * @param key the distance from the hub
 * @return the hood angle at the asked for distance 
 * FOR HIGH HUB
 **/
double VisionLookup::getValue(double key){

    std::unordered_map<double,double>::const_iterator got = visionMap.find(key);
    if (got == visionMap.end())
        return 0;
    else
        return got->second;
}

/**
 * @param key the distance from the hub
 * @return the desired velocity at the asked for distance 
 **/
double VisionLookup::getVelocity(double key){
    std::unordered_map<double,double>::const_iterator got = velocityMap.find(key);
    if (got == velocityMap.end()){
        return 0;
    }else{
        return got->second;
    }

}

/**
 * @return the greatest distance from the hood map
 **/
double VisionLookup::highestVal(){
    double highest = 0;
    for (auto x : visionMap){
        if(x.first > highest){
            highest = x.first;
        }
    }
    return highest;
}

/**
 * @return the smallest distance from the hood map
 **/
double VisionLookup::lowestVal(){
    double lowest = 25;
    for (auto x : visionMap){
        if(x.first < lowest){
            lowest = x.first;
        }
    }
    return lowest;
}

/**
 * @return the greatest distance from the velocity map
 **/
double VisionLookup::highestVelocity(){
    double highest = 0;
    for (auto x : velocityMap){
        if(x.first > highest){
            highest = x.first;
        }
    }
    return highest;
}

/**
 * @return the smallest distance from the velocity map
 **/
double VisionLookup::lowestVelocity(){
    double lowest = 25;
    for (auto x : velocityMap){
        if(x.first < lowest){
            lowest = x.first;
        }
    }
    return lowest;
}





