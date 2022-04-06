
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
    visionMap[4] = 23.31;
    visionMap[5] = 23.5;
    visionMap[6] = 25.07; 
    visionMap[7] = 26.21; 
    visionMap[8] = 27.37; 
    visionMap[9] = 29.62; 
    visionMap[10] = 33.78; 
    visionMap[11] = 34.90; 
    visionMap[12] = 36.49; 
    visionMap[13] = 36.39; 
    visionMap[14] = 37.27; 
    visionMap[15] = 37.28; 
    visionMap[16] = 37.29; 
    visionMap[17] = 40.02; 
    visionMap[18] = 39.87; 
    visionMap[19] = 40; 

    // velocity map
    // key is the distance in feet
    // assigned value is the desired flywheel velocity in rpm
    velocityMap[4] = 1250;
    velocityMap[5] = 1250; 
    velocityMap[6] = 1300; 
    velocityMap[7] = 1325; 
    velocityMap[8] = 1370; 
    velocityMap[9] =  1430; 
    velocityMap[10] =  1430; 
    velocityMap[11] = 1510; 
    velocityMap[12] =  1550; 
    velocityMap[13] = 1600; 
    velocityMap[14] = 1650; 
    velocityMap[15] = 1720; 
    velocityMap[16] = 1800; 
    velocityMap[17] = 1950; 
    velocityMap[18] = 1950; 
    velocityMap[19] = 2100;

    // hood angle map LOW HUB
    forwardVisionMap[4] = 25.3;
    forwardVisionMap[5] = 27.3;
    forwardVisionMap[6] = 28.8;
    forwardVisionMap[7] = 30.6;
    forwardVisionMap[8] = 31.9;
    forwardVisionMap[9] = 34.7;
    forwardVisionMap[11] = 37.25;
    forwardVisionMap[10] = 35.1;
    forwardVisionMap[12] = 37.7;
    forwardVisionMap[13] = 39;
    forwardVisionMap[14] = 39.4;
    forwardVisionMap[15] = 23.31;
    forwardVisionMap[16] = 23.31;
    forwardVisionMap[17] = 23.31;
    forwardVisionMap[18] = 23.31;
    forwardVisionMap[19] = 23.31;
    forwardVisionMap[20] = 23.31;
    
    //velocity map LOW HUB
    fowardVelocityMap[4] = 1450;
    fowardVelocityMap[5] = 1500;
    fowardVelocityMap[6] = 1600;
    fowardVelocityMap[7] = 1650;
    fowardVelocityMap[8] = 1750;
    fowardVelocityMap[9] = 1750;
    fowardVelocityMap[10] = 1775;
    fowardVelocityMap[11] = 1850;
    fowardVelocityMap[12] = 1950;
    fowardVelocityMap[13] = 2000;
    fowardVelocityMap[14] = 2100;
    fowardVelocityMap[15] = 2300;
    fowardVelocityMap[16] = 2300;
    fowardVelocityMap[17] = 2300;
    fowardVelocityMap[18] = 2300;
    fowardVelocityMap[19] = 2300;
    fowardVelocityMap[20] = 2300;
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

/**
 * @param key the distance from the hub
 * @return the hood angle at the asked for distance
 * FOR LOW HUB 
 **/
// double VisionLookup::getLowValue(double key){

//     std::unordered_map<double,double>::const_iterator got = lowVisionMap.find(key);
//     if (got == lowVisionMap.end())
//         return 0;
//     else
//         return got->second;
// }

/**
 * @param key the distance from the hub
 * @return the desired velocity at the asked for distance 
 * FOR LOW HUB
 **/
// double VisionLookup::getLowVelocity(double key){
//     std::unordered_map<double,double>::const_iterator got = lowVelocityMap.find(key);
//     if (got == lowVelocityMap.end())
//         return 0;
//     else
//         return got->second;

// }




