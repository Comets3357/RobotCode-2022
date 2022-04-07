
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

//hood roller ratio back to 3.5
    velocityMap[15] = 1720; 
    velocityMap[16] = 1800; 
    velocityMap[17] = 1950; 
    velocityMap[18] = 1950; 
    velocityMap[19] = 2100;

    // hood angle map LOW HUB
    // lowVisionMap[4] = 25.3;
    // lowVisionMap[5] = 27.3;
    // lowVisionMap[6] = 28.8;
    // lowVisionMap[7] = 30.6;
    // lowVisionMap[8] = 31.9;
    // lowVisionMap[9] = 34.7;
    // lowVisionMap[10] = 35.1;
    // lowVisionMap[11] = 37.25;
    // lowVisionMap[12] = 37.7;
    // lowVisionMap[13] = 39;
    // lowVisionMap[14] = 39.4;
    // lowVisionMap[15] = ;
    // lowVisionMap[16] = ;
    // lowVisionMap[17] = ;
    // lowVisionMap[18] = ;
    // lowVisionMap[19] = ;
    // lowVisionMap[20] = ;

    
    // velocity map LOW HUB
    // lowVelocityMap[4] = 1450;
    // lowVelocityMap[5] = 1500;
    // lowVelocityMap[6] = 1600;
    // lowVelocityMap[7] = 1650;
    // lowVelocityMap[8] = 1750;
    // lowVelocityMap[9] = 1750;
    // lowVelocityMap[10] = 1775;
    // lowVelocityMap[11] = 1850;
    // lowVelocityMap[12] = 1950;
    // lowVelocityMap[13] = 2000;
    // lowVelocityMap[14] = 2100;
    // lowVelocityMap[15] = 2300;
    // lowVelocityMap[16] = ;
    // lowVelocityMap[17] = ;
    // lowVelocityMap[18] = ;
    // lowVelocityMap[19] = ;
    // lowVelocityMap[20] = ;
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




