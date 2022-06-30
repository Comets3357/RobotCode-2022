
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
    visionMap[4] = 24.91;
    visionMap[5] = 26.11;
    visionMap[6] = 26.85; 
    visionMap[7] = 27.7; 
    visionMap[8] = 30.5; 
    visionMap[9] = 31.5; 
    visionMap[10] = 32.90; // 7.5 feet in real life with bumpers (edge of tarmac) - back of robot
    visionMap[11] = 33.5; // 8.5 feet in real life with bumpers (side field) - back of robot
    visionMap[12] = 35.00; 
    visionMap[13] = 35.3; 
    visionMap[14] = 36.3; 
    visionMap[15] = 37.3; //  12 feet in real life with bumpers (close protected zone) - back of robot
    visionMap[16] = 37.29; 
    visionMap[17] = 40.02; 
    visionMap[18] = 39.87; 
    visionMap[19] = 40; 

    // velocity map
    // key is the distance in feet
    // assigned value is the desired flywheel velocity in rpm
    velocityMap[4] = 1100;
    velocityMap[5] = 1110; 
    velocityMap[6] = 1140; 
    velocityMap[7] = 1180; 
    velocityMap[8] = 1200; 
    velocityMap[9] =  1250; 
    velocityMap[10] =  1270;  
    velocityMap[11] = 1300; 
    velocityMap[12] =  1330; 
    velocityMap[13] = 1360; 
    velocityMap[14] = 1580; 
    velocityMap[15] = 1650; 
    velocityMap[16] = 1730; 
    velocityMap[17] = 1880; 
    velocityMap[18] = 1880; 
    velocityMap[19] = 2030;


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





