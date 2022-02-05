
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
    // inserting values by using [] operator
    visionMap[4] = 25.3;
    visionMap[5] = 27.3;
    visionMap[6] = 28.8;
    visionMap[7] = 30.6;
    visionMap[8] = 31.9;
    visionMap[9] = 34.7;
    visionMap[10] = 35.1;
    visionMap[11] = 37.25;
    visionMap[12] = 37.7;
    visionMap[13] = 39;
    visionMap[14] = 39.4;
    // visionMap[15] = ;
    // visionMap[16] = ;
    // visionMap[17] = ;
    // visionMap[18] = ;

    // velocity map
    velocityMap[4] = 1450;
    velocityMap[5] = 1500;
    velocityMap[6] = 1600;
    velocityMap[7] = 1650;
    velocityMap[8] = 1750;
    velocityMap[9] = 1750;
    velocityMap[10] = 1775;
    velocityMap[11] = 1850;
    velocityMap[12] = 1950;
    velocityMap[13] = 2000;
    velocityMap[14] = 2100;
    velocityMap[15] = 2300;
}

double VisionLookup::getValue(double key){

    std::unordered_map<double,double>::const_iterator got = visionMap.find(key);
    if (got == visionMap.end())
        return 0;
    else
        return got->second;
}

double VisionLookup::getVelocity(double key){
    std::unordered_map<double,double>::const_iterator got = velocityMap.find(key);
    if (got == velocityMap.end())
        return 0;
    else
        return got->second;

}

double VisionLookup::highestVal(){
    double highest = 0;
    for (auto x : visionMap){
        if(x.first > highest){
            highest = x.first;
        }
    }
    return highest;
}

double VisionLookup::highestVelocity(){
    double highest = 0;
    for (auto x : velocityMap){
        if(x.first > highest){
            highest = x.first;
        }
    }
    return highest;
}


