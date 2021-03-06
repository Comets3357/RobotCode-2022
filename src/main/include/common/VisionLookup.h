#pragma once
#include "Constants.h"
#include <iostream>
#include <unordered_map>
using namespace std;

struct VisionLookupData
{
    
};

class VisionLookup
{

    public:
        void RobotInit();
        void RobotPeriodic(const RobotData &robotData, VisionLookupData &visionLookupData);
        double getValue(double key);
        double getVelocity(double key);

        double highestVal();
        double lowestVal();
        double highestVelocity();
        double lowestVelocity();
        
    private:
        // Declaring umap to be of <double, double> type
        // key will be of double type and mapped value will
        // be of double type
        
        //high hub visionMap
        unordered_map<double, double> visionMap;

        //high hub velocity values
        unordered_map<double, double> velocityMap;

};