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
        double highestVal();
        
        // Declaring umap to be of <double, double> type
        // key will be of double type and mapped value will
        // be of double type
        unordered_map<double, double> visionMap;

        
    private:


};