#pragma once

#include <frc/BuiltInAccelerometer.h>
//#include <frc/Compressor.h>
//#include <frc/PowerDistribution.h>

struct OtherComponentsData
{
    // accelerometer
    double accelX, accelY, accelZ;
};

class OtherComponents
{

public:
    void RobotPeriodic(OtherComponentsData &otherComponentsData);

    void updateData(OtherComponentsData &otherComponentsData);

private:
    //accelerometer
    frc::BuiltInAccelerometer accelerometer{};

    // // compressor
    // frc::Compressor compressor{0};
    // // PDP
    // frc::PowerDistributionPanel pdp{0};
};