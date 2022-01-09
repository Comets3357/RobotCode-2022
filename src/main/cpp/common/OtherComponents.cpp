#include "common/OtherComponents.h"

void OtherComponents::RobotPeriodic(OtherComponentsData &otherComponentsData)
{
    updateData(otherComponentsData);
}

void OtherComponents::updateData(OtherComponentsData &otherComponentsData)
{
    // accelerometer
    otherComponentsData.accelX = accelerometer.GetX();
    otherComponentsData.accelY = accelerometer.GetY();
    otherComponentsData.accelZ = accelerometer.GetZ();
}