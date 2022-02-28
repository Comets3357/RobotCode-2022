#include "common/Jetson.h"

void Jetson::RobotInit()
{
    currentAlliance = 0;
    driverStation = driverStation.GetInstance();
}

void Jetson::RobotPeriodic()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");

    if (driverStation.GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
       currentAlliance = 1;
    }
    else if (driverStation.GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        currentAlliance = 0;
    }

    table->PutNumber("Alliance Jetson", currentAlliance);
}