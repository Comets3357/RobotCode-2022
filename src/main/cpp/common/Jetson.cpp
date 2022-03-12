#include "common/Jetson.h"

void Jetson::RobotInit()
{
    currentAlliance = 0;
}

void Jetson::RobotPeriodic()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
       currentAlliance = 1;
    }
    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        currentAlliance = 0;
    }

    table->PutNumber("Alliance Jetson", currentAlliance);
    table->PutNumber("blue h min", 90); // THIS CHANGES AT COMPS
    table->PutNumber("blue h max", 105); // THIS CHANGES AT COMPS
    table->PutNumber("blue s min", 120); // THIS CHANGES AT COMPS
    table->PutNumber("blue s max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("blue v min", 60); // THIS CHANGES AT COMPS
    table->PutNumber("blue v max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("red h lower min", 0); // THIS CHANGES AT COMPS
    table->PutNumber("red h lower max", 15); // THIS CHANGES AT COMPS
    table->PutNumber("red h upper min", 170); // THIS CHANGES AT COMPS
    table->PutNumber("red h upper max", 180); // THIS CHANGES AT COMPS
    table->PutNumber("red s min", 140); // THIS CHANGES AT COMPS
    table->PutNumber("red s max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("red v min", 0); // THIS CHANGES AT COMPS
    table->PutNumber("red v max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("realsense center x", 0);
    table->PutNumber("realsense center y", 0);
    table->PutNumber("realsense x fov", 87);
    table->PutNumber("realsense y fov", 58);
    table->PutNumber("ball radius", 4.75);
    table->PutNumber("realsense height", 34.5);
    table->PutNumber("realsense angle", 26);
    table->PutNumber("red erosion", 0); // THIS CHANGES AT COMPS
    table->PutNumber("red dilation", 1); // THIS CHANGES AT COMPS
    table->PutNumber("blue erosion", 4); // THIS CHANGES AT COMPS
    table->PutNumber("blue dilation", 5); // THIS CHANGES AT COMPS
}