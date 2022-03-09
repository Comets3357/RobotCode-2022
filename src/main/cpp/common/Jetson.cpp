#include "common/Jetson.h"
#include "RobotData.h"

void Jetson::RobotInit()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");

    currentAlliance = 0;

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
    table->PutNumber("blue h max", 110); // THIS CHANGES AT COMPS
    table->PutNumber("blue s min", 180); // THIS CHANGES AT COMPS
    table->PutNumber("blue s max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("blue v min", 50); // THIS CHANGES AT COMPS
    table->PutNumber("blue v max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("red h lower min", 0); // THIS CHANGES AT COMPS
    table->PutNumber("red h lower max", 12); // THIS CHANGES AT COMPS
    table->PutNumber("red h upper min", 168); // THIS CHANGES AT COMPS
    table->PutNumber("red h upper max", 180); // THIS CHANGES AT COMPS
    table->PutNumber("red s min", 120); // THIS CHANGES AT COMPS
    table->PutNumber("red s max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("red v min", 58); // THIS CHANGES AT COMPS
    table->PutNumber("red v max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("realsense center x", 0);
    table->PutNumber("realsense center y", 0);
    table->PutNumber("realsense x fov", 87);
    table->PutNumber("realsense y fov", 58);
    table->PutNumber("ball radius", 4.75);
    table->PutNumber("realsense height", 34.5);
    table->PutNumber("realsense angle", 26);
    table->PutNumber("red erosion", 4); // THIS CHANGES AT COMPS
    table->PutNumber("red dilation", 8); // THIS CHANGES AT COMPS
    table->PutNumber("blue erosion", 3); // THIS CHANGES AT COMPS
    table->PutNumber("blue dilation", 5); // THIS CHANGES AT COMPS

    distanceFromBall = 0;
    angleOffBall = 0;
}

void Jetson::RobotPeriodic(const RobotData &robotData, JetsonData &jetsonData)
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");

    double currentAlliance = 0;
    double leftCurrent = robotData.controlData.lDrive;
    double rightCurrent = robotData.controlData.rDrive;
    double maxCurrent = 0;
        
    distanceFromBall = table->GetNumber("Distance To Closest Ball", 0);
    angleOffBall = table->GetNumber("Angle To Closest Ball", 0);

        
    if (leftCurrent > rightCurrent)
    {
        maxCurrent = leftCurrent;
    }
    else
    {
        maxCurrent = rightCurrent;
    }

    maxCurrent = 0.8 * maxCurrent;

    if (angleOffBall > 1)
    {
        jetsonData.leftSkew = getSkew(distanceFromBall) + maxCurrent;
        jetsonData.rightSkew = (- getSkew(distanceFromBall)) + maxCurrent;
    }
    else if (angleOffBall < -1)
    {
        jetsonData.leftSkew = (- getSkew(distanceFromBall)) + maxCurrent;
        jetsonData.rightSkew = getSkew(distanceFromBall) + maxCurrent;
    }
    else
    {
        jetsonData.leftSkew = getSkew(distanceFromBall) + maxCurrent;
        jetsonData.rightSkew = getSkew(distanceFromBall) + maxCurrent;
    }

    if (jetsonData.leftSkew < 0)
    {
        jetsonData.leftSkew = 0;
    }

    if (jetsonData.rightSkew < 0)
    {
        jetsonData.rightSkew = 0;
    }
}

double Jetson::getDistanceFromBall()
{
    return distanceFromBall;
}

double Jetson::getAngleOffBall()
{
    return angleOffBall;
}

double Jetson::getSkew(double distance)
{
    return (-0.001667 * (distance)) + 0.2;
}