#include "common/Jetson.h"
#include "RobotData.h"

void Jetson::RobotInit()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");

    currentAlliance = 0;

    // PUTS OUT VALS INTO NETWORK TABLE TO TUNE JETSON
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
    table->PutNumber("realsense angle", 33);
    table->PutNumber("red erosion", 4); // THIS CHANGES AT COMPS
    table->PutNumber("red dilation", 8); // THIS CHANGES AT COMPS
    table->PutNumber("blue erosion", 3); // THIS CHANGES AT COMPS
    table->PutNumber("blue dilation", 5); // THIS CHANGES AT COMPS

    distanceFromBall = 0;
    angleOffBall = 0;
}

void Jetson::RobotPeriodic(const RobotData &robotData, JetsonData &jetsonData)
{
    // ATTAINS INSTANCE OF NETWORK TABLE
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");
    
    // ATTAINS LEFT AND RIGHT DRIVE BASE DATA
    double leftCurrent = robotData.controlData.lDrive;
    double rightCurrent = robotData.controlData.rDrive;
    double maxCurrent = 0;
    
    if (leftCurrent > rightCurrent)
    {
        maxCurrent = leftCurrent;
    }
    else
    {
        maxCurrent = rightCurrent;
    }

    // SETS CURRENT ALLIANCE FOR JETSON
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
       currentAlliance = 1;
    }
    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        currentAlliance = 0;
    }
    
    table->PutNumber("Alliance Jetson", currentAlliance);
    
    distanceFromBall = table->GetNumber("Distance To Closest Ball", 0);
    angleOffBall = table->GetNumber("Angle To Closest Ball", 0);

    // frc::SmartDashboard::PutNumber("skew", getSkew(angleOffBall, distanceFromBall));
    // frc::SmartDashboard::PutNumber("distance from ball", distanceFromBall);
    jetsonData.leftSkew = 0;
    jetsonData.rightSkew = 0;

    if (robotData.drivebaseData.driveMode == driveMode_vector)
    {
        if (maxCurrent > 0.08)
        {
            if (angleOffBall > 2)
            {
                jetsonData.leftSkew = maxCurrent;
                jetsonData.rightSkew = maxCurrent / getSkew(angleOffBall, distanceFromBall);
            }
            else if (angleOffBall < -2)
            {
                jetsonData.leftSkew = maxCurrent / getSkew(angleOffBall, distanceFromBall);
                jetsonData.rightSkew = maxCurrent;
            }
        }
        else if (maxCurrent < -0.08)
        {
            jetsonData.leftSkew = leftCurrent;
            jetsonData.rightSkew = rightCurrent;
        }
    }
}

// getter for if another class needs access to this information
double Jetson::getDistanceFromBall()
{
    return distanceFromBall;
}

// getter for if another class needs access to this information
double Jetson::getAngleOffBall()
{
    return angleOffBall;
}

// this skew is based off of angle and distance
double Jetson::getSkew(double angle, double distance)
{
    // skew is derived from a quadratic equation
    double angleSkew = 0.00022715 * std::pow(angle, 2);
    // skew is derived from a linear equation
    double distanceSkew = (-0.002083 * distance) + 1;
    // returns a skew that is a dynamic quadratic equation based off distance and angle off from target
    return (angleSkew * distanceSkew) + 1;
}