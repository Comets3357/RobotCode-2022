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
    // ATTAINS INSTANCE OF NETWORK TABLE
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");
    
    // ATTAINS LEFT AND RIGHT DRIVE BASE DATA
    double leftCurrent = robotData.controlData.lDrive;
    double rightCurrent = robotData.controlData.rDrive;
    double maxCurrent = 0;
    
    double frontBack = robotData.controlData.maxStraight * (leftCurrent + rightCurrent) / 2;
    double leftRight = robotData.controlData.maxTurn * (rightCurrent - leftCurrent) / 2;

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

    if (robotData.drivebaseData.driveMode == driveMode_vector)
    {
        // GETS DISTANCE AND ANGLE OFF FROM BALL
        distanceFromBall = table->GetNumber("Distance To Closest Ball", 0);
        angleOffBall = table->GetNumber("Angle To Closest Ball", 0);

        // CHECKS TO SEE WHICH JOYSTICK HAS MORE POWER TO USE
        if (leftCurrent > rightCurrent)
        {
            maxCurrent = leftCurrent;
        }
        else
        {
            maxCurrent = rightCurrent;
        }

        //frc::SmartDashboard::PutNumber("skew", getSkew(distanceFromBall));

        // CREATES DEADZONE
        if (std::abs(maxCurrent) > 0.08)
        {
            // IF WE ARE DRIVING FORWARD, THEN THE SKEW WILL WORK
            if (maxCurrent > 0.08)
            {
                // CHECKS TO SEE IF WE ARE WITHIN A SAFE DISTANCE OF THE BALL
                if (distanceFromBall < 120)
                {
                    maxCurrent = 0.8 * maxCurrent;

                    // SKEWS DRIVE BASE BASED OFF WHERE THE BALL IS IN FRAME
                    if (angleOffBall > 2)
                    {
                        jetsonData.leftSkew = getSkew(angleOffBall, distanceFromBall) * maxCurrent;
                        jetsonData.rightSkew = maxCurrent;
                    }
                    else if (angleOffBall < -2)
                    {
                        jetsonData.leftSkew = maxCurrent;
                        jetsonData.rightSkew = getSkew(angleOffBall, distanceFromBall) * maxCurrent;
                    }
                    else
                    {
                        jetsonData.leftSkew = getSkew(angleOffBall, distanceFromBall) * maxCurrent;
                        jetsonData.rightSkew = getSkew(angleOffBall, distanceFromBall) * maxCurrent;
                    }

                    // MAKES SURE WE AREN'T JUST UNDERDRIVING ANY SIDE TO REACH BALL
                    if (jetsonData.leftSkew <= 0)
                    {
                        jetsonData.leftSkew = 0.05;
                    }

                    if (jetsonData.rightSkew <= 0)
                    {
                        jetsonData.rightSkew = 0.05;
                    }
                }
                else // IF BALL IS GREATER THAN 120 INCHES THEN ARCADE MODE IS STILL ACTIVATED
                {
                    //deadzone NOT needed for drone controller
                    if (leftCurrent <= -0.08 || leftCurrent >= 0.08)
                    {
                        jetsonData.leftSkew = (frontBack - leftRight);
                    }
                    else
                    {
                        jetsonData.leftSkew = 0;
                    }

                    if (rightCurrent <= -0.08 || rightCurrent >= 0.08)
                    {
                        jetsonData.rightSkew = (frontBack + leftRight);
                    }
                    else
                    {
                        jetsonData.rightSkew = 0;
                    }
                }
            }
            else if (maxCurrent < -0.08) // GIVES DRIVER FULL CONTROL DRIVING BACKWARDS AND ARCADE CONTROL
            {
                
                if (leftCurrent <= -0.08 || leftCurrent >= 0.08)
                {
                    jetsonData.leftSkew = (frontBack - leftRight);
                }
                else
                {
                    jetsonData.leftSkew = 0;
                }

                if (rightCurrent <= -0.08 || rightCurrent >= 0.08)
                {
                    jetsonData.rightSkew = (frontBack + leftRight);
                }
                else
                {
                    jetsonData.rightSkew = 0;
                }
            }
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
    double angleSkew = 0.000108167 * std::pow(angle, 2);
    // skew is derived from a linear equation
    double distanceSkew = (-0.004167 * distance) + 1;
    // returns a skew that is a dynamic quadratic equation based off distance and angle off from target
    return (angleSkew * distanceSkew) + 1;
}