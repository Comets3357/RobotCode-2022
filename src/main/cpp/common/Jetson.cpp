#include "common/Jetson.h"
#include "RobotData.h"

void Jetson::RobotInit()
{
    currentAlliance = 0;
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");

    currentAlliance = 0;

    // PUTS OUT VALS INTO NETWORK TABLE TO TUNE JETSON
    table->PutNumber("blue h min", 90); // THIS CHANGES AT COMPS
    table->PutNumber("blue h max", 105); // THIS CHANGES AT COMPS
    table->PutNumber("blue s min", 120); // THIS CHANGES AT COMPS
    table->PutNumber("blue s max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("blue v min", 50); // THIS CHANGES AT COMPS
    table->PutNumber("blue v max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("red h lower min", 0); // THIS CHANGES AT COMPS
    table->PutNumber("red h lower max", 9); // THIS CHANGES AT COMPS
    table->PutNumber("red h upper min", 175); // THIS CHANGES AT COMPS
    table->PutNumber("red h upper max", 180); // THIS CHANGES AT COMPS
    table->PutNumber("red s min", 160); // THIS CHANGES AT COMPS
    table->PutNumber("red s max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("red v min", 90); // THIS CHANGES AT COMPS
    table->PutNumber("red v max", 255); // THIS CHANGES AT COMPS
    table->PutNumber("realsense center x", 0);
    table->PutNumber("realsense center y", 0);
    table->PutNumber("realsense x fov", 87);
    table->PutNumber("realsense y fov", 58);
    table->PutNumber("ball radius", 4.75);
    table->PutNumber("realsense height", 34.5);
    table->PutNumber("red erosion", 6); // THIS CHANGES AT COMPS
    table->PutNumber("red dilation", 6); // THIS CHANGES AT COMPS
    table->PutNumber("blue erosion", 2); // THIS CHANGES AT COMPS
    table->PutNumber("blue dilation", 3); // THIS CHANGES AT COMPS
    table->PutNumber("realsense angle", 33);
}

void Jetson::RobotPeriodic(const RobotData &robotData, JetsonData &jetsonData)
{
    // ATTAINS INSTANCE OF NETWORK TABLE
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("default");

    // ATTAINS DISTANCE AND ANGLE FROM BALL    
    jetsonData.distanceFromBall = table->GetNumber("Distance To Closest Ball", 0);
    jetsonData.angleOffBall = table->GetNumber("Angle To Closest Ball", 0);
    jetsonData.ballCount = table->GetNumber("ball count", 0);
    
    // frc::SmartDashboard::PutNumber("BALLLS", jetsonData.distanceFromBall);
    // frc::SmartDashboard::PutNumber("HEHEHEHEHEHEHEHEHEHEH", jetsonData.angleOffBall);
    

    // ATTAINS LEFT AND RIGHT DRIVE BASE DATA
    double leftCurrent = robotData.controlData.lDrive;
    double rightCurrent = robotData.controlData.rDrive;
    double maxCurrent = 0;

    // frc::SmartDashboard::PutNumber("left drive12", leftCurrent);
    // frc::SmartDashboard::PutNumber("right drive12", rightCurrent);

    jetsonData.leftSkew = 0;
    jetsonData.rightSkew = 0;

    // SETS WHAT ALLIANCE WE ARE ON AND PUTS THAT OUT TO JETSON
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
       currentAlliance = 1;
    }
    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        currentAlliance = 0;
    }
    
    table->PutNumber("Alliance Jetson", currentAlliance);

    // IF IN VECTOR MODE GO INTO SKEW DB CONTROL
    if (robotData.drivebaseData.driveMode == driveMode_vector)
    {
        // IF THE AVERAGE OF THE TWO SIDE VELOCITIES IS GREATER THAN 0 THEN PROCEED
        //if (((leftCurrent + rightCurrent) / 2) > 0)
        //{
            // SETS MAXCURRENT TO THE GREATER JOYSTICK VAL
            if (jetsonData.ballCount > 0)
            {
                if (jetsonData.distanceFromBall < 120)
                {
                    if (rightCurrent > leftCurrent)
                    {
                        maxCurrent = rightCurrent;
                    }
                    else
                    {
                        maxCurrent = leftCurrent;
                    }

                    //maxCurrent = rightCurrent;

                    // IF THE MAX CURRENT IS ABOVE DEADZONE THEN GOES INTO ACTUAL SKEW
                    if (maxCurrent > 0.08)
                    {
                        
                        // CREATES DEADZONE OF 4 DEGREES AND THEN SKEWS DB TO THE BALL BASED OFF THE BALLS DISTANCE AND ANGLE OFF 
                        if (jetsonData.angleOffBall > 2)
                        {
                            jetsonData.leftSkew = maxCurrent;
                            jetsonData.rightSkew = maxCurrent / getSkew(jetsonData.angleOffBall, jetsonData.distanceFromBall);
                        }
                        else if (jetsonData.angleOffBall < -2)
                        {
                            jetsonData.leftSkew = maxCurrent / getSkew(jetsonData.angleOffBall, jetsonData.distanceFromBall);
                            jetsonData.rightSkew = maxCurrent;
                        }
                        else
                        {
                            jetsonData.leftSkew = leftCurrent;
                            jetsonData.rightSkew = rightCurrent;
                        }
                    }
                    else if (maxCurrent < -0.08)
                    {
                        jetsonData.leftSkew = leftCurrent;
                        jetsonData.rightSkew = rightCurrent;
                    }
                }
                else
                {
                    jetsonData.leftSkew = leftCurrent;
                    jetsonData.rightSkew = rightCurrent;   
                }
            }
            else
            {
                jetsonData.leftSkew = leftCurrent;
                jetsonData.rightSkew = rightCurrent;
            }
            // }
            // else if (maxCurrent < -0.08) // IF MAX CURRENT IS BELOW DEADZONE THEN JUST SET VELOCITES TO RIGHT AND LEFT TO ALLOW FOR DRIVING BACKWARDS
            // {
            //     jetsonData.leftSkew = leftCurrent;
            //     jetsonData.rightSkew = rightCurrent;
            // }
        //}
        // else // ELSE GO RETURN THE RIGHT AND LEFT VELOCITIES TO ALLOW TURNING
        // {
        //     jetsonData.leftSkew = leftCurrent;
        //     jetsonData.rightSkew = rightCurrent;    
        // }
    }
    // frc::SmartDashboard::PutNumber("LEFT SKEW", jetsonData.leftSkew);
    // frc::SmartDashboard::PutNumber("RIGHT SKEW", jetsonData.rightSkew);

}

// SKEW BASED OFF DISTANCE AND ANGLE
double Jetson::getSkew(double angle, double distance)
{
    // ANGLE SKEW DERIVED FROM QUADRATIC EQUATION
    double angleSkew = 0.00130715 * std::pow(angle, 2);
    // DISTANCE SKEW DERIVED FROM DECLINING LINEAR EQUATION
    double distanceSkew = (-0.002083 * distance) + 1;
    // RETURNS THE DISTANCE AND ANGLE SKEW BASED OFF THE SELECT POSITION OF THE BALL
    // DYNAMIC SKEWING TO ALLOW FOR SPECIFIC BALL POSITION
    return (angleSkew * distanceSkew) + 1;
}