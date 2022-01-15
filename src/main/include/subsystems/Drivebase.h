#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>

#include <string>
#include <fstream>



struct RobotData;

struct DrivebaseData
{
    double currentLDBPos;
    double currentRDBPos;

    double lDriveVel;
    double rDriveVel;
};

class Drivebase
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData);
    void DisabledInit();

private:

    void updateData(const RobotData &robotData, DrivebaseData &drivebaseData);
    void teleopControl(const RobotData &robotData);

    // forwards are leads
    ctre::phoenix::motorcontrol::can::TalonFX dbLF{1};
    ctre::phoenix::motorcontrol::can::TalonFX dbLC{2};
    ctre::phoenix::motorcontrol::can::TalonFX dbLB{3};
    ctre::phoenix::motorcontrol::can::TalonFX dbRF{4};
    ctre::phoenix::motorcontrol::can::TalonFX dbRC{5};
    ctre::phoenix::motorcontrol::can::TalonFX dbRB{6};

    



};