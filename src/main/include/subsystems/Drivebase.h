#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <wpi/uv/Error.h>




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

    // dbL and dbR are lead motors, LFs and RFs are following motors
    ctre::phoenix::motorcontrol::can::TalonFX dbL{leftLeadDeviceID};
    ctre::phoenix::motorcontrol::can::TalonFX dbLF{leftFollowDeviceID};

    ctre::phoenix::motorcontrol::can::TalonFX dbR{rightLeadDeviceID};
    ctre::phoenix::motorcontrol::can::TalonFX dbRF{rightFollowDeviceID};


};