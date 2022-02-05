#pragma once

#include <frc/TimedRobot.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>
#include <vector>
#include <string>
#include <frc/smartdashboard/SendableChooser.h>

#include "controller/Controller.h"


struct RobotData;

struct AutonData
{
    std::string autonRoutineName;
    frc::Trajectory trajectory;
    int autonStep = -1; // starts at -1 because getNextTrajectoryStep() increments
    std::vector<std::string> pathGroup;
};

class Auton
{
public:
    void RobotInit(AutonData &autonData);
    void AutonomousInit(AutonData &autonData);
    void AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControllerData &controllerData);
    void DisabledInit();
private:
    void sendAutonSelectionChooser();

    frc::SendableChooser<std::string> autonChooser;

    // auton routine secondary control functions:
    void potato(const RobotData &robotData, ControllerData &controllerData);

    frc::Pose2d getPose(double x, double y, double deg);
};