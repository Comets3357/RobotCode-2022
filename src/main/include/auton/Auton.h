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

enum AutonSelect
{
    autonSelect_potato,
    autonSelect_driveStraight,
    autonSelect_intakeAlternate,
};

struct AutonData
{
    AutonSelect autonSelect = autonSelect_potato;
    frc::Trajectory trajectory;
    int autonStep = -1; // starts at -1 because getTrajectoryFile() increments
    std::vector<std::string> pathGroup;
    frc::Pose2d startPoint;
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
    void sendStartPointChooser();
    void potato(const RobotData &robotData, ControllerData &controllerData);
    void driveStraight(const RobotData &robotData, ControllerData &controllerData);
    void intakeAlternate(const RobotData &robotData, ControllerData &controllerData);
    frc::Pose2d getPose(double x, double y, double deg);

    frc::SendableChooser<AutonSelect> autonChooser;
    frc::SendableChooser<frc::Pose2d> startPointChooser;
};