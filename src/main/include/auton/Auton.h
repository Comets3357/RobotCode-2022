#pragma once

#include <frc/TimedRobot.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>
#include <vector>
#include <string>

#include "controller/Controller.h"


struct RobotData;

enum AutonSelect
{
    autonSelect_potato,
    autonSelect_driveStraight,
    autonSelect_intakeAlternate,
    autonSelect_barrelRace
};

struct AutonData
{
    AutonSelect autonSelect = autonSelect_barrelRace;
    frc::Trajectory trajectory;
    int autonStep = -1; // starts at -1 because getTrajectoryFile() increments
    std::vector<std::string> pathGroup;
};

class Auton
{
public:
    void AutonomousInit(AutonData &autonData);
    void AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControllerData &controllerData);
    void DisabledInit();
private:
    void potato(const RobotData &robotData, ControllerData &controllerData);
    void driveStraight(const RobotData &robotData, ControllerData &controllerData);
    void intakeAlternate(const RobotData &robotData, ControllerData &controllerData);
    void barrelRace(const RobotData &robotData, ControllerData &controllerData);


};