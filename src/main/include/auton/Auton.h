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
    int autonStep = -1; // starts at -1 because getNextAutonStep() increments
    std::vector<std::string> pathGroup;
};

class Auton
{
public:
    void RobotInit(AutonData &autonData);
    void AutonomousInit(AutonData &autonData);
    void AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControlData &controlData);
    void DisabledInit();
private:
    void sendAutonSelectionChooser();

    frc::SendableChooser<std::string> autonChooser;


    // secondary controls:
    void potato(const RobotData &robotData, ControlData &controlData);

    // exit tarmac, collect 1, turn, shoot 2:
    void taxiShoot(const RobotData &robotData, ControlData &controlData);

    // three ball autons (taxiShoot + terminal shoot):
    void threeBallB(const RobotData &robotData, ControlData &controlData);
    void threeBallC(const RobotData &robotData, ControlData &controlData);

    // four ball autons (taxiShoot, neighboring ball, termincal shoot):
    void fourBallC(const RobotData &robotData, ControlData &controlData);
    void fourBallCHP(const RobotData &robotData, ControlData &controlData);

    void sixBallC(const RobotData &robotData, ControlData &controlData);
};