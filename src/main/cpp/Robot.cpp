#include "Robot.h"

void Robot::RobotInit()
{
    timer.RobotInit(robotData.timerData);
    gyro.RobotInit();

    drivebase.RobotInit();
}

void Robot::RobotPeriodic()
{
    timer.RobotPeriodic(robotData.timerData);
    gyro.RobotPeriodic(robotData.gyroData);

    if (IsEnabled())
    {
        otherComponents.RobotPeriodic(robotData.otherComponentsData);

        drivebase.RobotPeriodic(robotData, robotData.drivebaseData);
    }
}

void Robot::AutonomousInit()
{
    auton.AutonomousInit(robotData.autonData);
    frc::SmartDashboard::PutString("autonomous init hello", "alksdjflaskjf");
}

void Robot::AutonomousPeriodic()
{
    auton.AutonomousPeriodic(robotData, robotData.autonData, robotData.controllerData);
}

void Robot::TeleopInit()
{
    gyro.TeleopInit();
}

void Robot::TeleopPeriodic()
{
    controller.TeleopPeriodic(robotData, robotData.controllerData, robotData.controlData);
}

void Robot::DisabledInit()
{
    timer.DisabledInit();

    drivebase.DisabledInit();
}

void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif