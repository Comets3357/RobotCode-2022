#include "Robot.h"

void Robot::RobotInit()
{
    gyro.RobotInit();
    limelight.RobotInit(robotData);

    auton.RobotInit(robotData.autonData);
    drivebase.RobotInit();
    intake.RobotInit();
    indexer.RobotInit();
    shooter.RobotInit(robotData.shooterData);
    climb.RobotInit();
}

void Robot::RobotPeriodic()
{
    gyro.RobotPeriodic(robotData.gyroData);
    limelight.RobotPeriodic(robotData, robotData.limelightData, visionLookup);
    colorSensor.RobotPeriodic(robotData);
    visionLookup.RobotPeriodic(robotData, robotData.visionLookupData);

    if (IsEnabled())
    {
        otherComponents.RobotPeriodic(robotData.otherComponentsData);
        drivebase.RobotPeriodic(robotData, robotData.drivebaseData, robotData.autonData);
        intake.RobotPeriodic(robotData, robotData.intakeData);
        indexer.RobotPeriodic(robotData, robotData.indexerData);
        shooter.RobotPeriodic(robotData, robotData.shooterData);
        climb.RobotPeriodic(robotData, robotData.climbData);
    }
}

void Robot::AutonomousInit()
{
    timer.EnabledInit(robotData.timerData);
    gyro.AutonomousInit(robotData.gyroData);
    auton.AutonomousInit(robotData.autonData);
    drivebase.AutonomousInit(robotData, robotData.drivebaseData, robotData.autonData);
    
}

void Robot::AutonomousPeriodic()
{
    timer.EnabledPeriodic(robotData.timerData);
    auton.AutonomousPeriodic(robotData, robotData.autonData, robotData.controlData);
}

void Robot::TeleopInit()
{
    timer.EnabledInit(robotData.timerData);
    gyro.TeleopInit(robotData.gyroData);
    drivebase.TeleopInit(robotData);
    
}

void Robot::TeleopPeriodic()
{
    timer.EnabledPeriodic(robotData.timerData);
    controller.TeleopPeriodic(robotData, robotData.controllerData, robotData.controlData);
}

void Robot::DisabledInit()
{
    drivebase.DisabledInit();
    intake.DisabledInit();
    indexer.DisabledInit();
    shooter.DisabledInit();
    climb.DisabledInit();
}

void Robot::DisabledPeriodic() 
{
    shooter.updateData(robotData, robotData.shooterData);
    intake.updateData(robotData, robotData.intakeData);
    indexer.updateData(robotData, robotData.indexerData);
}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif