#include "Robot.h"

void Robot::RobotInit()
{
    gyro.RobotInit();

    auton.RobotInit(robotData.autonData);
    drivebase.RobotInit();
    intake.RobotInit();
    indexer.RobotInit();
    shooter.RobotInit();
    climb.RobotInit();
}

void Robot::RobotPeriodic()
{
    gyro.RobotPeriodic(robotData.gyroData);
    limelight.RobotPeriodic(robotData, robotData.limelightData, visionLookup);
    colorSensor.RobotPeriodic(robotData);
    visionLookup.RobotPeriodic(robotData, robotData.visionLookupData);
    // LED.RobotPeriodic(robotData);

    frc::SmartDashboard::PutNumber("mode", robotData.controlData.mode);

    //frc::SmartDashboard::PutNumber("mode", robotData.controlData.mode);


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
    indexer.AutonomousInit(robotData.indexerData);
    shooter.EnabledInit(robotData.shooterData);
}

void Robot::AutonomousPeriodic()
{
    timer.EnabledPeriodic(robotData.timerData);
    auton.AutonomousPeriodic(robotData, robotData.autonData, robotData.controlData);
}

void Robot::TeleopInit()
{
    gyro.TeleopInit(robotData.gyroData);
    drivebase.TeleopInit(robotData);
    shooter.EnabledInit(robotData.shooterData);
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
    indexer.DisabledPeriodic(robotData, robotData.indexerData);
}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif