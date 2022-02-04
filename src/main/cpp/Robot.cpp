#include "Robot.h"

void Robot::RobotInit()
{
    timer.RobotInit(robotData.timerData);
    gyro.RobotInit();
    limelight.RobotInit(robotData);

    drivebase.RobotInit();
    intake.RobotInit();
<<<<<<< Updated upstream
    indexer.RobotInit();
    shooter.RobotInit(robotData.shooterData);
    climb.RobotInit();
=======
>>>>>>> Stashed changes
}

void Robot::RobotPeriodic()
{
    timer.RobotPeriodic(robotData.timerData);
    gyro.RobotPeriodic(robotData.gyroData);
    limelight.RobotPeriodic(robotData, robotData.limelightData, visionLookup);
    colorSensor.RobotPeriodic(robotData);
    visionLookup.RobotPeriodic(robotData, robotData.visionLookupData);

    if (IsEnabled())
    {
        otherComponents.RobotPeriodic(robotData.otherComponentsData);
        drivebase.RobotPeriodic(robotData, robotData.drivebaseData);
        intake.RobotPeriodic(robotData, robotData.intakeData);
<<<<<<< Updated upstream
        indexer.RobotPeriodic(robotData, robotData.indexerData);
        shooter.RobotPeriodic(robotData, robotData.shooterData);
        climb.RobotPeriodic(robotData, robotData.climbData);
=======
>>>>>>> Stashed changes
    }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {
    
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