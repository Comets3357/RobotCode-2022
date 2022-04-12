#include "Robot.h"
#include <frc/livewindow/LiveWindow.h>

void Robot::RobotInit()
{
    gyro.RobotInit();
    jetson.RobotInit();

    auton.RobotInit(robotData.autonData);
    drivebase.RobotInit();
    intake.RobotInit();
    indexer.RobotInit();
    shooter.RobotInit(robotData.shooterData);
    climb.RobotInit();
    networkTables.RobotInit();
    arduino.RobotInit();
}

void Robot::RobotPeriodic()
{
    gyro.RobotPeriodic(robotData.gyroData);
    limelight.RobotPeriodic(robotData, robotData.limelightData, visionLookup);
    visionLookup.RobotPeriodic(robotData, robotData.visionLookupData);
    jetson.RobotPeriodic(robotData, robotData.jetsonData);
    arduino.RobotPeriodic(robotData, robotData.arduinoData);
    colorSensor.RobotPeriodic(robotData, robotData.colorSensorData);
    jetson.RobotPeriodic(robotData, robotData.jetsonData);
    networkTables.RobotPeriodic(robotData);


    // frc::SmartDashboard::PutNumber("mode", robotData.controlData.mode);

    //frc::SmartDashboard::PutNumber("mode", robotData.controlData.mode);


    if (!IsTest())
    {
        intake.RobotPeriodic(robotData, robotData.intakeData);
        otherComponents.RobotPeriodic(robotData.otherComponentsData);
        drivebase.RobotPeriodic(robotData, robotData.drivebaseData, robotData.autonData);
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
    shooter.EnabledInit(robotData.controlData, robotData.shooterData);
}

void Robot::AutonomousPeriodic()
{
    timer.EnabledPeriodic(robotData.timerData);
    auton.AutonomousPeriodic(robotData, robotData.autonData, robotData.controlData, robotData.controllerData);
    // arduino.RobotPeriodic(robotData);
}

void Robot::TeleopInit()
{
    gyro.TeleopInit(robotData.gyroData);
    drivebase.TeleopInit(robotData);
    shooter.EnabledInit(robotData.controlData, robotData.shooterData);
}

void Robot::TeleopPeriodic()
{
    timer.EnabledPeriodic(robotData.timerData);
    controller.TeleopPeriodic(robotData, robotData.controllerData, robotData.controlData);
    // arduino.RobotPeriodic(robotData, robotData.arduinoData);

}

void Robot::DisabledInit()
{
    timer.DisabledInit(robotData.timerData);
    drivebase.DisabledInit();
    intake.DisabledInit();
    indexer.DisabledInit();
    shooter.DisabledInit();
    climb.DisabledInit();
}

void Robot::DisabledPeriodic() 
{
    timer.DisabledPeriodic(robotData.timerData);
    shooter.DisabledPeriodic(robotData, robotData.shooterData);
    intake.DisabledPeriodic(robotData, robotData.intakeData);
    indexer.DisabledPeriodic(robotData, robotData.indexerData);
    arduino.DisabledPeriodic();
}


void Robot::TestInit(){
    frc::LiveWindow::SetEnabled(false); // to block their weird dashboard thing

    arduino.RobotInit();
    gyro.RobotInit();

    benchTest.TestInit(robotData.benchTestData, robotData.controlData);
    drivebase.RobotInit();
    intake.RobotInit();
    indexer.RobotInit();
    shooter.RobotInit(robotData.shooterData);
    climb.RobotInit();
    climb.TestInit(robotData.climbData);
}

//BENCH TEST CODE
void Robot::TestPeriodic(){
    //runs all of the test functions (and one controller function) so things actually run
    controller.TestPeriodic(robotData, robotData.controllerData, robotData.controlData);
    benchTest.TestPeriodic(robotData, robotData.benchTestData, robotData.controlData);

    indexer.TestPeriodic(robotData, robotData.indexerData);
    climb.TestPeriodic(robotData, robotData.climbData);
    drivebase.TestPeriodic(robotData, robotData.drivebaseData);
    intake.TestPeriodic(robotData, robotData.intakeData);
    shooter.TestPeriodic(robotData, robotData.shooterData);

    
    arduino.RobotPeriodic(robotData, robotData.arduinoData);
    colorSensor.RobotPeriodic(robotData, robotData.colorSensorData);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif