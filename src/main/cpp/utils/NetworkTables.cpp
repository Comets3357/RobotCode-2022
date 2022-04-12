#include "utils/NetworkTables.h"

#include <frc/DriverStation.h>
#include <frc/PowerDistribution.h>

void NetworkTables::RobotInit()
{
    /*
    * @brief Initializes network table for dashboard
    */
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("dashboard");

    /*
    * @brief Initializes entries for dashboard data
    */
    // Match
    autonSelect = table->GetEntry("autonSelect");
    timeLeft = table->GetEntry("timeLeft");
    // Robot
    controlMode = table->GetEntry("controlMode");
    driveMode = table->GetEntry("driveMode");
    robotSpeed = table->GetEntry("robotSpeed");
    batteryVoltage = table->GetEntry("batteryVoltage");
    // totalCurrent = table->GetEntry("totalCurrent");
    // Gyro
    odometryX = table->GetEntry("odometryX");
    odometryY = table->GetEntry("odometryY");
    odometryYaw = table->GetEntry("odometryYaw");
    // Drivebase
    dbInverted = table->GetEntry("dbInverted");
    turnResponsive = table->GetEntry("turnResponsive");
    avgDriveVel = table->GetEntry("avgDriveVel");
    // Indexer
    indexerTop = table->GetEntry("indexerTop");
    indexerBottom = table->GetEntry("indexerBottom");
    eBallCountZero = table->GetEntry("eBallCountZero");
    // Shooter
    readyShoot = table->GetEntry("readyShoot");
    shootMode = table->GetEntry("shootMode");
    targetHub = table->GetEntry("targetHub");
    autoReject = table->GetEntry("autoReject");
    shotDistanceTrim = table->GetEntry("shotDistanceTrim");
    validTarget = table->GetEntry("validTarget");
    // Climb
    climbSequence = table->GetEntry("climbSequence");
    climbAmperage = table->GetEntry("climbAmperage");
    // Other
    ballCount = table->GetEntry("ballCount");
}


void NetworkTables::RobotPeriodic(const RobotData &robotData)
{
    /*
    * @brief Updates dashboard network table entries to varaiables from robot code
    */
    // Match
    autonSelect.SetDouble(0.0);
    timeLeft.SetDouble(/* 215 - robotData.timerData.secSinceEnabled */ frc::DriverStation::GetMatchTime());
    // Robot
    controlMode.SetDouble(robotData.controlData.mode);
    driveMode.SetDouble(robotData.drivebaseData.driveMode);
    robotSpeed.SetDouble(0.0);
    batteryVoltage.SetDouble(frc::DriverStation::GetBatteryVoltage());
    // totalCurrent.SetDouble(PD.GetTotalCurrent());
    // Gyro
    odometryX.SetDouble(robotData.drivebaseData.currentPose.X().to<double>());
    odometryY.SetDouble(robotData.drivebaseData.currentPose.Y().to<double>());
    odometryYaw.SetDouble(robotData.drivebaseData.currentPose.Rotation().Radians().to<double>());
    // Drivebase
    dbInverted.SetDouble(robotData.controlData.dbInverted);
    turnResponsive.SetDouble(robotData.controlData.turnResponsive);
    avgDriveVel.SetDouble(robotData.drivebaseData.avgDriveVel);
    // Indexer
    indexerTop.SetDouble(robotData.indexerData.topBeamBreak);
    indexerBottom.SetDouble(robotData.indexerData.midBeamBreak);
    eBallCountZero.SetDouble(robotData.indexerData.eBallCountZero);
    // Shooter
    readyShoot.SetDouble(robotData.shooterData.readyShoot);
    shootMode.SetDouble(robotData.controlData.shootMode);
    targetHub.SetDouble(robotData.controlData.upperHubShot);
    autoReject.SetDouble(robotData.controlData.autoRejectOpponentCargo);
    shotDistanceTrim.SetDouble(robotData.controlData.saDistanceOffset);
    validTarget.SetBoolean(robotData.limelightData.validTarget || (robotData.controlData.shootMode != shootMode_vision));
    // Climb
    climbSequence.SetDouble(0.0);
    climbAmperage.SetDouble(0.0);
    // Other
    // ballCount.SetDouble(robotData.jetsonData.ballCount);
}