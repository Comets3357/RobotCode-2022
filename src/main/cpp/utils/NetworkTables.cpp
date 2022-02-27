#include "utils/NetworkTables.h"

void NetworkTables::RobotInit()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("dashboard");

    // both
    secRemaining = table->GetEntry("secRemaining");
    mode = table->GetEntry("mode");
    indexerTop = table->GetEntry("indexerTop");
    indexerBottom = table->GetEntry("indexerBottom");
    eBallCountZero = table->GetEntry("eBallCountZero");

    // primary only
    dbInverted = table->GetEntry("dbInverted");
    odometryX = table->GetEntry("odometryX");
    odometryY = table->GetEntry("odometryY");
    odometryYaw = table->GetEntry("odometryYaw");

    // secondary only
    shootMode = table->GetEntry("shootMode");
    shootUnassignedAsOpponent = table->GetEntry("shootUnassignedAsOpponent");
    upperHubShot = table->GetEntry("upperHubShot");
    climbSequence = table->GetEntry("climbSequence");
    climbAmperage = table->GetEntry("climbAmperage");
    readyShoot = table->GetEntry("readyShoot");
    driveMode = table->GetEntry("driveMode");
    autonSelect = table->GetEntry("autonSelect");
}

void NetworkTables::TeleopPeriodic(const RobotData &robotData)
{
    // both
    secRemaining.SetDouble(robotData.timerData.secRemaining);
    mode.SetDouble(robotData.controlData.mode);
    indexerTop.SetDouble(robotData.indexerData.topIndexer);
    indexerBottom.SetDouble(robotData.indexerData.bottomIndexer);
    // eBallCountZero.SetBoolean(robotData.indexerData.eBallCountZero);

    // primary only
    dbInverted.SetBoolean(robotData.controlData.dbInverted);
    odometryX.SetDouble(robotData.drivebaseData.currentPose.X().to<double>());
    odometryY.SetDouble(robotData.drivebaseData.currentPose.Y().to<double>());
    odometryYaw.SetDouble(robotData.drivebaseData.currentPose.Rotation().Radians().to<double>());
    
    // secondary only
    shootMode.SetDouble(robotData.controlData.shootMode);
    shootUnassignedAsOpponent.SetBoolean(robotData.shooterData.shootUnassignedAsOpponent);
    // upperHubShot.SetBoolean();
    // climbSequence.SetBoolean();
    // climbAmperage.SetBoolean();
    readyShoot.SetBoolean(robotData.shooterData.readyShoot);
    driveMode.SetBoolean(robotData.drivebaseData.driveMode);
}