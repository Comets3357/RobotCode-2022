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

    // primary only
    dbInverted = table->GetEntry("dbInverted");
    odometryX = table->GetEntry("odometryX");
    odometryY = table->GetEntry("odometryY");
    odometryYaw = table->GetEntry("odometryYaw");
    doneShooting = table->GetEntry("doneShooting");

    // secondary only
    shootMode = table->GetEntry("shootMode");
    shootUnassignedAsOpponent = table->GetEntry("shootUnassignedAsOpponent");
    upperHubShot = table->GetEntry("upperHubShot");
    climbSequence = table->GetEntry("climbSequence");
    climbAmperage = table->GetEntry("climbAmperage");
    flywheelUpToSpeed = table->GetEntry("flywheelUpToSpeed");
    driveMode = table->GetEntry("driveMode");
    autonSelect = table->GetEntry("autonSelect");
}

void NetworkTables::TeleopPeriodic(const RobotData &robotData)
{
    // both
    secRemaining.SetDouble(robotData.timerData.secRemaining);
    mode.SetDouble(robotData.controlData.mode);
    indexerTop.SetDouble(0);
    indexerBottom.SetDouble(0);

    // primary only
    dbInverted.SetBoolean(robotData.controlData.dbInverted);
    odometryX.SetDouble(robotData.drivebaseData.currentPose.X().to<double>());
    odometryY.SetDouble(robotData.drivebaseData.currentPose.Y().to<double>());
    odometryYaw.SetDouble(robotData.drivebaseData.currentPose.Rotation().Radians().to<double>());
    doneShooting.SetBoolean(robotData.shooterData.doneShooting);
    
    // secondary only
    shootMode.SetDouble(robotData.controlData.shootMode);
    shootUnassignedAsOpponent.SetBoolean(robotData.shooterData.shootUnassignedAsOpponent);
    // upperHubShot.SetBoolean();
    // climbSequence.SetBoolean();
    // climbAmperage.SetBoolean();
    // flywheelUpToSpeed.SetBoolean();
    driveMode.SetBoolean(robotData.drivebaseData.driveMode);
}