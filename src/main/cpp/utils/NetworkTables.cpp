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
    totalCurrent = table->GetEntry("totalCurrent");
    // Gyro
    odometryX = table->GetEntry("odometryX");
    odometryY = table->GetEntry("odometryY");
    odometryYaw = table->GetEntry("odometryYaw");
    // Drivebase
    dbInverted = table->GetEntry("dbInverted");
    turnResponsive = table->GetEntry("turnResponsive");
    // Indexer
    indexerTop = table->GetEntry("indexerTop");
    indexerBottom = table->GetEntry("indexerBottom");
    eBallCountZero = table->GetEntry("eBallCountZero");
    // Shooter
    readyShoot = table->GetEntry("readyShoot");
    shootMode = table->GetEntry("shootMode");
    targetHub = table->GetEntry("targetHub");
    shootUBAO = table->GetEntry("shootUBAO");
    // Climb
    climbSequence = table->GetEntry("climbSequence");
    climbAmperage = table->GetEntry("climbAmperage");
}


void NetworkTables::TeleopPeriodic(const RobotData &robotData)
{
    /*
    * @brief Updates dashboard network table entries to varaiables from robot code
    */
    // Match
    autonSelect.SetDouble(0.0);
    timeLeft.SetDouble(robotData.timerData.secRemaining);
    // Robot
    controlMode.SetDouble(robotData.controlData.mode);
    driveMode.SetDouble(robotData.drivebaseData.driveMode);
    robotSpeed.SetDouble(0.0);
    batteryVoltage.SetDouble(frc::DriverStation::GetBatteryVoltage());
    totalCurrent.SetDouble(PD.GetTotalCurrent());
    // Gyro
    odometryX.SetDouble(robotData.drivebaseData.currentPose.X().to<double>());
    odometryY.SetDouble(robotData.drivebaseData.currentPose.Y().to<double>());
    odometryYaw.SetDouble(robotData.drivebaseData.currentPose.Rotation().Radians().to<double>());
    // Drivebase
    dbInverted.SetDouble(robotData.controlData.dbInverted);
    turnResponsive.SetDouble(robotData.controlData.turnResponsive);
    // Indexer
    indexerTop.SetDouble(robotData.indexerData.topIndexer);
    indexerBottom.SetDouble(robotData.indexerData.bottomIndexer);
    eBallCountZero.SetDouble(robotData.indexerData.eBallCountZero);
    // Shooter
    readyShoot.SetDouble(robotData.shooterData.readyShoot);
    shootMode.SetDouble(robotData.controlData.shootMode);
    targetHub.SetDouble(frc::DriverStation::GetBatteryVoltage());
    shootUBAO.SetDouble(robotData.shooterData.shootUnassignedAsOpponent);
    
    // Climb
    climbSequence.SetDouble(0.0);
    climbAmperage.SetDouble(0.0);
}