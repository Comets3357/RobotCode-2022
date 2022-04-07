#pragma once

#include "robotData.h"
#include <frc/PowerDistribution.h>

class NetworkTables
{
public:
    /*
    * @brief Defines entries for dashboard data
    */
    // Match
    nt::NetworkTableEntry autonSelect;
    nt::NetworkTableEntry timeLeft;
    // Robot
    nt::NetworkTableEntry controlMode;
    nt::NetworkTableEntry driveMode;
    nt::NetworkTableEntry robotSpeed;
    nt::NetworkTableEntry batteryVoltage;
    nt::NetworkTableEntry totalCurrent;
    // Gyro
    nt::NetworkTableEntry odometryX;
    nt::NetworkTableEntry odometryY;
    nt::NetworkTableEntry odometryYaw;
    // Drivebase
    nt::NetworkTableEntry dbInverted;
    nt::NetworkTableEntry turnResponsive;
    // Indexer
    nt::NetworkTableEntry indexerTop;
    nt::NetworkTableEntry indexerBottom;
    nt::NetworkTableEntry eBallCountZero;
    // Shooter
    nt::NetworkTableEntry readyShoot;
    nt::NetworkTableEntry shootMode;
    nt::NetworkTableEntry targetHub;
    nt::NetworkTableEntry shootUBAO;
    // Climb
    nt::NetworkTableEntry climbSequence;
    nt::NetworkTableEntry climbAmperage;

    /*
    * @brief Defines events where dashboard entries will be updated
    */
    void RobotInit();
    void TeleopPeriodic(const RobotData &robotData);

    frc::PowerDistribution PD{};
};