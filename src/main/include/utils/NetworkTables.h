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
    // nt::NetworkTableEntry totalCurrent;
    // Gyro
    nt::NetworkTableEntry odometryX;
    nt::NetworkTableEntry odometryY;
    nt::NetworkTableEntry odometryYaw;
    // Drivebase
    nt::NetworkTableEntry dbInverted;
    nt::NetworkTableEntry turnResponsive;
    nt::NetworkTableEntry avgDriveVel;
    // Indexer
    nt::NetworkTableEntry indexerTop;
    nt::NetworkTableEntry indexerBottom;
    nt::NetworkTableEntry eBallCountZero;
    // Shooter
    nt::NetworkTableEntry readyShoot;
    nt::NetworkTableEntry shootMode;
    nt::NetworkTableEntry targetHub;
    nt::NetworkTableEntry autoReject;
    nt::NetworkTableEntry shotDistanceTrim;
    nt::NetworkTableEntry validTarget;
    // Climb
    nt::NetworkTableEntry climbSequence;
    nt::NetworkTableEntry climbAmperage;
    // Other
    nt::NetworkTableEntry ballCount;    // balls visible to RealSense

    /*
    * @brief Defines events where dashboard entries will be updated
    */
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData);

    frc::PowerDistribution PD{};
};