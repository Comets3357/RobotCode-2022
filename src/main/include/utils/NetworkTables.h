#include "robotData.h"

class NetworkTables
{
public:
    // both
    nt::NetworkTableEntry secRemaining;
    nt::NetworkTableEntry mode;
    nt::NetworkTableEntry indexerTop;
    nt::NetworkTableEntry indexerBottom;

    // primary only
    nt::NetworkTableEntry dbInverted;
    nt::NetworkTableEntry odometryX;
    nt::NetworkTableEntry odometryY;
    nt::NetworkTableEntry odometryYaw;
    nt::NetworkTableEntry doneShooting;

    // secondary only
    nt::NetworkTableEntry shootMode;
    nt::NetworkTableEntry shootUnassignedAsOpponent;
    nt::NetworkTableEntry upperHubShot;
    nt::NetworkTableEntry climbSequence;
    nt::NetworkTableEntry climbAmperage;
    nt::NetworkTableEntry flywheelUpToSpeed;
    nt::NetworkTableEntry driveMode;
    nt::NetworkTableEntry autonSelect;

    void RobotInit();
    void TeleopPeriodic(const RobotData &robotData);

    double x = 0;
    double y = 0;
};