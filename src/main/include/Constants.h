#pragma once

// drivebase:
static const int
    leftLeadDeviceID = 1,
    leftFollowDeviceID = 2,
    rightLeadDeviceID = 4,
    rightFollowDeviceID = 5;

//intake:
static const int
    intakeRollerID = 11,
    intakePivotID = 12,
    intakeMecanumID = 13;

//intake PID constants:
static const double
    pkP = 0.02, pkI = 0, pkD = 0, pkIz = 0, pkFF = 0, pkMaxOutput = 1, pkMinOutput = -1,
    wkP = 1, wkI = 0, wkD = 0, wkIz = 0, wkFF = 0, wkMaxOutput = 1, wkMinOutput = -1,
    mkP = 1, mkI = 0, mkD = 0, mkIz = 0, mkFF = 0, mkMaxOutput = 1, mkMinOutput = -1;

//indexer
static const int
    indexerBeltsID = 15;

// shooter
static const int
    shooterWheelLeadID = 21,
    shooterWheelFollowID = 22,
    shooterHoodID = 23,
    shooterKickID = 24,
    shooterTurretID = 25;

// climb
static const int
    climbLiftID = 32;





