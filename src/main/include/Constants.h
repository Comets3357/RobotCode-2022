#pragma once

// drivebase:
static const int
    leftLeadDeviceID = 1,
    leftFollowDeviceID = 2,
    rightLeadDeviceID = 3,
    rightFollowDeviceID = 4;

//intake:
static const int
    intakeRollerID = 10,
    intakePivotID = 11,
    intakeMecanumID = 12;

//intake PID constants:
static const double
    pkP = 0.02, pkI = 0, pkD = 0, pkIz = 0, pkFF = 0, pkMaxOutput = 1, pkMinOutput = -1,
    wkP = 1, wkI = 0, wkD = 0, wkIz = 0, wkFF = 0, wkMaxOutput = 1, wkMinOutput = -1,
    mkP = 1, mkI = 0, mkD = 0, mkIz = 0, mkFF = 0, mkMaxOutput = 1, mkMinOutput = -1;

//indexer
static const int
    indexerBeltsID = 21,
    indexerWheelID = 22;

// shooter
static const int
    shooterWheelLeadID = 31,
    shooterWheelFollowID = 32,
    shooterHoodID = 33,
    shooterTurretID = 34;

//shooter PID constants:
static const double
    swkP = 0.02, swkI = 0, swkD = 0, swkIz = 0, swkFF = 0, swkMaxOutput = 1, swkMinOutput = -1,
    hkP = 1, hkI = 0, hkD = 0, hkIz = 0, hkFF = 0, hkMaxOutput = 1, hkMinOutput = -1;

// climb
static const int
    climbLiftID = 41;





