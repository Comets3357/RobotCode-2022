#pragma once

// drivebase:
static const int
    leftLeadDeviceID = 1,
    leftFollowDeviceID = 2,
    rightLeadDeviceID = 4,
    rightFollowDeviceID = 5;

//intake:
static const int
    intakeRollerID = 12,
    intakePivotID = 11,
    intakeMecanumID = 13;

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





