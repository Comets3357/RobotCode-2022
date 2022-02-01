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
    intakeSingulatorID = 13;

static const double
    absOut = 0.485,
    absIn = 0.618,
    revOut = 16.1,
    revIn = 0;

//indexer
static const int
    indexerBeltsID = 22,
    indexerWheelID = 21;

// shooter
static const int
    shooterWheelLeadID = 31,
    shooterWheelFollowID = 32,
    shooterHoodID = 33,
    shooterTurretID = 34;

static const double
    hoodabsOut = 0.043,
    hoodabsIn = 0.742,
    hoodrevOut = -39,
    hoodrevIn = 0;

//shooter PID constants:
static const double
    swkP = 0.02, swkI = 0, swkD = 0, swkIz = 0, swkFF = 0, swkMaxOutput = 1, swkMinOutput = -1,
    hkP = 1, hkI = 0, hkD = 0, hkIz = 0, hkFF = 0, hkMaxOutput = 1, hkMinOutput = -1;

// climb
static const int
    climbLiftID = 41;

//limelight:
static const double
    hubHeight = 104,
    limelightMount = 37,
    limelightAngle = 37.75,
    xcameraDistanceFromBot = 3.0625,
    ycameraDistanceFromBot = 9.5,
    shooterDistanceFromCenterOfBot = 2;

// DIO
static const int
    intakeAbsoluteEncoderPort = 8,
    HoodAbsoluteEncoderPort = 1,
    bottomBeamBreakPort = 3,
    midBeamBreakPort = 9,
    topBeamBreakPort = 5;





