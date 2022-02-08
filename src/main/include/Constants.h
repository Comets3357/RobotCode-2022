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
    absIn = 0.616,
    revOut = 6.4,
    revIn = 0;

//indexer
static const int
    indexerBeltsID = 22,
    indexerWheelID = 21;

// shooter
static const int
    shooterWheelLeadID = 32,
    shooterHoodID = 33,
    shooterTurretID = 34;

static const double
    hoodabsOut = 0.028,
    hoodabsIn = 0.933,
    hoodrevOut = -37,
    hoodrevIn = 0,
    hoodAngleOut = 43,
    hoodAngleIn = 21;

// climb
static const int
    climbLiftID = 41;

//limelight:
static const double
    hubHeight = 104,
    limelightMount = 36.5,
    limelightAngle = 36.3,
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





