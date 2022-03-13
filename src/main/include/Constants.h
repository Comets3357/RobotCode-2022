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
    //practice bot
    absOut = 0.485,
    absIn = 0.616,
    revOut = 6.4,
    revIn = 0;

    //comp bot intake pivot positions
    // absOut = 0.465,
    // absIn = 0.605,
    // revOut = 6.14,
    // revIn = 0;

//indexer
static const int
    indexerBeltsID = 22,
    indexerWheelID = 21;

// shooter
static const int
    shooterWheelLeadID = 31,
    shooterHoodID = 32,
    shooterTurretID = 34,
    hoodRollerID = 33;

static const double
    //comp bot shooter hood positions
    hoodabsOut = 0.028,
    hoodabsIn = 0.90,
    hoodrevOut = -37,
    hoodrevIn = 0,
    hoodAngleOut = 43,
    hoodAngleIn = 21, 

    turretZeroRev = 0,
    turretFullRotationRev = 400;

// climb
static const int
    climbElevatorID = 41,
    climbArmsID = 43;
    
const float 
    climbArmsZero = 0.811;

//limelight:
static const double
    pi = 3.141592653589793238463,
    hubHeight = 104,
    limelightMount = 36.5, //height of mount
    limelightAngle = 36.2, //angle of limelight on mount
    xcameraDistanceFromBot = 3.0625, //offset from center of shooter X
    ycameraDistanceFromBot = 9.5, //offset from center of shooter Y
    shooterDistanceFromCenterOfBot = 2, 
    crosshairOffset = 0;

// DIO
static const int
    intakeAbsoluteEncoderPort = 8,
    climbArmsAbsID = 4,
    HoodAbsoluteEncoderPort = 1,
    TurretAbsoluteEncoderPort = 0,
    bottomBeamBreakPort = 3,
    midBeamBreakPort = 9,
    topBeamBreakPort = 5;




