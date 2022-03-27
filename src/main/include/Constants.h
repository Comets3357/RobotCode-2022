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
    // absOut = 0.485,
    // absIn = 0.616,
    // revOut = 6.4,
    // revIn = 0;

    //comp bot intake pivot positions
    absOut = 0.465,
    absIn = 0.605,
    revOut = 6.14,
    revIn = 0;

//indexer
static const int
    indexerBeltsID = 22,
    indexerWheelID = 21;

// shooter IDs
static const int
    shooterWheelLeadID = 31,
    shooterHoodID = 33, 
    shooterTurretID = 34, 
    hoodRollerID = 32; 

static const double
    //comp bot shooter hood positions
    hoodabsOut = 0.07,
    hoodabsIn = 0.945,
    hoodrevOut = -38,
    hoodrevIn = 0,
    hoodAngleOut = 43,
    hoodAngleIn = 21,

    //PRATLAS shooter hood positions
    // hoodabsOut = 0.103,
    // hoodabsIn = 0.98,
    // hoodrevOut = -38,
    // hoodrevIn = 0,
    // hoodAngleOut = 43,
    // hoodAngleIn = 21,

    //turret positions
    turretZeroDegrees = 0, 
    turretFullRotationDegrees = 540, 
    turretMiddleDegrees = (turretFullRotationDegrees - turretZeroDegrees)/2,
    turretZeroRev = 85.74, //87.5
    turretFullRotationRev_CCW = 174.85, //174.85
    turretFullRotationRev_C = 0, //0
    turretZeroAbs = 0.499, //0.479
    turretFullRotationAbs_CCW = 0.933, //0.91
    turretFullRotationAbs_C = 0.076; //0.05
    

    //FOR PRATLAS
    // turretZeroDegrees = 5, //15
    // turretFullRotationDegrees = 535, //540
    // turretMiddleDegrees = (turretFullRotationDegrees - turretZeroDegrees)/2,
    // turretZeroRev = 87.5, //87.5
    // turretFullRotationRev_CCW = 174.85, //174.85
    // turretFullRotationRev_C = 0, //0
    // turretZeroAbs = 0.479, //0.479
    // turretFullRotationAbs_CCW = 0.91, //0.91
    // turretFullRotationAbs_C = 0.05; //0.05


//turret gyro offset
static const float
    //at the the corresponding rate of rotation, offset the turret by an extra 20 degrees to account for the extra strength needed (centipetal acceleration?)
    turretGyroOffset2 = -50,
    turretGyroOffset1 = 0;
    
static const double 
    //at certain rotational rate points add the speed offset above
    rotationalRate2 = 400,
    rotationalRate1 = 0;


//Set shooting hood positions and velocity 
static const float
    outerLaunchHood = hoodrevOut,
    outerLaunchVel = 1990,
    innerLaunchHood = hoodrevOut,
    innerLaunchVel = 2040,
    wallHood = -32.33,
    wallVel = 1860,
    fenderHood = -0.25,
    fenderVel = 1690,

    outerLaunchHood_Low = hoodrevOut,
    outerLaunchVel_Low = 1990,
    innerLaunchHood_Low = hoodrevOut,
    innerLaunchVel_Low = 2040,
    wallHood_Low = -32.33,
    wallVel_Low = 1860,
    fenderHood_Low = -0.25,
    fenderVel_Low = 1240;


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




