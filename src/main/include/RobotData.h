#pragma once

// includes other files' data
#include "controller/Controller.h"
#include "common/OtherComponents.h"
#include "common/Gyro.h"
#include "common/Limelight.h"
#include "common/VisionLookup.h"
#include "common/Timer.h"
#include "common/Arduino.h"
#include "common/Jetson.h"
#include "common/BenchTest.h"

#include "auton/Auton.h"

#include "subsystems/Drivebase.h"
#include "subsystems/Climb.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

//could be separated into all separate files for the data *from* each subsystem
//commented out variables are not in use
struct RobotData
{
    ControllerData controllerData;
    ControlData controlData;
    OtherComponentsData otherComponentsData;
    GyroData gyroData;
    LimelightData limelightData;
    VisionLookupData visionLookupData;
    TimerData timerData;
    BenchTestData benchTestData;

    AutonData autonData;

    DrivebaseData drivebaseData;
    IntakeData intakeData;
    IndexerData indexerData;
    ClimbData climbData;
    ShooterData shooterData;
    ArduinoData arduinoData;
    
};