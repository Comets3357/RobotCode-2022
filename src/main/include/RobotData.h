#pragma once

// includes other files' data
#include "controller/Controller.h"
#include "common/OtherComponents.h"
#include "common/Gyro.h"
#include "common/Limelight.h"
#include "common/Timer.h"
#include "common/ColorSensor.h"

#include "auton/Auton.h"

#include "subsystems/Drivebase.h"
#include "subsystems/Climb.h"
#include "subsystems/Intake.h"
#include "subsystems/Indexer.h"
#include "subsystems/Shooter.h"

struct RobotData
{
    ControllerData controllerData;
    ControlData controlData;
    OtherComponentsData otherComponentsData;
    GyroData gyroData;
    LimelightData limelightData;
    TimerData timerData;

    AutonData autonData;

    DrivebaseData drivebaseData;
    IntakeData intakeData;
    IndexerData indexerData;
    ClimbData climbData;
    ShooterData shooterData;
};