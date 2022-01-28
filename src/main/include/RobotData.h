#pragma once

// includes other files' data
#include "controller/Controller.h"
#include "common/OtherComponents.h"
#include "common/Gyro.h"
#include "common/Limelight.h"
#include "common/Timer.h"
#include "common/LEDs.h"

#include "auton/Auton.h"

#include "subsystems/Drivebase.h"

//could be separated into all separate files for the data *from* each subsystem
//commented out variables are not in use
struct RobotData
{
    ControllerData controllerData;
    ControlData controlData;
    OtherComponentsData otherComponentsData;
    GyroData gyroData;
    LimelightData limelightData;
    TimerData timerData;
    ColorSensorData colorSensorData;

    AutonData autonData;

    DrivebaseData drivebaseData;
};