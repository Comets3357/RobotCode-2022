#include "subsystems/Drivebase.h"
#include "RobotData.h"

void Drivebase::RobotInit()
{
    dbL.ConfigFactoryDefault();
    dbLF.ConfigFactoryDefault();
    dbR.ConfigFactoryDefault();
    dbRF.ConfigFactoryDefault();
    

    dbLF.Follow(dbL);
    dbRF.Follow(dbR);

    dbL.SetInverted(true);
    dbLF.SetInverted(true);

    dbR.SetInverted(false);    
    dbRF.SetInverted(false);

    dbL.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbR.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);

    // NEED TO SET CURRENT LIMIT
    /**
  * Configure the current limits that will be used
  * Stator Current is the current that passes through the motor stators.
  *  Use stator current limits to limit rotor acceleration/heat production
  * Supply Current is the current that passes into the controller from the supply
  *  Use supply current limits to prevent breakers from tripping
  *
  *                                                               enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  */
    dbL.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbLF.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbR.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbRF.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));

    // PIDs for blue db
    dbL.Config_kF(0, 0.032514);
    dbL.Config_kP(0, 0.038723);
    dbL.Config_kD(0, 0);

    dbR.Config_kF(0, 0.032514);
    dbR.Config_kP(0, 0.038723);
    dbR.Config_kD(0, 0);

    // PIDs for 2022
    /* dbL.Config_kF(0, 0.073067);
    dbL.Config_kP(0, 0.19673);
    dbL.Config_kD(0, 0);

    dbR.Config_kF(0, 0.073067);
    dbR.Config_kP(0, 0.19673);
    dbR.Config_kD(0, 0); */

    setPercentOutput(0, 0);
   
   // odometry
    // resetOdometry(2, 3, 0);
    frc::SmartDashboard::PutData("Field", &field);
    odometryInitialized = false;
}

void Drivebase::TeleopInit(const RobotData &robotData) {
    if (!odometryInitialized) {
        resetOdometry(robotData.autonData.startPoint, robotData.gyroData.rawYaw);
        odometryInitialized = true;
    }
}

void Drivebase::AutonomousInit(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) {
    // every time autonomous is enabled

    // wpi::outs() << "autonomous INIT!";
    
    frc::SmartDashboard::PutString("db auton init", "done");

    lastDegrees.clear();
    odometryInitialized = false;

    // get trajectory from auton's pointer
    getTrajectoryFile(robotData, drivebaseData, autonData);

    // frc::SmartDashboard::PutNumber("traj test", robotData.autonData.trajectory.TotalTime().to<double>());

    // resetOdometry(3.167, 7.492, robotData);
    // resetOdometry(9.25, 3.71, 1.821, -0.668, robotData);
    // resetOdometry(0, 0, 1, 0, robotData);
    // resetOdometry(0, 6.658, 1, 0, robotData);
    // resetOdometry(3, 1, 0, 1, robotData);
    // resetOdometry(0, 3.658, 1, 0, robotData);
    // resetOdometry(8.261, 1, 0, 0.5, robotData);


    // frc::SmartDashboard::PutNumber("trajX", 0);
    // frc::SmartDashboard::PutNumber("trajY", 0);
}

void Drivebase::RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData)
{
    updateData(robotData, drivebaseData);

    if (frc::DriverStation::IsEnabled())
    {
        dbL.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbR.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    }

    if (frc::DriverStation::IsTeleop()) {
        teleopControl(robotData);
    }
    else if (frc::DriverStation::IsAutonomous())
    {
        autonControl(robotData, drivebaseData, autonData);
    }

    frc::SmartDashboard::PutNumber("driveMode", drivebaseData.driveMode);

    frc::SmartDashboard::PutBoolean("odometryInitialized", odometryInitialized);
}

void Drivebase::DisabledInit()
{
    
    setPercentOutput(0, 0);
    dbL.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    dbR.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    odometryInitialized = false;
}

// updates encoder and gyro values
void Drivebase::updateData(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    // //add back wheel encoders at some point
    // drivebaseData.currentLDBPos = dbLMEncoder.GetPosition();
    // drivebaseData.currentRDBPos = dbRMEncoder.GetPosition();

    drivebaseData.lDriveVel = dbL.GetSensorCollection().GetIntegratedSensorVelocity() / mpsToTpds;
    drivebaseData.rDriveVel = dbR.GetSensorCollection().GetIntegratedSensorVelocity() / mpsToTpds;

    
    frc::SmartDashboard::PutNumber("lDriveVel", drivebaseData.lDriveVel);
    frc::SmartDashboard::PutNumber("rDriveVel", drivebaseData.rDriveVel);

    // call updateOdometry
    updateOdometry(robotData, drivebaseData);

    // drivebaseData.previousLDBPos = dbLMEncoder.GetPosition();
    // drivebaseData.previousRDBPos = dbRMEncoder.GetPosition();

    frc::SmartDashboard::PutNumber("odometryPoseX", odometry.GetPose().Translation().X().to<double>());
    frc::SmartDashboard::PutNumber("odometryPoseY", odometry.GetPose().Translation().Y().to<double>());

    frc::SmartDashboard::PutNumber("currentPoseX", drivebaseData.currentPose.Translation().X().to<double>());
    frc::SmartDashboard::PutNumber("currentPoseY", drivebaseData.currentPose.Translation().Y().to<double>());

    frc::SmartDashboard::PutNumber("lEncoderPosition", dbL.GetSensorCollection().GetIntegratedSensorPosition());
    frc::SmartDashboard::PutNumber("rEncoderPosition", dbR.GetSensorCollection().GetIntegratedSensorPosition());
}
// driving functions:

// adjusts for the deadzone and converts joystick input to velocity values for PID
void Drivebase::teleopControl(const RobotData &robotData)
{
    

    // double tempLDrive = robotData.controlData.lDrive;
    // double tempRDrive = robotData.controlData.rDrive;

    // // converts from tank to arcade drive, limits the difference between left and right drive
    // double frontBack = robotData.controlData.maxStraight * (tempLDrive + tempRDrive) / 2;
    // double leftRight = robotData.controlData.maxTurn * (tempRDrive - tempLDrive) / 2;

    // //deadzone NOT needed for drone controller
    // if (tempLDrive <= -0.08 || tempLDrive >= 0.08)
    // {
    //     tempLDrive = (frontBack - leftRight);
    // }
    // else
    // {
    //     tempLDrive = 0;
    // }

    // if (tempRDrive <= -0.08 || tempRDrive >= 0.08)
    // {
    //     tempRDrive = (frontBack + leftRight);
    // }
    // else
    // {
    //     tempRDrive = 0;
    // }

    // //set as percent vbus
    // setPercentOutput(tempLDrive, tempRDrive);


    turnInPlaceTeleop(180 - robotData.gyroData.rawYaw, robotData);
}

void Drivebase::autonControl(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) {
    // sample the desired pos based on time from trajectory object
    // get chassis speeds from ramsete controller, comparing current pos w/ desired pos
    // translate chassis speeds to wheel speeds w/ toWheelSPeeds from kinematics using rate of turn and linear speed
    // feed wheel speeds to PID
    // check if done with current path by either checking TotalTime() or checking in vicinity of final target point

    if (drivebaseData.driveMode == driveMode_break)
    {
        setVelocity(0, 0);
        frc::SmartDashboard::PutNumber("breakEndSec", breakEndSec);
        if (robotData.timerData.secSinceEnabled > breakEndSec) {
            getTrajectoryFile(robotData, drivebaseData, autonData);
        }
    }
    else if (drivebaseData.driveMode == driveMode_turnInPlace)
    {
        turnInPlaceAuton(turnInPlaceDegrees - robotData.gyroData.rawYaw, robotData, drivebaseData, autonData);
    }
    else if (drivebaseData.driveMode == driveMode_trajectory)
    {
        frc::SmartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);

        units::second_t sampleSec{robotData.timerData.secSinceEnabled - trajectorySecOffset};

        frc::SmartDashboard::PutNumber("sampleSec", sampleSec.to<double>());

        double totalTime = trajectory.TotalTime().to<double>();
        frc::SmartDashboard::PutNumber("trajTotalTime", totalTime);

        double lowest;
        if (sampleSec.to<double>() < totalTime) {
            lowest = sampleSec.to<double>();
        } else {
            // lowest = totalTime;
            // lowest = /* sampleSec.to<double>() */ totalTime;
            getTrajectoryFile(robotData, drivebaseData, autonData);
        }
        
        units::second_t lowestSec{lowest};

        frc::Trajectory::State trajectoryState = trajectory.Sample(lowestSec);
        frc::Pose2d desiredPose = trajectoryState.pose;

        double trajX = desiredPose.X().to<double>();
        double trajY = desiredPose.Y().to<double>();
        frc::SmartDashboard::PutNumber("trajX", trajX);
        frc::SmartDashboard::PutNumber("trajY", trajY);





        frc::ChassisSpeeds chassisSpeeds = ramseteController.Calculate(odometry.GetPose(), trajectoryState);

        frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

        double leftWheelSpeed = wheelSpeeds.left.to<double>();
        double rightWheelSpeed = wheelSpeeds.right.to<double>();
        frc::SmartDashboard::PutNumber("leftWheelSpeed", leftWheelSpeed);
        frc::SmartDashboard::PutNumber("rightWheelSpeed", rightWheelSpeed);

        setVelocity(leftWheelSpeed, rightWheelSpeed);
    }
}

void Drivebase::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {

}

void Drivebase::drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot) {
    
}

/**
 * 
 */
void Drivebase::updateOdometry(const RobotData &robotData, DrivebaseData &drivebaseData) {
    // update odometry w/ angle, L & R distances in m

    // find change in distance
    // double changeLDBPos = drivebaseData.currentLDBPos - drivebaseData.previousLDBPos;
    // double changeRDBPos = drivebaseData.currentRDBPos - drivebaseData.previousRDBPos;

    // double forwardSpeed = (changeLDBPos + changeRDBPos) / 2;
    
    // frc::SmartDashboard::PutNumber("changeLDBPos", changeLDBPos);
    // frc::SmartDashboard::PutNumber("changeRDBPos", changeRDBPos);
    // frc::SmartDashboard::PutNumber("forwardSpeed", forwardSpeed);

    double pi = 2 * std::acos(0.0);
    // frc::SmartDashboard::PutNumber("pi", pi);
    // double currentAngle = (robotData.gyroData.rawYaw / 180) * pi;
    // frc::SmartDashboard::PutNumber("currentAngle", currentAngle);

    // drivebaseData.cumulativeX += (forwardSpeed * std::cos(currentAngle));
    // drivebaseData.cumulativeY += (forwardSpeed * std::sin(currentAngle));

    // frc::SmartDashboard::PutNumber("cumulativeX", drivebaseData.cumulativeX);
    // frc::SmartDashboard::PutNumber("cumulativeY", drivebaseData.cumulativeY);

    // wpi::outs() << drivebaseData.cumulativeX;



    // library's odometry
    units::radian_t currentRadians{(robotData.gyroData.rawYaw / 180) * pi};
    frc::Rotation2d currentRotation{currentRadians};
    frc::SmartDashboard::PutNumber("currentRadians", currentRadians.to<double>());

    // NEGATIVE because left motor/encoder should be inverted
    units::meter_t leftDistance{-dbL.GetSensorCollection().GetIntegratedSensorPosition() / metersToTicks};
    units::meter_t rightDistance{dbR.GetSensorCollection().GetIntegratedSensorPosition() / metersToTicks};

    odometry.Update(currentRotation, leftDistance, rightDistance);

    field.SetRobotPose(odometry.GetPose());

    double libX = odometry.GetPose().Translation().X().to<double>();
    double libY = odometry.GetPose().Translation().Y().to<double>();

    double libXMeters = libX / metersToTicks;
    double libYMeters = libY / metersToTicks;

    frc::SmartDashboard::PutNumber("libX", libX);
    frc::SmartDashboard::PutNumber("libY", libY);

    frc::SmartDashboard::PutNumber("libXMeters", libXMeters);
    frc::SmartDashboard::PutNumber("libYMeters", libYMeters);

    drivebaseData.currentPose = getPose(libXMeters, libYMeters, currentRadians.to<double>());
    frc::SmartDashboard::PutNumber("odometryRadians", odometry.GetPose().Rotation().Radians().to<double>());
}

/**
 * @param pose position to reset odometry to (Pose2d)
 * @param resetAngle angle to reset odometry to (degrees, double)
 */
void Drivebase::resetOdometry(const frc::Pose2d &pose, double gyroAngle) {
    const units::radian_t gyroRadians{gyroAngle};
    frc::Rotation2d gyroRotation{gyroRadians};

    odometry.ResetPosition(pose, gyroRotation);
    zeroEncoders();
}

// rest odometry to zero position, current gyro angle (degrees, double)
void Drivebase::resetOdometry(double resetAngle) {
    const units::radian_t resetRadians{resetAngle};
    frc::Rotation2d resetRotation{resetRadians};

    odometry.ResetPosition(kZeroPose, kZeroAngle);
    zeroEncoders();
    
}

// reset odoemtry to zero position & zero angle
void Drivebase::resetOdometry() {
    odometry.ResetPosition(kZeroPose, kZeroAngle);
    zeroEncoders();
}

void Drivebase::resetOdometry(double x, double y, const RobotData &robotData) {
    const units::meter_t meterX{x};
    const units::meter_t meterY{y};

    const double pi = 2 * std::acos(0.0);
    const units::radian_t radianYaw{robotData.gyroData.rawYaw / 180 * pi};
    frc::SmartDashboard::PutNumber("Pi", pi);
    frc::SmartDashboard::PutNumber("radian yaw", robotData.gyroData.rawYaw / 180 * pi);

    const frc::Rotation2d resetRotation{radianYaw};
    const frc::Pose2d resetPose{meterX, meterY, radianYaw};
    odometry.ResetPosition(resetPose, resetRotation);

    zeroEncoders();
}

// reset odometry to any double x, y, deg
void Drivebase::resetOdometry(double x, double y, double radians, const RobotData &robotData) {
    const units::meter_t meterX{x};
    const units::meter_t meterY{y};

    const units::radian_t radianYaw{radians};
    // frc::SmartDashboard::PutNumber("Pi", pi);
    // frc::SmartDashboard::PutNumber("radian yaw", robotData.gyroData.rawYaw / 180 * pi);

    const units::radian_t gyroRadians{robotData.gyroData.rawYaw / 180 * M_PI};
    frc::SmartDashboard::PutNumber("RORaw Yaw", robotData.gyroData.rawYaw);

    const frc::Rotation2d gyroRotation{gyroRadians};
    const frc::Pose2d resetPose{meterX, meterY, radianYaw};
    odometry.ResetPosition(resetPose, gyroRotation);

    zeroEncoders();
}

// reset odometry to any double x, y, tanX, tanY
void Drivebase::resetOdometry(double x, double y, double tanX, double tanY, const RobotData &robotData) {
    const units::meter_t meterX{x};
    const units::meter_t meterY{y};

    const double pi = 2 * std::acos(0.0);

    frc::SmartDashboard::PutNumber("std::tan (tanY / tanX)", (tanY / tanX));
    frc::SmartDashboard::PutNumber("std::tan (tanY / tanX)", std::tan(tanY / tanX));
    frc::SmartDashboard::PutString("what the", "heck");

    units::radian_t radianYaw{M_PI / 2};
    if (tanX != 0) {
        units::radian_t blockScopeAngle{std::atan(tanY / tanX)};
        radianYaw = blockScopeAngle;
        frc::SmartDashboard::PutString("calculation", "happened");
    } else if (tanY < 0) {
        units::radian_t blockScopeAngle{ 3 * M_PI / 2 };
        radianYaw = blockScopeAngle;
        frc::SmartDashboard::PutString("neg tanY", "happened");
    }
    // frc::SmartDashboard::PutNumber("Pi", pi);
    // frc::SmartDashboard::PutNumber("radian yaw", robotData.gyroData.rawYaw / 180 * pi);

    // units::radian_t radianYaw{0};

    frc::SmartDashboard::PutNumber("radianYaw", radianYaw.to<double>());

    const units::radian_t gyroRadians{robotData.gyroData.rawYaw / 180 * pi};
    frc::SmartDashboard::PutNumber("RORaw Yaw", robotData.gyroData.rawYaw);

    const frc::Rotation2d gyroRotation{gyroRadians};
    const frc::Pose2d resetPose{meterX, meterY, radianYaw};
    odometry.ResetPosition(resetPose, gyroRotation);

    zeroEncoders();
}


// sets the drive base velocity for auton
void Drivebase::setVelocity(double leftVel, double rightVel)
{
    // leftVel *= 891.136;
    // rightVel *= 891.136;

    // dbLMPID.SetReference(leftVel, rev::ControlType::kVelocity);
    // dbRMPID.SetReference(rightVel, rev::ControlType::kVelocity);


    // dbL.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 2*2048*10);
    // dbR.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 2*2048*10);

    // conversion factor, see Discord #advanced-auton image
    // const double mpsToTps = (6.0 / 0.1524) * (1 / (6.0 * M_PI)) * (64.0 / 8.0) * (2048.0) * (0.1);
    // const double mpsToTps = (4.0 / 0.1016) * (1 / (4.0 * M_PI)) * (44.0 / 9.0) * (2048.0) * (0.1);

    double leftTPDS = leftVel * mpsToTpds;
    double rightTPDS = rightVel * mpsToTpds;

    dbL.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, leftTPDS);
    dbR.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, rightTPDS);
}

void Drivebase::zeroEncoders() {
    dbL.GetSensorCollection().SetIntegratedSensorPosition(0.0);
    dbR.GetSensorCollection().SetIntegratedSensorPosition(0.0);
    frc::SmartDashboard::PutString("zeroed encoders", "yes");
}

frc::Pose2d Drivebase::getPose(double x, double y, double deg) {
    units::meter_t meterX{x};
    units::meter_t meterY{y};

    const units::radian_t radianYaw{deg / 180 * M_PI};
    const frc::Rotation2d rotation{radianYaw};
    frc::Pose2d pose{meterX, meterY, rotation};
    return pose;
}

void Drivebase::getTrajectoryFile(const RobotData &robotData, DrivebaseData &drivebaseData,  AutonData &autonData) {

    autonData.autonStep++;

    if (autonData.autonStep < autonData.pathGroup.size()) {
        frc::SmartDashboard::PutString("getTrajectoryFile()", "b");
        frc::SmartDashboard::PutNumber("autonStep", autonData.autonStep);
        frc::SmartDashboard::PutString("robotData.autonData.pathGroup[robotData.autonData.autonStep", autonData.pathGroup[autonData.autonStep]);

        std::string trajectoryName = autonData.pathGroup.at(autonData.autonStep);

        // frc::SmartDashboard::PutString("trajectoryName", trajectoryName);

        if (trajectoryName.substr(0, 11) == "turnInPlace")
        {
            drivebaseData.driveMode = driveMode_turnInPlace;
            turnInPlaceDegrees = robotData.gyroData.rawYaw + std::stod(trajectoryName.substr(12, trajectoryName.length()));
            
            return;
        }

        else if (trajectoryName.substr(0, 5) == "break")
        {
            drivebaseData.driveMode = driveMode_break;
            breakEndSec = std::stod(trajectoryName.substr(6, trajectoryName.length())) + robotData.timerData.secSinceEnabled;
            return;
        }

        else
        {
            drivebaseData.driveMode = driveMode_trajectory; 

            fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

            fs::path pathDirectory = deployDirectory / "paths" / (trajectoryName + ".wpilib.json");

            frc::SmartDashboard::PutString("pathDirectory", pathDirectory.string());

            trajectory = frc::TrajectoryUtil::FromPathweaverJson(pathDirectory.string());
            frc::SmartDashboard::PutNumber("ogSecSE", robotData.timerData.secSinceEnabled);
            trajectorySecOffset = robotData.timerData.secSinceEnabled;

            
            if (!odometryInitialized) {
                // automatically reset odometry to start pose of the first path
                units::second_t zeroSec{0};


                frc::Trajectory::State trajectoryState = trajectory.Sample(zeroSec);
                frc::Pose2d firstPose = trajectoryState.pose;

                double firstX = firstPose.X().to<double>();
                double firstY = firstPose.Y().to<double>();
                
                double firstRadians = firstPose.Rotation().Radians().to<double>();
                frc::SmartDashboard::PutNumber("firstRadians", firstRadians);

                resetOdometry(firstX, firstY, firstRadians, robotData);

                odometryInitialized = true;
            }

            frc::SmartDashboard::PutBoolean("odometryInitialized", odometryInitialized);
        }
    }
}

void Drivebase::turnInPlaceAuton(double degrees, const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) {

    frc::SmartDashboard::PutNumber("degree diff", degrees);
    
    lastDegrees.push_back(degrees);
    if (lastDegrees.size() > 5) {
        lastDegrees.pop_front();
    }

    double leftOutput = 0;
    double rightOutput = 0;

    int directionFactor = 1;
    if (degrees <= 0) {
        directionFactor = -1;
    }

    if (allValuesWithin(lastDegrees, 1)) {
        setPercentOutput(0, 0);
        getTrajectoryFile(robotData, drivebaseData, autonData);
        frc::SmartDashboard::PutString("AUTON", "TURN IN PLACE");
    } else {
        leftOutput = std::pow(std::abs(degrees / 361), 1) + 0.07;
        rightOutput = std::pow(std::abs(degrees / 361), 1) + 0.07;

    }
    

    frc::SmartDashboard::PutNumber("leftOutput", leftOutput);
    frc::SmartDashboard::PutNumber("rightOutput", rightOutput);
    
    setPercentOutput(leftOutput * (-directionFactor), rightOutput * (directionFactor));
}

void Drivebase::turnInPlaceTeleop(double degrees, const RobotData &robotData) {
    frc::SmartDashboard::PutNumber("degree diff", degrees);
    
    lastDegrees.push_back(degrees);
    if (lastDegrees.size() > 5) {
        lastDegrees.pop_front();
    }

    double leftOutput = 0;
    double rightOutput = 0;

    int directionFactor = 1;
    if (degrees <= 0) {
        directionFactor = -1;
    }

    if (allValuesWithin(lastDegrees, 1)) {
        setPercentOutput(0, 0);
        frc::SmartDashboard::PutString("TELEOP", "TURN IN PLACE");
    } else {
        leftOutput = std::pow(std::abs(degrees / 361), 1) + 0.07;
        rightOutput = std::pow(std::abs(degrees / 361), 1) + 0.07;

    }
    

    frc::SmartDashboard::PutNumber("leftOutput", leftOutput);
    frc::SmartDashboard::PutNumber("rightOutput", rightOutput);
    
    setPercentOutput(leftOutput * (-directionFactor), rightOutput * (directionFactor));
}


void Drivebase::setPercentOutput(double leftVBus, double rightVBus) {
    dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, leftVBus);
    dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightVBus);
}


bool Drivebase::allValuesWithin(std::deque<double> deque, double tolerance) {
    bool hasOutlier = false;
    for (int i = 0; i < deque.size(); i++) {
        if (std::abs(deque[i]) > tolerance) {
            hasOutlier = true;
        }
    }
    return !hasOutlier;
}