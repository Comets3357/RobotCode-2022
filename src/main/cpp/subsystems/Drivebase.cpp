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

    // PIDs
    dbL.Config_kP(0, 0.039271);
    dbL.Config_kD(0, 0);

    dbR.Config_kP(0, 0.039271);
    dbR.Config_kD(0, 0);


    dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
   
   // odometry
    // resetOdometry(2, 3, 0);
    frc::SmartDashboard::PutData("Field", &field);
}

void Drivebase::TeleopInit() {
    resetOdometry(0, 0, 0);
}

void Drivebase::AutonomousInit(AutonData &autonData) {
    // every time autonomous is enabled

    // wpi::outs() << "autonomous INIT!";
    
    frc::SmartDashboard::PutString("db auton init", "done");

    // get trajectory from auton's pointer
    trajectory = (autonData.trajectory);
    frc::SmartDashboard::PutNumber("traj test", autonData.trajectory.TotalTime().to<double>());

    // resetOdometry(3.167, 7.492, robotData);
    resetOdometry(0, 3, 0);

    frc::SmartDashboard::PutNumber("trajX", 0);
    frc::SmartDashboard::PutNumber("trajY", 0);
}

void Drivebase::RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData)
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
        autonControl(robotData);
    }

}

void Drivebase::DisabledInit()
{
    
    dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    dbL.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbR.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
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
    

    double tempLDrive = robotData.controlData.lDrive;
    double tempRDrive = robotData.controlData.rDrive;

    // converts from tank to arcade drive, limits the difference between left and right drive
    double frontBack = robotData.controlData.maxStraight * (tempLDrive + tempRDrive) / 2;
    double leftRight = robotData.controlData.maxTurn * (tempRDrive - tempLDrive) / 2;

    //deadzone NOT needed for drone controller
    if (tempLDrive <= -0.08 || tempLDrive >= 0.08)
    {
        tempLDrive = (frontBack - leftRight);
    }
    else
    {
        tempLDrive = 0;
    }

    if (tempRDrive <= -0.08 || tempRDrive >= 0.08)
    {
        tempRDrive = (frontBack + leftRight);
    }
    else
    {
        tempRDrive = 0;
    }

    //set as percent vbus
    dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, tempLDrive);
    dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, tempRDrive);
}

void Drivebase::autonControl(const RobotData &robotData) {
    // sample the desired pos based on time from trajectory object
    // get chassis speeds from ramsete controller, comparing current pos w/ desired pos
    // translate chassis speeds to wheel speeds w/ toWheelSPeeds from kinematics using rate of turn and linear speed
    // feed wheel speeds to PID
    // check if done with current path by either checking TotalTime() or checking in vicinity of final target point

    frc::SmartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);

    
    
    const units::second_t secSinceEnabled{robotData.timerData.secSinceEnabled};
    frc::Trajectory::State trajectoryState = trajectory.Sample(secSinceEnabled);
    frc::Pose2d desiredPose = trajectoryState.pose;

    double totalTime = trajectory.TotalTime().to<double>();
    frc::SmartDashboard::PutNumber("trajTotalTime", totalTime);

    double trajX = desiredPose.X().to<double>();
    double trajY = desiredPose.Y().to<double>();
    frc::SmartDashboard::PutNumber("trajX", trajX);
    frc::SmartDashboard::PutNumber("trajY", trajY);

    frc::ChassisSpeeds chassisSpeeds =  ramseteController.Calculate(odometry.GetPose(), trajectoryState);

    frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

    double leftWheelSpeed = wheelSpeeds.left.to<double>();
    double rightWheelSpeed = wheelSpeeds.right.to<double>();
    frc::SmartDashboard::PutNumber("leftWheelSpeed", leftWheelSpeed);        frc::SmartDashboard::PutNumber("rightWheelSpeed", rightWheelSpeed);

    setVelocity(leftWheelSpeed, rightWheelSpeed);
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
}

/**
 * @param pose position to reset odometry to (Pose2d)
 * @param resetAngle angle to reset odometry to (degrees, double)
 */
void Drivebase::resetOdometry(const frc::Pose2d &pose, double resetAngle) {
    const units::radian_t resetRadians{resetAngle};
    frc::Rotation2d resetRotation{resetRadians};

    odometry.ResetPosition(pose, kZeroAngle);
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
void Drivebase::resetOdometry(double x, double y, double deg) {
    const units::meter_t meterX{x * 34220};
    const units::meter_t meterY{y * 34220};

    const double pi = 2 * std::acos(0.0);
    const units::radian_t radianYaw{deg / 180 * pi};
    // frc::SmartDashboard::PutNumber("Pi", pi);
    // frc::SmartDashboard::PutNumber("radian yaw", robotData.gyroData.rawYaw / 180 * pi);

    const frc::Rotation2d resetRotation{radianYaw};
    const frc::Pose2d resetPose{meterX, meterY, radianYaw};
    odometry.ResetPosition(resetPose, resetRotation);

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

    const double pi = 2 * std::acos(0.0);
    const units::radian_t radianYaw{deg / 180 * pi};
    const frc::Rotation2d rotation{radianYaw};
    frc::Pose2d pose{meterX, meterY, rotation};
    return pose;
}