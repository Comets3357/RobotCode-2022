#pragma once

#include "Constants.h"
#include "auton/Auton.h"

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/controller/RamseteController.h>
// #include <frc/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <cmath>
#include <algorithm>

#include <string>
#include <fstream>

struct RobotData;

struct DrivebaseData
{
     // in meters eventually
    double currentLDBPos = 0.0;
    double currentRDBPos = 0.0;

    double previousLDBPos = 0.0;
    double previousRDBPos = 0.0;

    double lDriveVel = 0.0;
    double rDriveVel = 0.0;

    // odometry
    frc::Pose2d currentPose{};

    double cumulativeX = 0.0;
    double cumulativeY = 0.0;
};

class Drivebase
{

public:
    void RobotInit();
    void TeleopInit(const RobotData &robotData);
    void AutonomousInit(const RobotData &robotData, AutonData &autonData);
    void RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData);
    void DisabledInit();

private:

    void updateData(const RobotData &robotData, DrivebaseData &drivebaseData);
    void teleopControl(const RobotData &robotData);
    void autonControl(const RobotData &robotData, AutonData &autonData);

    void setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
    void drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
    void updateOdometry(const RobotData &robotData, DrivebaseData &drivebaseData);
    void resetOdometry(const frc::Pose2d &pose, double resetAngle);
    void resetOdometry(double resetAngle);
    void resetOdometry();
    void resetOdometry(double x, double y, const RobotData &robotData);
    void resetOdometry(double x, double y, double degrees, const RobotData &robotData);
    void resetOdometry(double x, double y, double tanX, double tanY, const RobotData &robotData);
    void zeroEncoders();
    void setVelocity(double leftVel, double rightVel);
    frc::Pose2d getPose(double x, double y, double deg);
    void getTrajectoryFile(const RobotData &robotData, AutonData &autonData);
    void turnInPlace(double degrees);

    // odometry
    const units::radian_t kZeroAngle{0.0};
    units::meter_t meterX{3.167};
    units::meter_t meterY{7.492};
    const frc::Translation2d testTrans{meterX, meterY};
    units::radian_t zeroRadians{0};
    const frc::Rotation2d testRot{zeroRadians};
    const frc::Pose2d kZeroPose{testTrans, testRot};
    // frc::Rotation2d gyroYaw{};  // updated by robotPeriodic
    frc::DifferentialDriveOdometry odometry{kZeroAngle};

    const units::meter_t kTrackWidth{0.55};
    frc::DifferentialDriveKinematics kinematics{kTrackWidth};

    frc::Trajectory trajectory{};
    frc::RamseteController ramseteController{};

    frc::Field2d field;

    double trajectorySecOffset = 0;

    // for 6 in wheels
    const double mpsToTpds = (6.0 / 0.1524) * (1 / (6.0 * M_PI)) * (64.0 / 8.0) * (2048.0) * (0.1);
    const double metersToTicks = (6.0 / 0.1524) * (1 / (6.0 * M_PI)) * (64.0 / 8.0) * (2048.0);

    // for 4 in wheels
    // const double mpsToTpds = (4.0 / 0.1016) * (1 / (4.0 * M_PI)) * (44.0 / 9.0) * (2048.0) * (0.1);
    // const double metersToTicks = (4.0 / 0.1016) * (1 / (4.0 * M_PI)) * (44.0 / 9.0) * (2048.0);

    // forwards are leads
    ctre::phoenix::motorcontrol::can::TalonFX dbL{leftLeadDeviceID};
    ctre::phoenix::motorcontrol::can::TalonFX dbLF{leftFollowDeviceID};

    ctre::phoenix::motorcontrol::can::TalonFX dbR{rightLeadDeviceID};
    ctre::phoenix::motorcontrol::can::TalonFX dbRF{rightFollowDeviceID};

    



};