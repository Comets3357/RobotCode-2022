#include "subsystems/Drivebase.h"
#include "RobotData.h"

void Drivebase::RobotInit()
{
    dbL.ConfigFactoryDefault();
    dbLF.ConfigFactoryDefault();
    dbR.ConfigFactoryDefault();
    dbRF.ConfigFactoryDefault();
    
    dbRF.Follow(dbR);
    dbLF.Follow(dbL);

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
    dbL.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 75, 50, 1.0));
    dbLF.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 75, 50, 1.0));
    dbR.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 75, 50, 1.0));
    dbRF.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 75, 50, 1.0));

    // PIDs for blue db
    /* dbL.Config_kF(0, 0.032514);
    dbL.Config_kP(0, 0.038723);
    dbL.Config_kD(0, 0);

    dbR.Config_kF(0, 0.032514);
    dbR.Config_kP(0, 0.038723);
    dbR.Config_kD(0, 0); */

    // PIDs for 2022 Calvin University
    // dbL.Config_kF(0, 0.077626);
    // dbL.Config_kP(0, 0.10352);
    // dbL.Config_kD(0, 0);

    // dbR.Config_kF(0, 0.077626);
    // dbR.Config_kP(0, 0.10352);
    // dbR.Config_kD(0, 0);

    // Atlas 03.26.22 Morning
    // dbL.Config_kF(0, 0.074655);
    // dbL.Config_kP(0, 0.1079);
    // dbL.Config_kD(0, 0);

    // dbR.Config_kF(0, 0.074655);
    // dbR.Config_kP(0, 0.1079);
    // dbR.Config_kD(0, 0);

    // Atlas 04.07.22 Final tread center drop but not fresh treads
    dbL.Config_kF(0, 0.072551);
    dbL.Config_kP(0, 0.1068);
    dbL.Config_kD(0, 0);

    dbR.Config_kF(0, 0.072551);
    dbR.Config_kP(0, 0.1068);
    dbR.Config_kD(0, 0);


    setPercentOutput(0, 0);

    zeroEncoders();

    odometryInitialized = false;
}

void Drivebase::TeleopInit(const RobotData &robotData) {
    if (!odometryInitialized) {
    //     frc::Pose2d startPoint = startPointChooser.GetSelected();
    //     resetOdometry(startPoint, robotData.gyroData.rawYaw);
        resetOdometry(0, 0, 0, robotData);
        odometryInitialized = true;
    }
    
}

void Drivebase::AutonomousInit(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) {    

    lastDegrees.clear();
    odometryInitialized = false;

    // get trajectory from auton's pointer
    getNextAutonStep(robotData, drivebaseData, autonData);
    zeroEncoders();
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
        teleopControl(robotData, drivebaseData);
    }
    else if (frc::DriverStation::IsAutonomous())
    {
        autonControl(robotData, drivebaseData, autonData);
    }
}

void Drivebase::DisabledInit()
{
    
    setPercentOutput(0, 0);
    dbL.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbR.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    odometryInitialized = false;
}

// updates encoder and gyro values
void Drivebase::updateData(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    // //add back wheel encoders at some point
    drivebaseData.currentLDBPos = dbL.GetSensorCollection().GetIntegratedSensorPosition();
    drivebaseData.currentRDBPos = dbR.GetSensorCollection().GetIntegratedSensorPosition();

    drivebaseData.lDriveVel = -dbL.GetSensorCollection().GetIntegratedSensorVelocity() / mpsToTpds;
    frc::SmartDashboard::PutNumber("lDriveVel", drivebaseData.lDriveVel);
    drivebaseData.rDriveVel = -dbR.GetSensorCollection().GetIntegratedSensorVelocity() / mpsToTpds;
    frc::SmartDashboard::PutNumber("rDriveVel", -drivebaseData.rDriveVel);

    // WARNING the average calcuation here subtracts for some reason. The values for left and right db velocity act as expected on their own...
    drivebaseData.avgDriveVel = (drivebaseData.lDriveVel - drivebaseData.rDriveVel) / 2.0;
    frc::SmartDashboard::PutNumber("avgDriveVel", drivebaseData.avgDriveVel);

    // option 1, will alloy us to shoot while pivoting drivebase
    drivebaseData.dbStationaryForShot = (std::abs(drivebaseData.avgDriveVel) < 0.1);
    // option 2, both driverails must be stationary for us to fire
    // drivebaseData.dbStationaryForShot = ((std::abs(drivebaseData.lDriveVel) < 0.2) && (std::abs(drivebaseData.rDriveVel) < 0.1));
    frc::SmartDashboard::PutBoolean("dbStationaryForShot", drivebaseData.dbStationaryForShot);

    // frc::SmartDashboard::PutNumber("driveMode", drivebaseData.driveMode);

    // call updateOdometry
    updateOdometry(robotData, drivebaseData);
}
// driving functions:

// adjusts for the deadzone and converts joystick input to velocity values for PID
void Drivebase::teleopControl(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    // assign drive mode
    if ((robotData.controlData.lDrive <= -0.08 || robotData.controlData.lDrive >= 0.08) || (robotData.controlData.rDrive <= -0.08 || robotData.controlData.rDrive >= 0.08)) {
        drivebaseData.driveMode = driveMode_joystick;
    }
    // else if (robotData.controlData.shootMode == shootMode_vision) {
        //drivebaseData.driveMode = driveMode_turnInPlace;
    // } 
    else {
        drivebaseData.driveMode = driveMode_joystick;
    }


    if (drivebaseData.driveMode == driveMode_joystick) {
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
        setPercentOutput(tempLDrive, tempRDrive);
    }
    // else if (drivebaseData.driveMode == driveMode_turnInPlace) {
        // turnInPlaceTeleop(-robotData.limelightData.angleOffset, robotData);
    // }
    // frc::SmartDashboard::PutNumber("limelight angle diff", -robotData.limelightData.angleOffset);


}

void Drivebase::autonControl(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) {
    // sample the desired pos based on time from trajectory object
    // get chassis speeds from ramsete controller, comparing current pos w/ desired pos
    // translate chassis speeds to wheel speeds w/ toWheelSPeeds from kinematics using rate of turn and linear speed
    // feed wheel speeds to PID
    // check if done with current path by either checking TotalTime() or checking in vicinity of final target point

    // frc::smartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);

    if (drivebaseData.driveMode == driveMode_break)
    {
        if (robotData.controlData.shootMode == shootMode_vision) {
            // turnInPlaceTeleop(-robotData.limelightData.angleOffset, robotData);
            // frc::smartDashboard::PutNumber("angleOffsetLimelight", robotData.limelightData.angleOffset);
        } else {
            setVelocity(0, 0);
        }
        // frc::SmartDashboard::PutNumber("breakEndSec", breakEndSec);
        if (robotData.timerData.secSinceEnabled > breakEndSec /* && !robotData.controlData.saFinalShoot */) {
            // frc::SmartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);
            // frc::SmartDashboard::PutNumber("breakEndSec", breakEndSec);
            getNextAutonStep(robotData, drivebaseData, autonData);
        }
    }
    else if (drivebaseData.driveMode == driveMode_turnInPlace)
    {
        turnInPlaceAuton(turnInPlaceDegrees - robotData.gyroData.rawYaw, robotData, drivebaseData, autonData);
    }
    else if (drivebaseData.driveMode == driveMode_trajectory)
    {
        // frc::SmartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);

        units::second_t sampleSec{robotData.timerData.secSinceEnabled - trajectorySecOffset};

        // frc::SmartDashboard::PutNumber("sampleSec", sampleSec.to<double>());

        double totalTime = trajectory.TotalTime().to<double>();
        // frc::SmartDashboard::PutNumber("trajTotalTime", totalTime);

        if (sampleSec.to<double>() > totalTime) {
            getNextAutonStep(robotData, drivebaseData, autonData);
        }
        
        frc::Trajectory::State trajectoryState = trajectory.Sample(sampleSec);
        frc::Pose2d desiredPose = trajectoryState.pose;

        double trajX = desiredPose.X().to<double>();
        double trajY = desiredPose.Y().to<double>();
        // frc::SmartDashboard::PutNumber("trajX", trajX);
        // frc::SmartDashboard::PutNumber("trajY", trajY);

        frc::ChassisSpeeds chassisSpeeds = ramseteController.Calculate(odometry.GetPose(), trajectoryState);

        frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

        double leftWheelSpeed = wheelSpeeds.left.to<double>();
        double rightWheelSpeed = wheelSpeeds.right.to<double>();
        // frc::SmartDashboard::PutNumber("leftWheelSpeed", leftWheelSpeed);
        // frc::SmartDashboard::PutNumber("rightWheelSpeed", rightWheelSpeed);

        setVelocity(leftWheelSpeed, rightWheelSpeed);
    }
}

void Drivebase::updateOdometry(const RobotData &robotData, DrivebaseData &drivebaseData) {

    // library's odometry
    units::radian_t currentRadians{(robotData.gyroData.rawYaw / 180) * M_PI};
    frc::Rotation2d currentRotation{currentRadians};

    // NEGATIVE because left motor/encoder should be inverted
    units::meter_t leftDistance{-dbL.GetSensorCollection().GetIntegratedSensorPosition() / metersToTicks};
    units::meter_t rightDistance{dbR.GetSensorCollection().GetIntegratedSensorPosition() / metersToTicks};

    odometry.Update(currentRotation, leftDistance, rightDistance);

    field.SetRobotPose(odometry.GetPose());
    frc::SmartDashboard::PutData("Field", &field);


    drivebaseData.currentPose = odometry.GetPose();
    drivebaseData.odometryX = drivebaseData.currentPose.X().to<double>();
    drivebaseData.odometryY = drivebaseData.currentPose.Y().to<double>();

    drivebaseData.odometryYaw = drivebaseData.currentPose.Rotation().Radians().to<double>();
    drivebaseData.odometryYaw = (drivebaseData.odometryYaw / M_PI * 180); // convert from radians [-pi, pi] to degrees [0, 360]
    if (drivebaseData.odometryYaw < 0) {
        drivebaseData.odometryYaw = 360 + drivebaseData.odometryYaw;
    }
    // frc::SmartDashboard::PutNumber("odometryX", drivebaseData.odometryX);
    // frc::SmartDashboard::PutNumber("odometryY", drivebaseData.odometryY);
    // frc::SmartDashboard::PutNumber("odometryYaw", drivebaseData.odometryYaw);
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

// reset odometry to any double x, y, deg
void Drivebase::resetOdometry(double x, double y, double radians, const RobotData &robotData) {
    const units::meter_t meterX{x};
    const units::meter_t meterY{y};

    const units::radian_t radianYaw{radians};
    // frc::SmartDashboard::PutNumber("Pi", pi);
    // frc::SmartDashboard::PutNumber("radian yaw", robotData.gyroData.rawYaw / 180 * pi);

    const units::radian_t gyroRadians{robotData.gyroData.rawYaw / 180 * M_PI};
    // frc::SmartDashboard::PutNumber("RORaw Yaw", robotData.gyroData.rawYaw);

    const frc::Rotation2d gyroRotation{gyroRadians};
    const frc::Pose2d resetPose{meterX, meterY, radianYaw};
    odometry.ResetPosition(resetPose, gyroRotation);

    zeroEncoders();
}


// sets the drive base velocity for auton
void Drivebase::setVelocity(double leftVel, double rightVel)
{
    // TDPS: ticks per decisecond

    double leftTPDS = leftVel * mpsToTpds;
    double rightTPDS = rightVel * mpsToTpds;

    dbL.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, leftTPDS);
    dbR.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, rightTPDS);
}

void Drivebase::zeroEncoders() {
    dbL.GetSensorCollection().SetIntegratedSensorPosition(0.0);
    dbR.GetSensorCollection().SetIntegratedSensorPosition(0.0);
    // frc::SmartDashboard::PutString("zeroed encoders", "yes");
}

// get the frc::Pose2d of a set of x, y meters & yaw degrees
frc::Pose2d Drivebase::getPose(double x, double y, double deg) {
    units::meter_t meterX{x};
    units::meter_t meterY{y};

    const units::radian_t radianYaw{deg / 180 * M_PI};
    const frc::Rotation2d rotation{radianYaw};
    frc::Pose2d pose{meterX, meterY, rotation};
    return pose;
}

void Drivebase::getNextAutonStep(const RobotData &robotData, DrivebaseData &drivebaseData,  AutonData &autonData) {

    autonData.autonStep++;

    if (autonData.autonStep < autonData.pathGroup.size()) {
        // frc::SmartDashboard::PutString("getNextAutonStep()", "b");
        // frc::SmartDashboard::PutNumber("autonStep", autonData.autonStep);
        // frc::SmartDashboard::PutString("robotData.autonData.pathGroup[robotData.autonData.autonStep", autonData.pathGroup[autonData.autonStep]);

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

        else {
            drivebaseData.driveMode = driveMode_trajectory; 

            fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

            fs::path pathDirectory = deployDirectory / "paths" / (trajectoryName + ".wpilib.json");

            // frc::SmartDashboard::PutString("pathDirectory", pathDirectory.string());

            trajectory = frc::TrajectoryUtil::FromPathweaverJson(pathDirectory.string());
            // frc::SmartDashboard::PutNumber("original seconds since enabled", robotData.timerData.secSinceEnabled);
            trajectorySecOffset = robotData.timerData.secSinceEnabled;

            
            if (!odometryInitialized) {
                // automatically reset odometry to start pose of the first path
                units::second_t zeroSec{0};


                frc::Trajectory::State trajectoryState = trajectory.Sample(zeroSec);
                frc::Pose2d firstPose = trajectoryState.pose;

                double firstX = firstPose.X().to<double>();
                double firstY = firstPose.Y().to<double>();
                
                double firstRadians = firstPose.Rotation().Radians().to<double>();
                // frc::SmartDashboard::PutNumber("firstRadians", firstRadians);

                resetOdometry(firstX, firstY, firstRadians, robotData);
                // frc::smartDashboard::PutNumber("autonStep OdoInit", autonData.autonStep);

                odometryInitialized = true;
            }

            // frc::smartDashboard::PutBoolean("odometryInitialized", odometryInitialized);
        }

        // frc::SmartDashboard::PutNumber("autonStep", autonData.autonStep);
    }
    else {
        drivebaseData.driveMode = driveMode_break;
    }
}

void Drivebase::turnInPlaceAuton(double degrees, const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) {

    // frc::SmartDashboard::PutNumber("degree diff", degrees);
    
    lastDegrees.push_back(degrees);
    if (lastDegrees.size() > 2) {
        lastDegrees.pop_front();
    }

    double leftOutput = 0;
    double rightOutput = 0;

    int directionFactor = 1;
    if (degrees <= 0) {
        directionFactor = -1;
    }

    // frc::smartDashboard::PutBoolean("allValuesWithin", allValuesWithin(lastDegrees, 1));
    if (allValuesWithin(lastDegrees, 5)) {
        setPercentOutput(0, 0);
        // only advance auton step if it's not shooting
        if (drivebaseData.driveMode == driveMode_turnInPlace) {
            getNextAutonStep(robotData, drivebaseData, autonData);
        }
        // frc::SmartDashboard::PutString("AUTON", "TURN IN PLACE");
    } else {
        // profile that adjusts aggressiveness of turn based on the amount of degrees left to turn. has been tuned for speed & accuracy on both small and large turns
        leftOutput = std::pow(std::abs(degrees / 400), 1.5) + 0.09;
        rightOutput = std::pow(std::abs(degrees / 400), 1.5) + 0.09;
    }
    

    // frc::SmartDashboard::PutNumber("leftOutput", leftOutput);
    // frc::SmartDashboard::PutNumber("rightOutput", rightOutput);
    
    setPercentOutput(leftOutput * (-directionFactor), rightOutput * (directionFactor));
}

void Drivebase::turnInPlaceTeleop(double degrees, const RobotData &robotData) {
    // frc::SmartDashboard::PutNumber("degree diff", degrees);
    
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

    if (allValuesWithin(lastDegrees, 4)) {
        setPercentOutput(0, 0);
        // frc::SmartDashboard::PutString("TELEOP", "TURN IN PLACE");
    } else {
        leftOutput = std::pow(std::abs(degrees / 400), 1.3) + 0.09;
        rightOutput = std::pow(std::abs(degrees / 400), 1.3) + 0.09;
    }
    

    // frc::SmartDashboard::PutNumber("leftOutput", leftOutput);
    // frc::SmartDashboard::PutNumber("rightOutput", rightOutput);
    
    setPercentOutput(leftOutput * (-directionFactor), rightOutput * (directionFactor));
}


void Drivebase::setPercentOutput(double leftVBus, double rightVBus) {
    dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, leftVBus);
    dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightVBus);
}

// checks deque contents to see if all values are within the given tolerance (true)
bool Drivebase::allValuesWithin(std::deque<double> deque, double tolerance) {
    bool hasOutlier = false;
    for (size_t i = 0; i < deque.size(); i++) {
        if (std::abs(deque[i]) > tolerance) {
            hasOutlier = true;
        }
    }
    return !hasOutlier;
}

void Drivebase::sendStartPointChooser() {
    startPointChooser.AddOption("(0, 0), 0 deg", getPose(0, 0, 0));
    startPointChooser.AddOption("(3, 1), 90 deg", getPose(3, 1, 90));
    frc::SmartDashboard::PutData("Select Start Point:", &startPointChooser);
}


void Drivebase::calcTurretEjectAngle(DrivebaseData &drivebaseData) {
    if (drivebaseData.odometryX <= 8.23) {
        double diffX = 0 - drivebaseData.odometryX;
        double diffY = 4.115 - drivebaseData.odometryY;
        drivebaseData.turretEjectAngle = (std::atan(diffY / diffX) * 180 / M_PI) - 180;
    } else {
        double diffX = 16.46 - drivebaseData.odometryX;
        double diffY = 4.115 - drivebaseData.odometryY;
        drivebaseData.turretEjectAngle = (std::atan(diffY / diffX) * 180 / M_PI);
    }
}



//BENCH TEST CODE
/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * BENCH TEST CODE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 **/

void Drivebase::TestPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData){
    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase && (robotData.controlData.manualBenchTest || robotData.controlData.autoBenchTest)){ //checks if we're testing drivebase
        if (robotData.benchTestData.stage == 0){
            //move right motors forwards
            dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, robotData.benchTestData.currentSpeed); //sets the right side speed
            dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0); //sets the left side speed
        } else if (robotData.benchTestData.stage == 1){
            //move right motors backwards
            dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -robotData.benchTestData.currentSpeed);
            dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        } else if (robotData.benchTestData.stage == 2){
            //move left motors forwards
            dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, robotData.benchTestData.currentSpeed);
        } else if (robotData.benchTestData.stage == 3){
            //move left motors backwards
            dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -robotData.benchTestData.currentSpeed);
        } else {
            dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        }
    } else {
        dbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0); //if not testing drivebase, then speeds get set to 0
        dbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
}