#include "auton/Auton.h"
#include "RobotData.h"

void Auton::RobotInit(AutonData &autonData) {
    sendAutonSelectionChooser();
    sendStartPointChooser();
}

void Auton::AutonomousInit(AutonData &autonData)
{
    autonData.autonStep = -1;

    // get selected auton from smartdashboard
    // import pathweaver json
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    // fs::path pathDirectory = deployDirectory / "paths" / "barrelRace3.wpilib.json";
    // frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(pathDirectory.string());

    // frc::SmartDashboard::PutString("deployDirectory.string()", deployDirectory.string());
    // wpi::outs() << "done in auton.cpp";

    // because getTrajectoryFile() steps autonStep

    // CHANGE THIS STRING AT THE END OF THE PATH TO CHANGE ROUTINE SELECTION
    fs::path autonDirectory = deployDirectory / "Autos" / "potato";
    frc::SmartDashboard::PutString("autonDiredctory", autonDirectory.string());

    // std::vector<std::string> pathGroup;

    std::ifstream inFile/* ("file.txt") */;
    inFile.open(autonDirectory.string());

    autonData.pathGroup.clear();

    if (inFile.fail()) {
       frc::SmartDashboard::PutString("fail", "failed");
    } else {
        std::string str;
        while (getline(inFile, str)) {
            frc::SmartDashboard::PutString("str", str);
            autonData.pathGroup.push_back(str);
        }
    }

    // remove newline char from all but the final line
    for (int i = 0; i < autonData.pathGroup.size() - 1; i++) {
        std::string correctPathName = autonData.pathGroup[i];
        if (i == 0) { frc::SmartDashboard::PutString("correctPathName", correctPathName); }
        // wpi::outs() << "ASDFGHJKL" + correctPathName;
        correctPathName = correctPathName.substr(0, correctPathName.length() - 1);  // get rid of hidden newline from file line read
        autonData.pathGroup[i] = correctPathName;
        frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);
    }

    frc::SmartDashboard::PutString("Auton Auto Init", "done");
    frc::SmartDashboard::PutString("autonData.pathGroup[autonData.pathGroup[0]", autonData.pathGroup[0]);
    // frc::SmartDashboard::PutString("autonData.pathGroup[autonData.autonStep]", autonData.pathGroup[autonData.autonStep]);

    // autonData.trajectory = trajectory;
    // autonData.pathGroup = &pathGroup;

    // autonData.pathGroup = pathGroup;

    autonData.autonSelect = autonChooser.GetSelected();
    frc::SmartDashboard::PutNumber("autonSelect", autonData.autonSelect);

    autonData.startPoint = startPointChooser.GetSelected();
    frc::SmartDashboard::PutNumber("autonSelect", autonData.startPoint.X().to<double>());
}

void Auton::sendAutonSelectionChooser() {
    autonChooser.AddOption("Potato", AutonSelect::autonSelect_potato);
    autonChooser.AddOption("Exit Init Line Towards Driver Station", AutonSelect::autonSelect_driveStraight);
    frc::SmartDashboard::PutData("Select Auton:", &autonChooser);
}

void Auton::sendStartPointChooser() {
    startPointChooser.AddOption("(0, 0), 0 deg", getPose(0, 0, 0));
    startPointChooser.AddOption("(3, 1), 90 deg", getPose(3, 1, 90));
    frc::SmartDashboard::PutData("Select Start Point:", &startPointChooser);
}

frc::Pose2d Auton::getPose(double x, double y, double deg) {
    units::meter_t meterX{x};
    units::meter_t meterY{y};

    const units::radian_t radianYaw{deg / 180 * M_PI};
    const frc::Rotation2d rotation{radianYaw};
    frc::Pose2d pose{meterX, meterY, rotation};
    return pose;
}

void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControllerData &controllerData)
{
    switch (autonData.autonSelect)
    {
    case autonSelect_potato:
        potato(robotData, controllerData);
        break;
    case autonSelect_driveStraight:
        driveStraight(robotData, controllerData);
        break;
    case autonSelect_intakeAlternate:
        intakeAlternate(robotData, controllerData);
        break;
    default:
        break;
    }
}

void Auton::potato(const RobotData &robotData, ControllerData &controllerData)
{
}

void Auton::driveStraight(const RobotData &robotData, ControllerData &controllerData)
{
}

void Auton::intakeAlternate(const RobotData &robotData, ControllerData &controllerData)
{
    double seconds = robotData.timerData.secSinceEnabled;

    if (std::lround(seconds) % 2 == 0)
    {
        controllerData.sRTrigger = true;
    }
    else
    {
        controllerData.sRTrigger = false;
    }
}