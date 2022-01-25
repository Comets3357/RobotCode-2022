#include "auton/Auton.h"
#include "RobotData.h"

void Auton::AutonomousInit(AutonData &autonData)
{
    // get selected auton from smartdashboard
    // import pathweaver json
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    // fs::path pathDirectory = deployDirectory / "paths" / "barrelRace3.wpilib.json";
    // frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(pathDirectory.string());

    // frc::SmartDashboard::PutString("deployDirectory.string()", deployDirectory.string());
    // wpi::outs() << "done in auton.cpp";

    // because getTrajectoryFile() steps autonStep
    autonData.autonStep = -1;

    // CHANGE THIS STRING AT THE END OF THE PATH TO CHANGE ROUTINE SELECTION
    fs::path autonDirectory = deployDirectory / "Autos" / "sequence";
    frc::SmartDashboard::PutString("autonDiredctory", autonDirectory.string());

    // std::vector<std::string> pathGroup;

    std::ifstream inFile/* ("file.txt") */;
    inFile.open(autonDirectory.string());

    if (inFile.fail()) {
       frc::SmartDashboard::PutString("fail", "failed");
    } else {
        std::string str;
        while (getline(inFile, str)) {
            frc::SmartDashboard::PutString("str", str);
            str = str.substr(0, str.length() - 1);  // get rid of hidden newline from file line read
            autonData.pathGroup.push_back(str);
        }
    }

    for (int i = 0; i < autonData.pathGroup.size(); i++) {
        frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);
    }

    frc::SmartDashboard::PutString("Auton Auto Init", "done");
    frc::SmartDashboard::PutString("autonData.pathGroup[autonData.pathGroup[0]", autonData.pathGroup[0]);
    // frc::SmartDashboard::PutString("autonData.pathGroup[autonData.autonStep]", autonData.pathGroup[autonData.autonStep]);

    // autonData.trajectory = trajectory;
    // autonData.pathGroup = &pathGroup;

    // autonData.pathGroup = pathGroup;
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
    case autonSelect_barrelRace:
        barrelRace(robotData, controllerData);
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

void Auton::barrelRace(const RobotData &robotData, ControllerData &controllerData) {

}