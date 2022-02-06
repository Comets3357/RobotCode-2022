#include "auton/Auton.h"
#include "RobotData.h"

void Auton::RobotInit(AutonData &autonData) {
    sendAutonSelectionChooser();
}

// creates pathGroup vector (list of strings that are interpretted by drivebase)
void Auton::AutonomousInit(AutonData &autonData)
{
    autonData.autonStep = -1;   // starts at -1 so that the stepper function can advance it to index 0 the first time

    // directory to deploy folder on roborio
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

    autonData.autonRoutineName = autonChooser.GetSelected();
    fs::path autonDirectory = deployDirectory / "Autos" / autonData.autonRoutineName;
    // frc::SmartDashboard::PutString("autonDiredctory", autonDirectory.string());

    std::ifstream inFile;
    inFile.open(autonDirectory.string());

    autonData.pathGroup.clear();

    if (inFile.fail()) {
       frc::SmartDashboard::PutString("fail", "failed");
    } else {
        std::string str;
        while (getline(inFile, str)) {
            // frc::SmartDashboard::PutString("str", str);
            autonData.pathGroup.push_back(str);
        }
    }

    // remove newline char from all but the final line
    for (int i = 0; i < autonData.pathGroup.size() - 1; i++) {
        std::string correctPathName = autonData.pathGroup[i];
        // if (i == 0) { frc::SmartDashboard::PutString("correctPathName", correctPathName); }
        // wpi::outs() << "ASDFGHJKL" + correctPathName;
        correctPathName = correctPathName.substr(0, correctPathName.length() - 1);  // get rid of hidden newline from file line read
        autonData.pathGroup[i] = correctPathName;
        // frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);
    }
}

void Auton::sendAutonSelectionChooser() {
    autonChooser.AddOption("potato", "potato");

    autonChooser.AddOption("exitShootA", "exitShootA");
    autonChooser.AddOption("exitShootB", "exitShootB");
    autonChooser.AddOption("exitShootC", "exitShootC");

    autonChooser.AddOption("threeBallA", "threeBallA");
    autonChooser.AddOption("threeBallB", "threeBallB");
    autonChooser.AddOption("threeBallC", "threeBallC");

    autonChooser.AddOption("fourBallA", "fourBallA");
    autonChooser.AddOption("fourBallB", "fourBallB");
    autonChooser.AddOption("fourBallC", "fourBallC");

    frc::SmartDashboard::PutData("Select Auton:", &autonChooser);
}

frc::Pose2d Auton::getPose(double x, double y, double deg) {
    units::meter_t meterX{x};
    units::meter_t meterY{y};

    const units::radian_t radianYaw{deg / 180 * M_PI};
    const frc::Rotation2d rotation{radianYaw};
    frc::Pose2d pose{meterX, meterY, rotation};
    return pose;
}

void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControlData &controlData)
{
    if (autonData.autonRoutineName == "potato")
    {
        potato(robotData, controlData);
    }
}


void Auton::potato(const RobotData &robotData, ControlData &controlData)
{
}


void exitShootA(const RobotData &robotData, ControlData &controlData) {
    double sec = robotData.timerData.secSinceEnabled;

    
}

void exitShootB(const RobotData &robotData, ControlData &controlData) {}

void exitShootC(const RobotData &robotData, ControlData &controlData) {}


void threeBallA(const RobotData &robotData, ControlData &controlData) {}

void threeBallB(const RobotData &robotData, ControlData &controlData) {}

void threeBallC(const RobotData &robotData, ControlData &controlData) {}


void fourBallA(const RobotData &robotData, ControlData &controlData) {}

void fourBallB(const RobotData &robotData, ControlData &controlData) {}

void fourBallC(const RobotData &robotData, ControlData &controlData) {}