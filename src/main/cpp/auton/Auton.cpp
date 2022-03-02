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
    for (int i = 0; i < autonData.pathGroup.size(); i++) {
        std::string correctPathName = autonData.pathGroup[i];

        // frc::SmartDashboard::PutBoolean("int bool" + std::to_string(i), correctPathName[correctPathName.length() - 1] == 13);
        // frc::SmartDashboard::PutNumber(std::to_string(i) + "int", correctPathName[correctPathName.length() - 1]);

        // if the last char in the string is a newline, delete it for proper auton selection processing
        if (int(correctPathName[correctPathName.length() - 1]) == 13) {
            correctPathName = correctPathName.substr(0, correctPathName.length() - 1);  // get rid of hidden newline from file line read
        }
        
        autonData.pathGroup[i] = correctPathName;

        frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);
        
        
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

    autonChooser.AddOption("sixBallC", "sixBallC");

    autonChooser.AddOption("test", "test");

    frc::SmartDashboard::PutData("Select Auton:", &autonChooser);
}


void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControlData &controlData)
{
    frc::SmartDashboard::PutString("autonRoutineName", autonData.autonRoutineName);

    if (autonData.autonRoutineName == "potato")
    {
        potato(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "exitShootA") {
        exitShoot(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "exitShootB") {
        exitShoot(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "exitShootC") {
        exitShoot(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "threeBallB") {
        threeBallB(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "fourBallC") {
        fourBallC(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "sixBallC") {
        sixBallC(robotData, controlData);
    }
}


void Auton::potato(const RobotData &robotData, ControlData &controlData)
{
}


void Auton::exitShoot(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    if (currentSec > 0 && currentSec < 4) {
        controlData.saIntake= true;
    } else {
        controlData.saIntake = false;
    }

    // shooting
    if (currentSec > 3 && currentSec < 9) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    if (currentSec > 5) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::threeBallA(const RobotData &robotData, ControlData &controlData) {}

void Auton::threeBallB(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    controlData.saIntake= true;

    // shooting
    if (currentSec > 2 && currentSec < 7) {
        controlData.shootMode = shootMode_vision;
    } else if (currentSec > 10) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    if (currentSec > 4) {
        controlData.saFinalShoot = true;
    } else if (currentSec > 12) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::threeBallC(const RobotData &robotData, ControlData &controlData) {}


void Auton::fourBallA(const RobotData &robotData, ControlData &controlData) {}

void Auton::fourBallB(const RobotData &robotData, ControlData &controlData) {}

void Auton::fourBallC(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    controlData.saIntake = true;

    // run flywheel and aim
    if (currentSec > 1 && currentSec < 6) {
        controlData.shootMode = shootMode_vision;
    } else if (currentSec > 10 && currentSec < 15) {
        controlData.shootMode = shootMode_vision;
    }
    else {
        controlData.shootMode = shootMode_none;
    }

    // final shoot
    if (currentSec > 3 && currentSec < 6) {
        controlData.saFinalShoot = true;
    } else if (currentSec > 13 && currentSec < 15) {
        controlData.saFinalShoot = true;
    }else {
        controlData.saFinalShoot = false;
    }
}

void Auton::sixBallC(const RobotData &robotData, ControlData &controlData) {}


// can only handle times to the 0.1 second precision
// void Auton::setShootTime(const RobotData &robotData, ControlData &controlData, double start, double end) {
        
//     double currentSec = robotData.timerData.secSinceEnabled;

//     if (currentSec > start - 2 && currentSec < end) {
//         controlData.shootMode = shootMode_vision;
//     }

//     if (currentSec > start && currentSec < start + 3) {
//         controlData.saFinalShoot = true;
//     }
// }