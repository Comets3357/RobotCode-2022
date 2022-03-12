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
    //    frc::SmartDashboard::PutString("fail", "failed");
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

        // frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);
        
        
        // frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);        
    }
}

void Auton::sendAutonSelectionChooser() {
    autonChooser.AddOption("potato", "potato");

    autonChooser.AddOption("taxiShootA", "taxiShootA");
    autonChooser.AddOption("taxiShootB", "taxiShootB");
    autonChooser.AddOption("taxiShootC", "taxiShootC");

    autonChooser.AddOption("threeBallB", "threeBallB");
    autonChooser.AddOption("threeBallC", "threeBallC");

    autonChooser.AddOption("fourBallC", "fourBallC");
    autonChooser.AddOption("fourBallCHP", "fourBallCHP");

    autonChooser.AddOption("sixBallC", "sixBallC");

    autonChooser.AddOption("test", "test");

    frc::SmartDashboard::PutData("Select Auton:", &autonChooser);
}


void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControlData &controlData)
{
    // frc::smartDashboard::PutString("autonRoutineName", autonData.autonRoutineName);

    if (autonData.autonRoutineName == "potato")
    {
        potato(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "taxiShootA") {
        taxiShoot(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "taxiShootB") {
        taxiShoot(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "taxiShootC") {
        taxiShoot(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "threeBallB") {
        threeBallB(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "threeBallC") {
        threeBallC(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "fourBallC") {
        fourBallC(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "fourBallCHP") {
        fourBallCHP(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "sixBallC") {
        sixBallC(robotData, controlData);
    }
}


void Auton::potato(const RobotData &robotData, ControlData &controlData)
{
    controlData.saIntake = false;
}


void Auton::taxiShoot(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    if (currentSec > 0 && currentSec < 4) {
        controlData.saIntake= true;
    } else {
        controlData.saIntake = false;
    }

    // shooting
    if (currentSec > 3 && currentSec < 7) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    if (currentSec > 5 && currentSec < 7) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::threeBallB(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    controlData.saIntake = true;

    // run flywheel and aim
    if (currentSec > 3 && currentSec < 8) {
        controlData.shootMode = shootMode_vision;
    } else if (currentSec > 10 && currentSec < 15) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    // final shoot
    if (currentSec > 5 && currentSec < 8) {
        controlData.saFinalShoot = true;
    } else if (currentSec > 14 && currentSec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::threeBallC(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    controlData.saIntake = true;

    // run flywheel and aim
    if (currentSec > 3 && currentSec < 8) {
        controlData.shootMode = shootMode_vision;
    } else if (currentSec > 10 && currentSec < 15) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    // final shoot
    if (currentSec > 5 && currentSec < 8) {
        controlData.saFinalShoot = true;
    } else if (currentSec > 14 && currentSec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::fourBallC(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    if (currentSec < 11) {
        controlData.saIntake = true;
    } else {
        controlData.saIntake = false;
    }

    // run flywheel and aim
    if (currentSec > 1 && currentSec < 4.5) {
        controlData.shootMode = shootMode_vision;
    } else if (currentSec > 12 && currentSec < 15) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }


    // final shoot
    if (currentSec > 3.2 && currentSec < 4.5) {
        controlData.saFinalShoot = true;
    } else if (currentSec > 13.5 && currentSec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}


void Auton::fourBallCHP(const RobotData &robotData, ControlData &controlData) {
    double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    if (currentSec < 11) {
        controlData.saIntake = true;
    } else {
        controlData.saIntake = false;
    }

    // run flywheel and aim
    if (currentSec > 1 && currentSec < 4.5) {
        controlData.shootMode = shootMode_vision;
    } else if (currentSec > 12 && currentSec < 15) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }


    // final shoot
    if (currentSec > 3.2 && currentSec < 4.5) {
        controlData.saFinalShoot = true;
    } else if (currentSec > 13.5 && currentSec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}


void Auton::sixBallC(const RobotData &robotData, ControlData &controlData) {}