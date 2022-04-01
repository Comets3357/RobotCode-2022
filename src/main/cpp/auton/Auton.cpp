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
    
    autonChooser.AddOption("fiveBallC", "fiveBallC");
    autonChooser.AddOption("fiveBallCAlt", "fiveBallCAlt");

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
        taxiShootA(robotData, controlData);
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
    else if (autonData.autonRoutineName == "fiveBallC") {
        fiveBallC(robotData, controlData);
    }
    else if (autonData.autonRoutineName == "fiveBallCAlt") {
        fiveBallC(robotData, controlData);
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
    double sec = robotData.timerData.secSinceEnabled;

    // intake
    if (sec > 0 && sec < 4) {
        controlData.saIntake= true;
    } else {
        controlData.saIntake = false;
    }

    // shooting
    if (sec > 3 && sec < 7) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    if (sec > 5 && sec < 7) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::taxiShootA(const RobotData &robotData, ControlData &controlData) {
    double sec = robotData.timerData.secSinceEnabled;

    // intake
    if (sec > 0 && sec < 4) {
        controlData.saIntake = true;
    } else if (sec > 5.5 && sec < 8) {

    } else {
        controlData.saIntake = false;
    }

    // shooting
    // if (sec > 0 && sec < 14.5) {
        controlData.shootMode = shootMode_vision;
    // } else {
        // controlData.shootMode = shootMode_none;
    // }

    if (sec > 2.5 && sec < 6) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }

    // eject
    if (sec > 8 && sec < 10) {
        controlData.saEjectBalls = true;
    } else {
        controlData.saEjectBalls = false;
    }
}

void Auton::threeBallB(const RobotData &robotData, ControlData &controlData) {
    double sec = robotData.timerData.secSinceEnabled;

    // intake
    if (sec < 13) {
        controlData.saIntake = true;
    }
    
    // run flywheel and aim
    if (sec > 1.3 && sec < 5.3) {
        controlData.shootMode = shootMode_vision;
    } else if (sec > 11.5 && sec < 15) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    // final shoot
    if (sec > 3.3 && sec < 5.3) {
        controlData.saFinalShoot = true;
    } else if (sec > 13.5 && sec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::threeBallC(const RobotData &robotData, ControlData &controlData) {
    double sec = robotData.timerData.secSinceEnabled;

    // intake
    controlData.saIntake = true;

    // run flywheel and aim
    if (sec > 3 && sec < 8) {
        controlData.shootMode = shootMode_vision;
    } else if (sec > 10 && sec < 15) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }

    // final shoot
    if (sec > 5 && sec < 8) {
        controlData.saFinalShoot = true;
    } else if (sec > 14 && sec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::fourBallC(const RobotData &robotData, ControlData &controlData) {
    double sec = robotData.timerData.secSinceEnabled;

    // intake
    if (sec < 11) {
        controlData.saIntake = true;
    } else {
        controlData.saIntake = false;
    }

    // run flywheel and aim
    if (sec > 1 && sec < 4.5) {
        controlData.shootMode = shootMode_vision;
    } else if (sec > 12 && sec < 15) {
        controlData.shootMode = shootMode_vision;
    } else {
        controlData.shootMode = shootMode_none;
    }


    // final shoot
    if (sec > 3.2 && sec < 4.5) {
        controlData.saFinalShoot = true;
    } else if (sec > 13.5 && sec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }
}

void Auton::fiveBallC(const RobotData &robotData, ControlData &controlData)
{
    double sec = robotData.timerData.secSinceEnabled;

    // intake
    if (sec > 0.5 && sec < 5) {
        controlData.saIntake = true;
    } else if (sec > 8 && sec < 13) {
        controlData.saIntake = true;
    } else {
        controlData.saIntake = false;
    }
    // controlData.saIntake = false;

    // aim
    // if (sec > 0 && sec < 14) {
        controlData.shootMode = shootMode_vision;
    // } else {
    //     controlData.shootMode = shootMode_none;
    // }
    

    // final shoot
    if (sec > 0 && sec < 1) {
        controlData.saFinalShoot = true;
    } else if (sec > 3.5 && sec < 7.5) {
        controlData.saFinalShoot = true;
    } else if (sec > 13 && sec < 15) {
        controlData.saFinalShoot = true;
    } else {
        controlData.saFinalShoot = false;
    }

}

void Auton::sixBallC(const RobotData &robotData, ControlData &controlData) {}