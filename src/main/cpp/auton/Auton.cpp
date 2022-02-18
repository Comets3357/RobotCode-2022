#include "auton/Auton.h"
#include "RobotData.h"

void Auton::RobotInit(AutonData &autonData) {
    sendAutonSelectionChooser();
}

// creates pathGroup vector (list of strings that are interpretted by drivebase)
void Auton::AutonomousInit(AutonData &autonData)
{
    // autonData.autonStep = -1;   // starts at -1 so that the stepper function can advance it to index 0 the first time

    // // directory to deploy folder on roborio
    // fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

    // autonData.autonRoutineName = autonChooser.GetSelected();
    // fs::path autonDirectory = deployDirectory / "Autos" / autonData.autonRoutineName;
    // // frc::SmartDashboard::PutString("autonDiredctory", autonDirectory.string());

    // std::ifstream inFile;
    // inFile.open(autonDirectory.string());

    // autonData.pathGroup.clear();

    // if (inFile.fail()) {
    //    frc::SmartDashboard::PutString("fail", "failed");
    // } else {
    //     std::string str;
    //     while (getline(inFile, str)) {
    //         // frc::SmartDashboard::PutString("str", str);
    //         autonData.pathGroup.push_back(str);
    //     }
    // }

    // // remove newline char from all but the final line
    // for (int i = 0; i < autonData.pathGroup.size(); i++) {
    //     std::string correctPathName = autonData.pathGroup[i];

    //     // frc::SmartDashboard::PutBoolean("int bool" + std::to_string(i), correctPathName[correctPathName.length() - 1] == 13);
    //     // frc::SmartDashboard::PutNumber(std::to_string(i) + "int", correctPathName[correctPathName.length() - 1]);

    //     // if the last char in the string is a newline, delete it for proper auton selection processing
    //     if (int(correctPathName[correctPathName.length() - 1]) == 13) {
    //         correctPathName = correctPathName.substr(0, correctPathName.length() - 1);  // get rid of hidden newline from file line read
    //     }
        
    //     autonData.pathGroup[i] = correctPathName;

    //     frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);
        
        
    //     // frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);        
    // }
}

void Auton::sendAutonSelectionChooser() {
    // autonChooser.AddOption("potato", "potato");

    // autonChooser.AddOption("exitShootA", "exitShootA");
    // autonChooser.AddOption("exitShootB", "exitShootB");
    // autonChooser.AddOption("exitShootC", "exitShootC");

    // autonChooser.AddOption("threeBallA", "threeBallA");
    // autonChooser.AddOption("threeBallB", "threeBallB");
    // autonChooser.AddOption("threeBallC", "threeBallC");

    // autonChooser.AddOption("fourBallA", "fourBallA");
    // autonChooser.AddOption("fourBallB", "fourBallB");
    // autonChooser.AddOption("fourBallC", "fourBallC");

    // autonChooser.AddOption("test", "test");

    // frc::SmartDashboard::PutData("Select Auton:", &autonChooser);
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
    // frc::SmartDashboard::PutString("autonRoutineName", autonData.autonRoutineName);

    // if (autonData.autonRoutineName == "potato")
    // {
    //     potato(robotData, controlData);
    // }
    // else if (autonData.autonRoutineName == "exitShootA") {
    //     exitShootA(robotData, controlData);
    // }
    // else if (autonData.autonRoutineName == "fourBallC") {
    //     fourBallC(robotData, controlData);
    // }
}


void Auton::potato(const RobotData &robotData, ControlData &controlData)
{
}


void Auton::exitShootA(const RobotData &robotData, ControlData &controlData) {
    // double currentSec = robotData.timerData.secSinceEnabled;

    // // intake
    // controlData.saIntake = true;

    // // shooting
    // if (currentSec > 2 && currentSec < 2.1) {
    //     controlData.saShooting= true;
    // } else {
    //     controlData.saShooting = false;
    // }

    // if (currentSec > 2.5) {
    //     controlData.saFinalShoot = true;
    // }
    
}

void Auton::exitShootB(const RobotData &robotData, ControlData &controlData) {}

void Auton::exitShootC(const RobotData &robotData, ControlData &controlData) {}


void Auton::threeBallA(const RobotData &robotData, ControlData &controlData) {}

void Auton::threeBallB(const RobotData &robotData, ControlData &controlData) {}

void Auton::threeBallC(const RobotData &robotData, ControlData &controlData) {}


void Auton::fourBallA(const RobotData &robotData, ControlData &controlData) {}

void Auton::fourBallB(const RobotData &robotData, ControlData &controlData) {}

void Auton::fourBallC(const RobotData &robotData, ControlData &controlData) {
    // double currentSec = robotData.timerData.secSinceEnabled;

    // intake
    controlData.saIntake = true;

    // frc::SmartDashboard::PutString("FOURBALLC", "RUNNING");

    setShootTime(controlData, currentSec, 3, 5);
    setShootTime(controlData, currentSec, 10.5, 15);
}


// can only handle times to the 0.1 second precision
void Auton::setShootTime(ControlData &controlData, double currentSec, double startSec, double endSec) {
        

        // // only should be worrying about the relevant call of this function based on the time stamps in the call
        // if (currentSec > (startSec - 1) && currentSec < (endSec + 0.2)) {

            frc::SmartDashboard::PutNumber("currentSecShooting", currentSec);
            // toggle on vision shooting 1 sec before start of time range

        //     if (currentSec > startSec - 1 && currentSec < startSec - 0.9) {
        //         controlData.saShooting = true;
        //     }
        //     else if (currentSec > startSec - 0.9 && currentSec < startSec - 0.8) {
        //         controlData.saShooting = false;
        //     }

        //     if (currentSec > endSec && currentSec < endSec + 0.1) {
        //         controlData.saShooting = true;
        //     }
        //     else if (currentSec > endSec + 0.1 && currentSec < endSec + 0.2) {
        //         controlData.saShooting = false;
        //     }

            if (currentSec > startSec && currentSec < endSec) {
                controlData.saFinalShoot = true;
            } else {
                controlData.saFinalShoot = false;
            }
        }
}