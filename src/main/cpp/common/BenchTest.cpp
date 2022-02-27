#include "RobotData.h"

void BenchTest::TestPeriodic(const RobotData &robotData, BenchTestData &benchTestData){
    frc::SmartDashboard::PutBoolean("Start bench test", robotData.controlData.startBenchTest);
    frc::SmartDashboard::PutNumber("MotorStage", robotData.benchTestData.stage); //prints the motor stage
    frc::SmartDashboard::PutNumber("BenchTestStage", robotData.benchTestData.testStage); //prints the subsystem we're currently on
    frc::SmartDashboard::PutNumber("Power", robotData.benchTestData.currentSpeed); //prints the current testing speed

    if (robotData.controlData.startBenchTest){  //starts testing to see if the robot is mechanically sound (toggle with A button on/off)

        if (robotData.controlData.incrementSpeed){ //if the button is pressed, then speed goes up by .1
            if (benchTestData.currentSpeed >= .7){ //caps the speed so it doesn't just infinitely go up
                benchTestData.currentSpeed = 0;
            } else {
                benchTestData.currentSpeed += .1; //speed goes up by .1
            }
        }

        if (robotData.controlData.PIDModeToggle){
            benchTestData.PIDMode = !benchTestData.PIDMode; //this variable should automatically reset between subsystems
        }

        //changes the motor that's currently being tested
        if (robotData.controlData.incrementMotor){
            benchTestData.stage++; //if the subsystem doesn't need to be incremented, then the motor stage is instead
            benchTestData.PIDMode = false; //sets the pid mode to false when incrementing motors
            benchTestData.currentSpeed = 0; //sets the speed to 0 when incrementing motors

            //if the final motor in a subsystem is reach, then the cycle resets to go through the motor sequence again
            if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && benchTestData.stage >= 4){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && benchTestData.stage >= 6){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && benchTestData.stage >= 9){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && benchTestData.stage >= 4){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase && benchTestData.stage >= 4){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage >= 6){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage > 3){
                benchTestData.stage = 0;
            }
        }
    } else {
        benchTestData.currentSpeed = 0; //sets the speed to 0 if the user needs to quickly stop testing
    }

    if (robotData.controlData.incrementSubsystem){ //increments the subsystem with B button
        if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Indexer; //increments subsystem
            benchTestData.stage = 0;  //sets the motor stage to 0 (so no motors are skipped)
            benchTestData.currentSpeed = 0; //sets the speed to 0 (so mototrs don't go flying from the start)
            benchTestData.PIDMode = false;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Intake;
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
            benchTestData.PIDMode = false;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Shooter;
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
            benchTestData.PIDMode = false;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Climb; //resets back to climb in case you want to test that again (bench test starts in climb)
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
            benchTestData.PIDMode = false;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Drivebase;
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
            benchTestData.PIDMode = false;
        }
    }
}