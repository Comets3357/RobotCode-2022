#include "RobotData.h"

void BenchTest::TestPeriodic(const RobotData &robotData, BenchTestData &benchTestData, const ControlData &controlData){ //beep boop
    frc::SmartDashboard::PutNumber("Auto bench test increment", increment);
    frc::SmartDashboard::PutBoolean("Start bench test", robotData.controlData.manualBenchTest);
    frc::SmartDashboard::PutBoolean("Auto bench test", robotData.controlData.autoBenchTest);
    frc::SmartDashboard::PutNumber("MotorStage", robotData.benchTestData.stage); //prints the motor stage
    frc::SmartDashboard::PutNumber("BenchTestStage", robotData.benchTestData.testStage); //prints the subsystem we're currently on
    frc::SmartDashboard::PutNumber("Power", robotData.benchTestData.currentSpeed); //prints the current testing speed


    //AUTO BENCH TEST

    if (controlData.autoBenchTest){
        //increments motor every 4 seconds (unless the motor has limits/dead stops)
        //this could all be on one line but I tried that and it made my brain hurt so I broke it up into 3 separate if statements
        if (benchTestData.testStage != BenchTestStage::BenchTestStage_Climb){
            if (benchTestData.testStage != BenchTestStage::BenchTestStage_Intake && benchTestData.testStage != BenchTestStage::BenchTestStage_Shooter){
                increment += .005; //if it's not climb, intake, or shooter, then the automatic bench test increments based on time
            } else if (benchTestData.stage != 0 && benchTestData.stage != 1 && (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && benchTestData.stage != 6 && benchTestData.stage != 7) && (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && benchTestData.stage != 3 && benchTestData.stage != 4)){
                increment += .005; //if it's intake or shooter but it isn't pivoting the intake or moving the hood in or out, then the automatic bench tests increments based on time
            }
        }

        if (increment == 1){
            increment = 0;
            benchTestData.stage++;
        }

        //if the final motor in a subsystem is reached, and the bench test is in auto mode, then the subsystem increments
        //additionally, if it reaches the end of shooter, instead of looping back to climb like manual, it makes it an arbitrary number to stop running bench test
        if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && benchTestData.stage >= 8){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Drivebase;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && benchTestData.stage >= 5){
            benchTestData.testStage = 6;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && benchTestData.stage >= 8){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Shooter;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && benchTestData.stage >= 4){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Intake;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase && benchTestData.stage >= 4){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Indexer;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage >= 5){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Shooter;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage >= 3){
            benchTestData.testStage = 6;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage >= 4){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Drivebase;
        }

        //if the bench test hits a dead stop, then increment the stage
        if (controlData.autoBenchTest && (robotData.climbData.armsLowerLimit || robotData.climbData.armsUpperLimit || robotData.climbData.lowerLimit || robotData.climbData.upperLimit || robotData.intakeData.topDeadStop || robotData.intakeData.bottomDeadStop || robotData.shooterData.topDeadStop || robotData.shooterData.bottomDeadStop)){
            benchTestData.stage++;
        }
    }


    //MISCELLANEOUS BENCH TEST


    //toggles the testing of PIDs, which will run the robot to its limits
    if (controlData.PIDModeToggle){
        benchTestData.PIDMode = !benchTestData.PIDMode;
    }


    //MANUAL BENCH TEST


    if (controlData.manualBenchTest){  //starts testing to see if the robot is mechanically sound (toggle with A button on/off)
        if (controlData.incrementSpeed){ //if the button is pressed, then speed goes up by .1
            if (benchTestData.currentSpeed >= .7){ //caps the speed so it doesn't just infinitely go up
                benchTestData.currentSpeed = 0;
            } else {
                benchTestData.currentSpeed += .1; //speed goes up by .1
            }
        }

        //changes the motor that's currently being tested
        if (controlData.incrementMotor){
            benchTestData.currentSpeed = 0; //sets the speed to 0 when incrementing motors
            benchTestData.stage++; //if the subsystem doesn't need to be incremented, then the motor stage is instead

            //if the final motor in a subsystem is reach, then the cycle resets to go through the motor sequence again
            if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && benchTestData.stage >= 8){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && benchTestData.stage >= 5){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && benchTestData.stage >= 8){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && benchTestData.stage >= 4){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase && benchTestData.stage >= 4){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage >= 5){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage >= 3){
                benchTestData.stage = 0;
            } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && !robotData.benchTestData.PIDMode && robotData.benchTestData.stage >= 4){
                benchTestData.stage = 0;
            }
        }

        if (controlData.incrementSubsystem){ //increments the subsystem with B button
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
}