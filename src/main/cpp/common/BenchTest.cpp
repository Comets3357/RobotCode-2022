#include "RobotData.h" //beep boop

void BenchTest::TestPeriodic(const RobotData &robotData, BenchTestData &benchTestData, const ControlData &controlData){
    frc::SmartDashboard::PutNumber("Auto bench test increment", increment); //time-based increment for auto bench test
    frc::SmartDashboard::PutBoolean("Manual bench test", robotData.controlData.manualBenchTest); //manual bench test
    frc::SmartDashboard::PutBoolean("Auto bench test", robotData.controlData.autoBenchTest); //auto bench test
    frc::SmartDashboard::PutNumber("Bench test motor stage", robotData.benchTestData.stage); //prints the motor stage
    frc::SmartDashboard::PutNumber("Bench test subsystem stage", benchTestData.testStage); //prints the subsystem we're currently on
    frc::SmartDashboard::PutNumber("Bench test power", robotData.benchTestData.currentSpeed); //prints the current testing speed


    //AUTOMATIC BENCH TEST


    if (controlData.autoBenchTest){

        //janky solution to skipping drivebase; basically at the end of climb, we set stage to -1, and then here to 0, which stops it from
        //skipping drivebase. weird solution, kinda janky, not how it should work, but it works, so cool
        if (benchTestData.stage < 0){
            benchTestData.stage = 0;
        }

        //increments motor every 4 seconds (unless the motor has limits/dead stops)
        //this could all be on one line but I tried that and it made my brain hurt so I broke it up into 3 separate if statements
        if (benchTestData.testStage != BenchTestStage::BenchTestStage_Climb){
            if (benchTestData.testStage != BenchTestStage::BenchTestStage_Intake && benchTestData.testStage != BenchTestStage::BenchTestStage_Shooter){
                increment += .005; //if it's not climb, intake, or shooter, then the automatic bench test increments based on time
            } else if (benchTestData.stage != 0 && benchTestData.stage != 1){
                increment += .005; //if it's intake or shooter but it isn't pivoting the intake or moving the hood in or out, then the automatic bench tests increments based on time
            }
        }

        //sets the speed based on the time; starts slow, and speeds up every second (diff values for indexer)
        if (benchTestData.testStage != BenchTestStage::BenchTestStage_Indexer){
            if (increment <= .25){
                benchTestData.currentSpeed = .1;
            } else if (increment <= .5){
                benchTestData.currentSpeed = .2;
            } else if (increment <= .75){
                benchTestData.currentSpeed = .3;
            } else if (increment <= 1){
                benchTestData.currentSpeed = .4;
            }
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer){
            if (increment <= .5){
                benchTestData.currentSpeed = .1;
            } else if (increment <= 1){
                benchTestData.currentSpeed = .2;
            }
        }

        //resets the time increment when moving between motors
        if (increment > 1){
            increment = 0;
            benchTestData.stage++;
        }

        //increments the motors at dead stops
        if (robotData.climbData.armsLowerLimit){
            benchTestData.stage = 1;
        } else if (robotData.climbData.armsUpperLimit){
            benchTestData.stage = 2;
        } else if (robotData.climbData.upperLimit){
            benchTestData.stage = 3;
        } else if (robotData.climbData.lowerLimit){
            benchTestData.stage = -1;
        } else if (robotData.intakeData.bottomDeadStop){
            benchTestData.stage = 1;
        } else if (robotData.intakeData.topDeadStop){
            benchTestData.stage = 2;
        } else if (robotData.shooterData.topDeadStop){
            benchTestData.stage = 1;
        } else if (robotData.shooterData.bottomDeadStop){
            benchTestData.stage = 2;
        }

        //if the final motor in a subsystem is reached, and the bench test is in auto mode, then the subsystem increments
        //additionally, if it reaches the end of shooter, instead of looping back to climb like manual, it makes it an arbitrary number to stop running bench test
        if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && benchTestData.stage == -1){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Drivebase;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && benchTestData.stage >= 3){
            benchTestData.stage = 0;
            benchTestData.testStage = 6;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && benchTestData.stage >= 6){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Shooter;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && benchTestData.stage >= 4){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Intake;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase && benchTestData.stage >= 4){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Indexer;
        }
    }


    //MISCELLANEOUS BENCH TEST


    //toggles the testing of PIDs, which will run the robot to its limits
    if (controlData.PIDModeToggle){
        benchTestData.PIDMode = !benchTestData.PIDMode;
    }


    //MANUAL BENCH TEST


    if (controlData.manualBenchTest){  //starts testing to see if the robot is mechanically sound (toggle with A button on/off)

        //increments the speed of motors when testing
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
            }
        }

        //increments the subsystem with B button
        if (controlData.incrementSubsystem){
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