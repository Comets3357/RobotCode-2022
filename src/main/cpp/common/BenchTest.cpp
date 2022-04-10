#include "RobotData.h" //beep boop

void BenchTest::TestInit(BenchTestData &benchTestData, ControlData &controlData){
    benchTestData.testStage = BenchTestStage::BenchTestStage_Climb; //sets the starting stage of the bench test
    benchTestData.currentSpeed = 0; //sets the speed of the motor we're currently testing
    benchTestData.stage = 0; //sets which motor we're currently testing
    benchTestData.PIDMode = false; //toggles pid testing
    controlData.autoBenchTest = false; //resets automatic bench test when enabling
    controlData.manualBenchTest = false; //resets manual bench test when enabling
}

void BenchTest::TestPeriodic(const RobotData &robotData, BenchTestData &benchTestData, ControlData &controlData){
    frc::SmartDashboard::PutBoolean("Bench test manual mode", robotData.controlData.manualBenchTest); //manual bench test
    frc::SmartDashboard::PutBoolean("Bench test automatic mode", robotData.controlData.autoBenchTest); //auto bench test
    frc::SmartDashboard::PutNumber("Bench test automatic increment", increment); //time-based increment for auto bench test - COMMENT OUT BEFORE COMP
    frc::SmartDashboard::PutNumber("Bench test motor stage", robotData.benchTestData.stage); //prints the motor stage
    frc::SmartDashboard::PutNumber("Bench test subsystem stage", benchTestData.testStage); //prints the subsystem we're currently on
    frc::SmartDashboard::PutNumber("Bench test motor increment power", robotData.benchTestData.currentSpeed); //prints the current testing speed (doesn't update for all motors)
    frc::SmartDashboard::PutBoolean("Bench test PID mode", benchTestData.PIDMode); //PID mode

    //toggles PID mode
    if (controlData.PIDModeToggle) benchTestData.PIDMode = !benchTestData.PIDMode;

    //changes the motor that's currently being tested
    if (controlData.incrementMotor && !controlData.autoBenchTest){
        benchTestData.currentSpeed = 0; //sets the speed to 0 when incrementing motors
        benchTestData.stage++; //if the subsystem doesn't need to be incremented, then the motor stage is instead

        //if the final motor in a subsystem is reach, then the cycle resets to go through the motor sequence again
        if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && benchTestData.stage >= 4) benchTestData.stage = 0;
        else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && benchTestData.stage >= 5) benchTestData.stage = 0;
        else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && benchTestData.stage >= 6) benchTestData.stage = 0;
        else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && benchTestData.stage >= 4) benchTestData.stage = 0;
        else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase && benchTestData.stage >= 4) benchTestData.stage = 0;
        else if (benchTestData.testStage == 5 && benchTestData.stage >= 4) benchTestData.stage = 0;
    }

    //increments the subsystem with right bumper button
    if (controlData.incrementSubsystem && !controlData.autoBenchTest){
        if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Indexer; //increments subsystem
            benchTestData.stage = 0;  //sets the motor stage to 0 (so no motors are skipped)
            benchTestData.currentSpeed = 0; //sets the speed to 0 (so motors don't go flying from the start)
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Intake;
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Shooter;
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter){
            benchTestData.testStage = 5;
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Drivebase;
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
        } else if (benchTestData.testStage == 5){
            benchTestData.testStage = BenchTestStage::BenchTestStage_Climb; //resets back to climb in case you want to test that again (bench test starts in climb)
            benchTestData.stage = 0;
            benchTestData.currentSpeed = 0;
        }
    }

    //MANUAL BENCH TEST
    if (controlData.manualBenchTest){  //starts bench test in manual mode (toggle with right bumper on/off)
        //increments the speed of motors when testing (speed starts at 0)
        if (controlData.incrementSpeed){ //if the button is pressed, then speed goes up by .1
            if (benchTestData.currentSpeed >= .7) benchTestData.currentSpeed = 0; //caps the speed so it doesn't just infinitely go up 
            else benchTestData.currentSpeed += .1; //speed goes up by .1
        }

        //stage 5 for testing LEDs
        if (benchTestData.testStage == 5){
            if (benchTestData.stage == 0) controlData.mode = Mode::mode_climb_manual;
            else if (benchTestData.stage == 1) controlData.mode = Mode::mode_climb_sa;
            else if (benchTestData.stage == 2) controlData.mode = Mode::mode_teleop_manual;
            else if (benchTestData.stage == 3) controlData.mode = Mode::mode_teleop_sa;
        }
    }

    //AUTOMATIC BENCH TEST
    if (controlData.autoBenchTest){
        //janky solution to skipping drivebase after climb: basically at the end of climb, stage gets set to -1, and then here to 0, which
        //stops it from skipping drivebase. weird solution, kinda janky, not how it should work, but it works, so cool
        if (benchTestData.stage < 0) benchTestData.stage = 0;

        //increments motor every 4 seconds (unless the motor has limits/dead stops)
        //I don't recommend actually reading this if statement unless you want your brain to hurt
        if (!(benchTestData.testStage == BenchTestStage::BenchTestStage_Climb) && !(benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && (benchTestData.stage == 0 || benchTestData.stage == 1)) && !(benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && (benchTestData.stage == 0 || benchTestData.stage == 1 || benchTestData.stage == 3 || benchTestData.stage == 4))){
            increment += .005; //if it's not climb, and it's not intake or shooter while pivoting the intake or moving the hood in or out or rotating the turret, then automatic bench test increments based on time
        }

        //sets the speed based on the time; starts slow, and speeds up every second
        if (increment <= .25) benchTestData.currentSpeed = .1;
        else if (increment <= .5) benchTestData.currentSpeed = .2;
        else if (increment <= .75) benchTestData.currentSpeed = .3;
        else if (increment <= 1) benchTestData.currentSpeed = .4;

        //miscellaneous stage 5 for LEDs (just runs through all of the different colors - should be enough to check if the arduino is working)
        if (benchTestData.testStage == 5 && benchTestData.stage == 0){
            if (increment <= .25) controlData.mode = Mode::mode_climb_manual;
            else if (increment <= .5) controlData.mode = Mode::mode_climb_sa;
            else if (increment <= .75) controlData.mode = Mode::mode_teleop_manual;
            else if (increment <= 1) controlData.mode = Mode::mode_teleop_sa;
        }

        //resets the time increment when moving between motors
        if (increment > 1){
            increment = 0;
            benchTestData.stage++;
        }

        //increments the motors at dead stops
        // if (robotData.climbData.armsLowerLimit) benchTestData.stage = 1;
        // else if (robotData.climbData.armsUpperLimit) benchTestData.stage = 2;
        // else if (robotData.climbData.elevatorUpperLimit) benchTestData.stage = 3;
        // else if (robotData.climbData.elevatorLowerLimit) benchTestData.stage = -1; //special case explained on lines 86-87
        // else if (robotData.intakeData.bottomDeadStop) benchTestData.stage = 1;
        // else if (robotData.intakeData.topDeadStop) benchTestData.stage = 2;
        // else if (robotData.shooterData.hoodTopDeadStop) benchTestData.stage = 1;
        // else if (robotData.shooterData.hoodBottomDeadStop) benchTestData.stage = 2;
        // else if (robotData.shooterData.turretBottomDeadStop) benchTestData.stage = 4;
        // else if (robotData.shooterData.turretTopDeadStop) benchTestData.stage = 5;

        //increments the motors at dead stops
        if (robotData.climbData.armsLowerLimit || robotData.climbData.armsUpperLimit || robotData.climbData.elevatorLowerLimit || robotData.climbData.elevatorUpperLimit || robotData.intakeData.bottomDeadStop || robotData.intakeData.topDeadStop || robotData.shooterData.hoodTopDeadStop || robotData.shooterData.hoodBottomDeadStop || robotData.shooterData.turretTopDeadStop || robotData.shooterData.turretBottomDeadStop){
            benchTestData.stage++;
        }

        //if the final motor in a subsystem is reached, then the subsystem increments
        //additionally, if it reaches the end of shooter, instead of looping back to climb like manual,
        //it makes it an arbitrary number to have a 8 seconds of delay in case the user wants to run bench test again
        if (benchTestData.testStage == BenchTestStage::BenchTestStage_Climb && benchTestData.stage >= 4){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Drivebase;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Shooter && benchTestData.stage >= 5){
            benchTestData.stage = 0;
            benchTestData.testStage = 5; //arbitrary value of delay stage
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && benchTestData.stage >= 6){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Shooter;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && benchTestData.stage >= 4){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Intake;
        } else if (benchTestData.testStage == BenchTestStage::BenchTestStage_Drivebase && benchTestData.stage >= 4){
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Indexer;
        } else if (benchTestData.testStage == 5 && benchTestData.stage >= 3){ //miscellaneous stage for LEDs and the 8 seconds of delay at the end before rerunning bench test
            benchTestData.stage = 0;
            benchTestData.testStage = BenchTestStage::BenchTestStage_Climb;
        }
    }
}