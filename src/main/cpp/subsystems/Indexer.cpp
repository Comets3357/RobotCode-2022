#include "RobotData.h"

void Indexer::RobotInit()
{
    indexerBeltInit();
    indexerWheelInit();
    indexerBelt.Set(0);
    indexerWheel.Set(0);

}

/**
 * init in auton with an alliance ball preloaded
 **/
void Indexer::AutonomousInit(IndexerData &indexerData) {
    indexerData.indexerContents.clear();
    indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
}

void Indexer::RobotPeriodic(const RobotData &robotData, IndexerData &indexerData)
{
    updateData(robotData, indexerData);
    updateSensors();
    count(robotData, indexerData);  // accounts for automatic counting as well as manual decrementing
                                    // there are manual functions for incremeneting and decrementing cargo as well, see controldata.cpp

    if(robotData.controlData.mode == mode_climb_manual || robotData.controlData.mode == mode_climb_sa){
        indexerBelt.Set(0);
        indexerWheel.Set(0);

    }else{
        if (robotData.controlData.mode == mode_teleop_manual)
        {
            manual(robotData, indexerData);
        }
        else if (robotData.controlData.mode == mode_teleop_sa)
        {
            semiAuto(robotData, indexerData);
        }
    }
    

}

void Indexer::DisabledInit()
{
    indexerBelt.Set(0);
    indexerWheel.Set(0);

}

void Indexer::DisabledPeriodic(const RobotData &robotData, IndexerData &indexerData){
    updateData(robotData, indexerData);
    updateSensors();
}

void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    if (indexerData.indexerContents.size() == 0 && lastTickBallCount > 0) {
        indexerData.eBallCountZero = true;
    } else {
        indexerData.eBallCountZero = false;
    }
    lastTickBallCount = indexerData.indexerContents.size();

    indexerData.topBeamBreak = getTopBeam();
    indexerData.midBeamBreak = getMidBeam();
}

void Indexer::manual(const RobotData &robotData, IndexerData &indexerData)
{

    if(robotData.controlData.mIndexerDown){ //run belt and wheel backwards  (B)
        indexerWheel.Set(-indexerWheelSpeed);
        indexerBelt.Set(-indexerShootingBeltSpeed);
    }else if(robotData.controlData.mIndexerUp){ //run belt and wheel forwards (X)
        indexerBelt.Set(indexerShootingBeltSpeed);
        indexerWheel.Set(indexerWheelSpeed);    
    }else{
        indexerBelt.Set(0);
        indexerWheel.Set(0);   
    }

}

void Indexer::semiAuto(const RobotData &robotData, IndexerData &indexerData)
{

    saBeltControl(robotData, indexerData);
    saWheelControl(robotData, indexerData);
}

/**
 * called when indexer is not going backwards
 * sense if balls enter indexer and assigns color to a ball that doesn't have one yet
 **/
void Indexer::incrementCount(const RobotData &robotData, IndexerData &indexerData)
{

    if (indexerData.indexerContents.size() == 0) { // if indexer is empty  
        if(getBottomBeamToggledOn()){  // if bb1 was just toggled add new ball
            newCargo(robotData, indexerData);
        }
    } else { // indexer is not empty
        if (getBottomBeamToggledOn()){ // bottom sensor was triggered
            newCargo(robotData, indexerData);
        } else if (indexerData.indexerContents.back() == Cargo::cargo_Unassigned){ // if last piece is unassigned
            // if the last cargo is an unassigned cargo and the bottom beam was not just triggered
            assignCargoColor(robotData, indexerData);
            // still not sure about this logic
        }
    }
        
}

/**
 * called when the bottom sensor was just tripped and adds a cargo to the indexerContents deque
 * checks which alliance you're on against the cargo color and adds as opponent, alliance, or unassigned rather than by color
 **/
void Indexer::newCargo(const RobotData &robotData, IndexerData &indexerData){

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){            // if you are red alliance

        if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){                 // and if the sensor senses red
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);                   // add red cargo
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){         // blue
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);                   // add blue
        } else {                                                                            // doesn't sense red OR blue (if this happens check that the LED is on)
            indexerData.indexerContents.push_back(Cargo::cargo_Unassigned);                 // add "unassigned"
        }

    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){   // same logic for blue as for red

        if(robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){ 
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } else {
            indexerData.indexerContents.push_back(Cargo::cargo_Unassigned);
        }

    } 
}

/**
 * called when the last element of the deque is an unassigned cargo and the bottom sensor was NOT just tripped
 * is supposed to assign a color to that unassigned cargo if for some reason the color sensor did not sense a color the first time around
 **/
void Indexer::assignCargoColor(const RobotData &robotData, IndexerData &indexerData)
{

    // same logic as above newCargo function, except you remove the last element before adding a new cargo
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){        // red alliance

        if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){             // red ball
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);               // replace with alliance ball
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){     // blue ball
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);               // replace with opponent ball
        } 
    } else {                                                                            // blue alliance                                                                                      

        if (robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){            // blue ball
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);               // replace with alliance ball
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){      // red ball
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);               // replace with opponent ball
        } 

    }
}

/**
 * senses when balls leave the indexer either from top or bottom and removes them from the indexerContents deque
 **/
void Indexer::decrementCount(const RobotData &robotData, IndexerData &indexerData, bool reverse){

    
    if (reverse && getBottomBeamToggledOff()){                                              // if you're reversing and bottom sensor toggles to not being tripped (ball passed completely through)
        if (indexerData.indexerContents.size() > 0){    // checks to make sure there's actually stuff in the indexer to make sure the code doesn't crash
            indexerData.indexerContents.pop_back();     // remove "bottom" element of deque
        }

    }
    
    if (!reverse && getTopBeamToggledOff() && (robotData.controlData.saFinalShoot)){    // if you're going FORWARD, specifically in saFinalShoot, and the top sensor toggles to not being tripped (ball passed completely through and shot)
        decrementDelay = 5;
    }else if (decrementDelay > 0 && decrementDelay <= 5){       // the top sensor hasn't just been toggled off but was recently toggled off
        decrementDelay--;                                       // count down the decrementDelay counter
    } else if(decrementDelay == 0){                                                    // the count down is done 
        if (indexerData.indexerContents.size() > 0){            // checks to make sure there's actually stuff in the indexer to make sure the code doesn't crash
            indexerData.indexerContents.pop_front();            // removes "top" element of deque
        }
        decrementDelay = 6;
    }
}

/**
 * called based on secondary driver button, removes the "top" element of deque manually
 **/
void Indexer::mDecrement(const RobotData &robotData, IndexerData &indexerData)
{
    if(indexerData.indexerContents.size() > 0){
        indexerData.indexerContents.pop_front();
    }
}

/**
 * runs all the counting functions (decrement, increment) based on current secondary controls
 **/
void Indexer::count(const RobotData &robotData, IndexerData &indexerData){

    if(robotData.controlData.saEjectBalls || robotData.controlData.mIndexerDown){   // if you're going backwards
        decrementCount(robotData, indexerData, true);                               // true means you're reversing
    } else {                                                                        // you are going forwards. this runs every time as you go forward
        decrementCount(robotData, indexerData, false);                              // false means going forward
                                                              // decrement runs while the indexer goes forward and backward because balls can exit from top or bottom
        incrementCount(robotData, indexerData);                                     // only runs increment when you go forward because we're not ever intaking while reversing
    }

    if(robotData.controlData.mode == mode_teleop_manual && robotData.controlData.mDecrementCargo){
        mDecrement(robotData, indexerData);                                         // manually decrement count if something was sensed wrong
    }

}


/**
 * runs the belt in semi auto mode based on the three sensors and secondary controls
 **/
void Indexer::saBeltControl(const RobotData &robotData, IndexerData &indexerData){
    if(robotData.controlData.saEjectBalls){             // if indexer is REVERSING (saEject curently is the only case where it runs backwards)
        indexerBelt.Set(-indexerShootingBeltSpeed);     // run the belt backwards fast
    } else if ((/* !pauseBelt(robotData, indexerData) && */ (robotData.shooterData.readyShoot) && robotData.controlData.saFinalShoot) || (!getTopBeam() && (!robotData.intakeData.intakeIdle || robotData.controlData.saFinalShoot))){ 
        // there are two main cases when you run the indexer forward: when you shoot, and when you're intaking
        // when shooting, you check that you're done pausing (see pauseBelt) to make sure every ball pauses before going into the shooter, 
        // anyways, we have to get the signal that the shooter is ready to eject AND that the top ball in the indexer is the opponent color to run it in that case
        // and you also check readyShoot to make sure the flywheel is up to speed, along with saFinalShoot to make sure secondary is commanding it to shoot
        // you also check that the intake has been given a command in the past second (see intakeIdle function in intake class) to make sure the indexer isn't running for no reason
        // when intaking, you want to run the ball up until it triggers the top sensor
        // even though this is the intaking mode it doesn't actually look at "if saIntake" directly or anything because no matter when we're intaking or just running a second cargo through,
        // we want to run it until it hits the top sensor, then stop it, no matter if the robot is shooting, intaking, or whatever
        // actually, new code
        // currently not stopping it if in ejecting mode because I don't want to think about that right now
        // anyways, we have to get the signal that the shooter is ready to eject AND that the top ball in the indexer is the opponent color to run it in that case
        if(robotData.controlData.saFinalShoot && robotData.shooterData.readyShoot){
            indexerBelt.Set(indexerShootingBeltSpeed);  // robot shoots at a higher speed than it intakes
        } else {
            indexerBelt.Set(indexerIntakingBeltSpeed);
        }
    }  else {                                           // otherwise the indexer does not run
        indexerBelt.Set(0);
    }

}

/**
 * runs the wheel in semi auto mode based on the three sensors and secondary controls
 **/
void Indexer::saWheelControl(const RobotData &robotData, IndexerData &indexerData){

    if(robotData.controlData.saEjectBalls){     // if indexer is REVERSING (saEject)
        indexerWheel.Set(-indexerWheelSpeed);   // run the wheel backwards
    } else if (((robotData.shooterData.readyShoot || robotData.controlData.saFinalShoot) && !getTopBeam()) || (!(getTopBeam() && getMidBeam()) && !robotData.intakeData.intakeIdle)){ 
        // there are two main cases when you run the wheel forward: shooting or intaking
        // when we shoot (readyShoot and saFinalShoot true), we only want to run the wheel as long as the top sensor is not being tripped
        // this is because it reduces the chances of the two cargo getting jammed together
        // when the top ball stops triggering the top sensor, that means it essentially left the indexer, and it's safe to run the next ball up
        // when intaking, the first ball goes in, goes to trip the top sensor before it stops
        // when we intake the next ball, we want to stop it when it trips the middle sensor. this is the only function of the middle sensor
        // the above two lines explain why we want to run the wheel (while the intake is not idle) as long as one of the top two sensors have not been tripped
        // and, of course, if the intake is not idle
        indexerWheel.Set(indexerWheelSpeed);
    } else {                                    // otherwise the wheel does not run
        indexerWheel.Set(0);
    }
    
}


// basic getter, init functions below
// usually these sensors return false for when they're tripped, these functions return opposite to match intuitive logic

/**
 * @return true if sensor is being tripped, false if sensor is not being tripped
 **/
bool Indexer::getBottomBeam(){
    return !bottomBeamBreak.Get();
}

/**
 * @return true if sensor is being tripped, false if sensor is not being tripped
 **/
bool Indexer::getMidBeam(){
    return !midBeamBreak.Get();
}

/**
 * @return true if sensor is being tripped, false if sensor is not being tripped
 **/
bool Indexer::getTopBeam(){
    return !topBeamBreak.Get();
}


/**
 * updates sensor data to store both current and previous states of sensors
 * used to make sure that toggles function properly even when toggle states have to be checked multiple times in one loop of periodic
 * also handles debounce counts in one place
 **/
void Indexer::updateSensors(){
    prevBottomBeam = currentBottomBeam;
    currentBottomBeam = getBottomBeam();
    prevTopBeam = currentTopBeam;
    currentTopBeam = getTopBeam();

    // not sure if debounce is actually being used
    // but basically if a sensor was just said to be toggled it won't be said to be toggled again within the next short period of time
    // and we count down the debounce here
    if(bottomDebounceCount > 0){
        bottomDebounceCount--;
    }
    if(topDebounceCount > 0){
        topDebounceCount--;
    }
}


/**
 * @return if the top sensor was just toggled to being tripped
 **/
bool Indexer::getTopBeamToggledOn(){
    //debounce
    if(topDebounceCount > 0){
        return false;
    }
    // if top sensor is currently being tripped and it previously wasn't
    if (currentTopBeam && !prevTopBeam){
        topDebounceCount = 2;
        return true;
    } else {
        return false;
    }
}

/**
 * @return if the top sensor was just toggled to not being tripped
 **/
bool Indexer::getTopBeamToggledOff(){
    // debounce
    if(topDebounceCount > 0){
        return false;
    }
    // if top sensor is currently not being tripped and it previously wasn't
    if (!currentTopBeam && prevTopBeam){
        topDebounceCount = 2;
        return true;        
    } else {
        return false;
    }
}

/**
 * @return if the bottom sensor was just toggled to being tripped
 **/
bool Indexer::getBottomBeamToggledOn(){
     //debounce does this count multiple times in a loop?
    if(bottomDebounceCount > 0){
        return false;
    }
    // if top sensor is currently being tripped and it previously wasn't
    if (currentBottomBeam && !prevBottomBeam){
        bottomDebounceCount = 10;
        return true;
    } else {
        return false;
    }

}

/**
 * @return if the bottom sensor was just toggled to not being tripped
 **/
bool Indexer::getBottomBeamToggledOff(){
     //debounce does this count multiple times in a loop?
    if(bottomDebounceCount > 0){
        return false;
    }
    // if top sensor is currently being tripped and it previously wasn't
    if (!currentBottomBeam && prevBottomBeam){
        bottomDebounceCount = 2;
        return true;
    } else {
        return false;
    }
}

/**
 * initializes the belt and sets current limit to 25
 **/
void Indexer::indexerBeltInit(){
    indexerBelt.RestoreFactoryDefaults();
    indexerBelt.SetInverted(true);
    indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerBelt.SetSmartCurrentLimit(25);
}

/**
 * initializes the wheel and sets current limit to 25
 **/
void Indexer::indexerWheelInit(){
    indexerWheel.RestoreFactoryDefaults();
    indexerWheel.SetInverted(false);
    indexerWheel.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerWheel.SetSmartCurrentLimit(25);
}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * BENCH TEST CODE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 **/

void Indexer::TestPeriodic(const RobotData &robotData, IndexerData &indexerData){
    //diagnosing issues with smart dashboard
    frc::SmartDashboard::PutBoolean("Indexer color sensor sensed blue?", robotData.arduinoData.ColorData == 11);
    frc::SmartDashboard::PutBoolean("Indexer color sensor sensed red?", robotData.arduinoData.ColorData == 12);
    frc::SmartDashboard::PutBoolean("Indexer color sensor sensed nothing?", robotData.arduinoData.ColorData == 10);
    frc::SmartDashboard::PutBoolean("Indexer color sensor raw data", robotData.arduinoData.ColorData);
    frc::SmartDashboard::PutNumber("Indexer first encoder", indexerBeltEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Indexer second encoder", indexerWheelEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("Indexer top beam break sensor", getTopBeam());
    frc::SmartDashboard::PutBoolean("Indexer middle beam break sensor", getMidBeam());
    frc::SmartDashboard::PutBoolean("Indexer bottom beam break sensor", getBottomBeam());

    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && (robotData.controlData.manualBenchTest || robotData.controlData.autoBenchTest)){ //checks if we're testing indexer
        if (robotData.benchTestData.stage == 0){
            //run wheel forwards
            indexerWheel.Set(robotData.benchTestData.currentSpeed); //sets the wheel speed
            indexerBelt.Set(0); //sets the belt speed
        } else if (robotData.benchTestData.stage == 1){
            //run wheel backwards
            indexerWheel.Set(-robotData.benchTestData.currentSpeed);
            indexerBelt.Set(0);
        } else if (robotData.benchTestData.stage == 2){
            //run belt forwards
            indexerBelt.Set(robotData.benchTestData.currentSpeed);
            indexerWheel.Set(0);
        } else if (robotData.benchTestData.stage == 3){
            //run belt backwards
            indexerBelt.Set(-robotData.benchTestData.currentSpeed);
            indexerWheel.Set(0);
        } else {
            indexerBelt.Set(0); //if the indexer stage isn't within 0 to 3, then the speeds get set to 0
            indexerWheel.Set(0);
        }
    } else {
        indexerBelt.Set(0); //if not testing indexer, then speeds get set to 0
        indexerWheel.Set(0);
    }
}