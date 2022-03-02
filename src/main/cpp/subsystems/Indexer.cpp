#include "RobotData.h"

void Indexer::RobotInit()
{
    indexerBeltInit();
    indexerWheelInit();
    indexerBelt.Set(0);
    indexerWheel.Set(0);

}

// init in auton with an alliance ball preloaded
void Indexer::AutonomousInit(IndexerData &indexerData) {
    indexerData.indexerContents.clear();
    indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
}

void Indexer::RobotPeriodic(const RobotData &robotData, IndexerData &indexerData)
{
    updateTopBeamToggled(indexerData);
    updateData(robotData, indexerData);
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
}

void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    if (indexerData.indexerContents.size() == 0 && lastTickBallCount > 0) {
        indexerData.eBallCountZero = true;
    } else {
        indexerData.eBallCountZero = false;
    }
    lastTickBallCount = indexerData.indexerContents.size();

    // debuggingStuff(robotData, indexerData);
}

void Indexer::manual(const RobotData &robotData, IndexerData &indexerData)
{

    if(robotData.controlData.mIndexerDown){ //run belt and wheel backwards  (B)
        indexerWheel.Set(-indexerWheelSpeed);
        indexerBelt.Set(-indexerBeltSpeed);
    }else if(robotData.controlData.mIndexerUp){ //run belt and wheel forwards (X)
        indexerBelt.Set(indexerBeltSpeed);
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

// void Indexer::debuggingStuff(const RobotData &robotData, IndexerData &indexerData){
//     // TESTING STUFF
//     frc::SmartDashboard::PutNumber("cargo count", indexerData.indexerContents.size());
//     frc::SmartDashboard::PutNumber("wrong ball?", robotData.controlData.wrongBall);

//     if (indexerData.indexerContents.size() == 0){

//         frc::SmartDashboard::PutString("top", "empty");
//         frc::SmartDashboard::PutString("bottom", "empty");

//     } else if (indexerData.indexerContents.size() == 1){

//         if(indexerData.indexerContents.front() == Cargo::cargo_Alliance){
//             frc::SmartDashboard::PutString("top", "alliance");
//             frc::SmartDashboard::PutString("bottom", "empty");
//         } else if (indexerData.indexerContents.front() == Cargo::cargo_Opponent){
//             frc::SmartDashboard::PutString("top", "opponent");
//             frc::SmartDashboard::PutString("bottom", "empty");
//         } else {
//             frc::SmartDashboard::PutString("top", "unassigned");
//             frc::SmartDashboard::PutString("bottom", "empty");
//         }

//     } else if (indexerData.indexerContents.size() == 2){

//         if(indexerData.indexerContents.front() == Cargo::cargo_Alliance){

//             if(indexerData.indexerContents.back() == Cargo::cargo_Alliance){
//                 frc::SmartDashboard::PutString("top", "alliance");
//                 frc::SmartDashboard::PutString("bottom", "alliance");
//             } else if(indexerData.indexerContents.back() == Cargo::cargo_Opponent){
//                 frc::SmartDashboard::PutString("top", "alliance");
//                 frc::SmartDashboard::PutString("bottom", "opponent");
//             } else {
//                 frc::SmartDashboard::PutString("top", "alliance");
//                 frc::SmartDashboard::PutString("bottom", "unassigned");
//             }

//         } else if (indexerData.indexerContents.front() == Cargo::cargo_Opponent){

//             if(indexerData.indexerContents.back() == Cargo::cargo_Alliance){
//                 frc::SmartDashboard::PutString("top", "opponent");
//                 frc::SmartDashboard::PutString("bottom", "alliance");
//             } else if(indexerData.indexerContents.back() == Cargo::cargo_Opponent){
//                 frc::SmartDashboard::PutString("top", "opponent");
//                 frc::SmartDashboard::PutString("bottom", "opponent");
//             } else {
//                 frc::SmartDashboard::PutString("top", "opponent");
//                 frc::SmartDashboard::PutString("bottom", "unassigned");
//             }

//         } else {
//             if(indexerData.indexerContents.back() == Cargo::cargo_Alliance){
//                 frc::SmartDashboard::PutString("top", "unassigned");
//                 frc::SmartDashboard::PutString("bottom", "alliance");
//             } else if(indexerData.indexerContents.back() == Cargo::cargo_Opponent){
//                 frc::SmartDashboard::PutString("top", "unassigned");
//                 frc::SmartDashboard::PutString("bottom", "opponent");
//             } else {
//                 frc::SmartDashboard::PutString("top", "unassigned");
//                 frc::SmartDashboard::PutString("bottom", "unassigned");
//             }
            
//         }
//     } else {
//         frc::SmartDashboard::PutString("top", "overload");
//         frc::SmartDashboard::PutString("bottom", "overload");
//     }
// }



// sense if balls enter indexer and assigns color to additional balls
void Indexer::incrementCount(const RobotData &robotData, IndexerData &indexerData)
{

    if (indexerData.indexerContents.size() == 0) { // if indexer is empty  
        if(getBottomBeamToggled(true)){  // if bb1 was just toggled add new ball
            newCargo(robotData, indexerData);
        }
    } else { // indexer is not empty
        if (getBottomBeamToggled(true)){ // bottom sensor was triggered
            newCargo(robotData, indexerData);
        } else if (indexerData.indexerContents.back() == Cargo::cargo_Unassigned){ // if last piece is unassigned
            // if the last cargo is an unassigned cargo and the bottom beam was not just triggered
            assignCargoColor(robotData, indexerData);
            // still not sure about this logic
        }

    }


        
}

void Indexer::newCargo(const RobotData &robotData, IndexerData &indexerData){

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){

        if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } else {
            indexerData.indexerContents.push_back(Cargo::cargo_Unassigned);
        }

    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){

        if(robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } else {
            indexerData.indexerContents.push_back(Cargo::cargo_Unassigned);
        }

    } 
}

void Indexer::assignCargoColor(const RobotData &robotData, IndexerData &indexerData)
{

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){

        if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } 
    } else {   //blue alliance                                                                                      

        if (robotData.colorSensorData.colorValue == CargoColor::cargo_Blue){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.colorValue == CargoColor::cargo_Red){
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } 

    }
}

// senses if balls leave indexer and removes them from the deque
void Indexer::decrementCount(const RobotData &robotData, IndexerData &indexerData, bool reverse){
    
    if (reverse && getBottomBeamToggled(false)){ // if you're reversing and bb1 toggles off (ball passed completely through)
        
        if (indexerData.indexerContents.size() > 0){
            indexerData.indexerContents.pop_back();
        }

    }else if (!reverse && indexerData.topBeamToggledOff && robotData.controlData.saFinalShoot){
        if (indexerData.indexerContents.size() > 0){
            indexerData.indexerContents.pop_front();
        }
    }
}

void Indexer::mDecrement(const RobotData &robotData, IndexerData &indexerData)
{
    if(indexerData.indexerContents.size() > 0){
        indexerData.indexerContents.pop_front();
    }
}

// runs both count functions in appropriate cases
void Indexer::count(const RobotData &robotData, IndexerData &indexerData){

    if(bottomDebounceCount > 0){
        bottomDebounceCount--;
    }

    if(robotData.controlData.saEjectBalls || robotData.controlData.mIndexerDown){ // if BACKWARDS
        decrementCount(robotData, indexerData, true); //true means you're reversing
    } else { // you are going forwards. this runs every time as you go forward
        decrementCount(robotData, indexerData, false); // going forward
        incrementCount(robotData, indexerData);
    }

    if(robotData.controlData.mode == mode_teleop_manual && robotData.controlData.mDecrementCargo){
        mDecrement(robotData, indexerData);
    }

}

void Indexer::saBeltControl(const RobotData &robotData, IndexerData &indexerData){

    if(robotData.controlData.saEjectBalls){ // if indexer is REVERSING (saEject or manual indexer backwards)
        indexerBelt.Set(-indexerBeltSpeed);
    } else if ((!pauseBelt(robotData, indexerData) && robotData.shooterData.readyShoot && robotData.controlData.saFinalShoot)|| (!getTopBeam() && !robotData.intakeData.intakeIdle)){ // if you're shooting or (BB3 is not  and the intake isn't idle)
        indexerBelt.Set(indexerBeltSpeed);
    }  else {
        indexerBelt.Set(0);
    }

}

void Indexer::saWheelControl(const RobotData &robotData, IndexerData &indexerData){

    if(getTopBeam()){
        runWheel = false; 
        // set this to true in decrementcount logic
        // checks if we should run the wheel so that we don't jam the balls up together in the indexer
    } else {
        runWheel = true;
    }

    if(robotData.controlData.saEjectBalls){ // if indexer is REVERSING (saEject or manual indexer backwards)
        indexerWheel.Set(-indexerWheelSpeed);
    } else if (((robotData.shooterData.readyShoot && robotData.controlData.saFinalShoot) && runWheel) || (!(getTopBeam() && getMidBeam()) && !robotData.intakeData.intakeIdle)){ // if indexer is not yet full or if indexer DOES have 2 balls but the 2nd sensor has not yet been tripped
        indexerWheel.Set(indexerWheelSpeed);
    } else {
        indexerWheel.Set(0);
    }
    
}

bool Indexer::pauseBelt(const RobotData &robotData, IndexerData &indexerData){

    if(indexerData.topBeamToggledOn){   // the top sensor was just toggled on
                                        // concern: if it was toggled on due to belt slippage?
        pauseBeltCount = 5;             // set pause belt count for .1s
        return true;
    } else if (pauseBeltCount > 0){
        pauseBeltCount--;
        return true;
    } else {                                        // pause belt false, belt paused for sufficient time
        return false;
    }
}

// basic getter, init functions below

// usually these sensors return false for when they're tripped, these functions return opposite to match intuitive logic
bool Indexer::getBottomBeam(){
    return !bottomBeamBreak.Get();
}

bool Indexer::getMidBeam(){
    return !midBeamBreak.Get();
}

bool Indexer::getTopBeam(){
    return !topBeamBreak.Get();
}

bool Indexer::getBottomBeamToggled(bool broken){

    //debounce
    if(bottomDebounceCount > 0){
        prevBottomBeam = getBottomBeam();
        return false;
    }

    if (getBottomBeam() == broken && prevBottomBeam != broken){
        // set prev to current and return true
        prevBottomBeam = getBottomBeam();
        // debounce
        bottomDebounceCount = 2;
        return true;
    } else {
        prevBottomBeam = getBottomBeam();
        return false;
    }
}



/**
 * @param broken -- true is if it toggled to sensing a ball
 *               -- false if it is toggled to NOT sensing a ball
 * @return whether it was toggled to the desired state
 * 
 * concern: when robot first starts up?
 *          when it's run more than one time in same 20 ms???
 * 
 **/
bool Indexer::getTopBeamToggled(bool broken){

    //debounce
    if(topDebounceCount > 0){
        prevTopBeam = getTopBeam();
        return false;
    }
    // if top sensor is currently in desired broken state and it previously wasn't
    if (getTopBeam() == broken && prevTopBeam != broken){
        // set prev to current and return true
        prevTopBeam = getTopBeam();
        //debounce
        topDebounceCount = 2;
        return true;
    } else {
        prevTopBeam = getTopBeam();
        return false;
    }
}

void Indexer::updateTopBeamToggled(IndexerData &indexerData){

    
    // debouncing. if it's not done debouncing just return
    if(topDebounceCount > 0){
        topDebounceCount--;
        prevTopBeam = getTopBeam();
        indexerData.topBeamToggledOff = false;
        indexerData.topBeamToggledOn = false;
        return;
    }

    if(getTopBeam()) {      // top sensor senses a ball
        if(!prevTopBeam){   // previously did not sense a ball, it was toggled
            prevTopBeam = getTopBeam();
            topDebounceCount = 2;
            indexerData.topBeamToggledOff = false;
            indexerData.topBeamToggledOn = true;
        } else {
            prevTopBeam = getTopBeam();
            indexerData.topBeamToggledOff = false;
            indexerData.topBeamToggledOn = false;
        }
    } else {                // top sensor does not sense a ball
        if(prevTopBeam){    // previously sensed a ball, it was toggled
            prevTopBeam = getTopBeam();
            topDebounceCount = 2;
            indexerData.topBeamToggledOff = true;
            indexerData.topBeamToggledOn = false;
        } else {
            prevTopBeam = getTopBeam();
            indexerData.topBeamToggledOff = false;
            indexerData.topBeamToggledOn = false;
        }
    }




}

void Indexer::indexerBeltInit(){
    indexerBelt.RestoreFactoryDefaults();
    indexerBelt.SetInverted(true);
    indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerBelt.SetSmartCurrentLimit(25);
}

void Indexer::indexerWheelInit(){
    indexerWheel.RestoreFactoryDefaults();
    indexerWheel.SetInverted(false);
    indexerWheel.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerWheel.SetSmartCurrentLimit(25);
}

//BENCH TEST CODE
void Indexer::TestPeriodic(const RobotData &robotData, IndexerData &indexerData){
    frc::SmartDashboard::PutNumber("Color sensor number", robotData.colorSensorData.colorValue);
    frc::SmartDashboard::PutNumber("First indexer encoder", indexerBeltEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Second indexer encoder", indexerWheelEncoder.GetPosition());

    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Indexer && robotData.controlData.startBenchTest){ //checks if we're testing indexer
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
        }
    } else {
        indexerWheel.Set(0);
        indexerBelt.Set(0);
    }
}