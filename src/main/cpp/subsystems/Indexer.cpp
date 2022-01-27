#include "RobotData.h"

void Indexer::RobotInit()
{
    indexerBeltInit();
    indexerWheelInit();
    indexerBelt.Set(0);
    indexerWheel.Set(0);

}

void Indexer::RobotPeriodic(const RobotData &robotData, IndexerData &indexerData)
{
    updateData(robotData, indexerData);
    count(robotData, indexerData);  // accounts for automatic counting as well as manual decrementing
                                    // pressing slcntrbtn will remove the first element of the deque
    if (robotData.controlData.manualMode)
    {
        manual(robotData, indexerData);
    }
    else
    {
        semiAuto(robotData, indexerData);
        // testControl(robotData);
    }

    // TESTING STUFF
    frc::SmartDashboard::PutNumber("cargo count", indexerData.indexerContents.size());
    frc::SmartDashboard::PutNumber("wrong ball?", robotData.controlData.wrongBall);
    

    // if(getBottomBeam()){
    //     if(robotData.colorSensorData.currentColor == frc::Color::kRed){
    //         frc::SmartDashboard::PutBoolean("currently red?", true);
    //         frc::SmartDashboard::PutBoolean("currently blue?", false);
    //     } else if(robotData.colorSensorData.currentColor == frc::Color::kBlue){
    //         frc::SmartDashboard::PutBoolean("currently red?", false);
    //         frc::SmartDashboard::PutBoolean("currently blue?", true);
    //     } else {
    //         frc::SmartDashboard::PutBoolean("currently red?", false);
    //         frc::SmartDashboard::PutBoolean("currently blue?", false);
    //     }
    // }


}

void Indexer::DisabledInit()
{
    indexerBelt.Set(0);
    indexerWheel.Set(0);
    // indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // indexerWheel.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

}

void Indexer::semiAuto(const RobotData &robotData, IndexerData &indexerData)
{

    saBeltControl(robotData, indexerData);
    saWheelControl(robotData, indexerData);

}

void Indexer::manual(const RobotData &robotData, IndexerData &indexerData)
{

    if(robotData.controlData.mIndexerBackwards){ //run belt and wheel backwards  (B)
        indexerWheel.Set(-indexerWheelSpeed);
        indexerBelt.Set(-indexerBeltSpeed);
    }else if(robotData.controlData.mIndexer){ //run belt and wheel forwards (X)
        indexerBelt.Set(indexerBeltSpeed);
        indexerWheel.Set(indexerWheelSpeed);    
    }else{
        indexerBelt.Set(0);
        indexerWheel.Set(0);   
    }

}

void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{

}

void Indexer::testControl(const RobotData &robotData){
    if(robotData.controlData.mIndexerBackwards){    // run belt and wheel backwards
                                                    // secondary B
        indexerBelt.Set(-indexerBeltSpeed);
        indexerWheel.Set(-indexerWheelSpeed);
    }else if(robotData.controlData.mIndexer){       // run belt and wheel forwards
                                                    // secondary X
        indexerBelt.Set(indexerBeltSpeed);
        indexerWheel.Set(indexerWheelSpeed);    
    }else if (robotData.controllerData.sYBtn) {     // run belt and wheel on exact speed control, separately
                                                    // right is belt, left is wheel                          
        indexerBelt.Set(robotData.controllerData.sRYStick*.3);
        indexerWheel.Set(robotData.controllerData.sLYStick*.3);   
    } else {
        indexerBelt.Set(0);
        indexerWheel.Set(0);   
    }
}

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

        if (robotData.colorSensorData.currentColor == frc::Color::kRed){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.currentColor == frc::Color::kBlue){
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } else {
            indexerData.indexerContents.push_back(Cargo::cargo_Unassigned);
        }

    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){

        if(robotData.colorSensorData.currentColor == frc::Color::kBlue){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.currentColor == frc::Color::kRed){
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } else {
            indexerData.indexerContents.push_back(Cargo::cargo_Unassigned);
        }

    } 
}

void Indexer::assignCargoColor(const RobotData &robotData, IndexerData &indexerData)
{

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){

        if (robotData.colorSensorData.currentColor == frc::Color::kRed){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.currentColor == frc::Color::kBlue){
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } 
    } else {   //blue alliance                                                                                      

        if (robotData.colorSensorData.currentColor == frc::Color::kBlue){ //not sure if I'm checking this correctly, using kRed and all
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Alliance);
        } else if (robotData.colorSensorData.currentColor == frc::Color::kRed){
            indexerData.indexerContents.pop_back();
            indexerData.indexerContents.push_back(Cargo::cargo_Opponent);
        } 

    }
}

// senses if balls leave indexer and removes them from the deque
void Indexer::decrementCount(const RobotData &robotData, IndexerData &indexerData, bool reverse){
    
    if (indexerData.indexerContents.size() > 0){ // making sure we don't pop when there's nothing in there
        if (reverse && getBottomBeamToggled(false)){ // if you're reversing and bb1 toggles off (ball passed completely through)
            indexerData.indexerContents.pop_back();
            // indexerData.ballCount--;
        }else if (!reverse && getTopBeamToggled(false)){
            indexerData.indexerContents.pop_front();
            // indexerData.ballCount--;
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
    if(topDebounceCount > 0){
        topDebounceCount--;
    }
    if(robotData.controlData.saEjectBalls || robotData.controlData.mIndexerBackwards){ // if BACKWARDS
        decrementCount(robotData, indexerData, true); //true means you're reversing
    } else {
        decrementCount(robotData, indexerData, false);
        incrementCount(robotData, indexerData);
    }

    /**
     * if FORWARDS
     * run increment
     * run decrement
     * 
     * if BACKWARDS
     * run only decrement     * 
     **/

    if(robotData.controlData.manualMode && robotData.controlData.mDecrementCargo){
        mDecrement(robotData, indexerData);
    }

}

void Indexer::saBeltControl(const RobotData &robotData, IndexerData &indexerData){

    if(robotData.controlData.saEjectBalls || robotData.controlData.mIndexerBackwards){ // if indexer is REVERSING (saEject or manual indexer backwards)
        indexerBelt.Set(-indexerBeltSpeed);
    } else if ((!getTopBeam() || robotData.shooterData.readyShoot) /**&& !robotData.intakeData.intakeIdle**/){ // if you're shooting or BB3 is not tripped
        indexerBelt.Set(indexerBeltSpeed);
    }  else {
        indexerBelt.Set(0);
    }
    

}

void Indexer::saWheelControl(const RobotData &robotData, IndexerData &indexerData){
    if(robotData.controlData.saEjectBalls || robotData.controlData.mIndexerBackwards){ // if indexer is REVERSING (saEject or manual indexer backwards)
        indexerWheel.Set(-indexerWheelSpeed);
    } else if ((robotData.shooterData.readyShoot || !(getTopBeam() && getMidBeam())) && !robotData.intakeData.intakeIdle){ // if indexer is not yet full or if indexer DOES have 2 balls but the 2nd sensor has not yet been tripped
        indexerWheel.Set(indexerWheelSpeed);
    } else {
        indexerWheel.Set(0);
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

// bool Indexer::getMidBeamToggled(bool broken){
//     if (getMidBeam() == broken && prevMidBeam != broken){
//         // set prev to current and return true
//         prevMidBeam = getMidBeam();
//         return true;
//     } else {
//         prevMidBeam = getMidBeam();
//         return false;
//     }
// }

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

void Indexer::indexerBeltInit(){
    indexerBelt.RestoreFactoryDefaults();
    indexerBelt.SetInverted(true);
    indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerBelt.SetSmartCurrentLimit(45);
}

void Indexer::indexerWheelInit(){
    indexerWheel.RestoreFactoryDefaults();
    indexerWheel.SetInverted(true);
    indexerWheel.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerWheel.SetSmartCurrentLimit(45);
}
