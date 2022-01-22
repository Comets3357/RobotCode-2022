// #include "subsystems/Indexer.h"
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
    if (robotData.controlData.manualMode)
    {
        manual(robotData, indexerData);
    }
    else
    {
        semiAuto(robotData, indexerData);
    }
}

void Indexer::semiAuto(const RobotData &robotData, IndexerData &indexerData){

    if(robotData.controlData.saEjectBalls){ //run belt and wheel backwards, ejecting through intake
        indexerBelt.Set(-IndexerBeltSpeed);
        indexerWheel.Set(-IndexerWheelSpeed);
    }else if(robotData.shooterData.readyShoot){ // you are shooting
        shootSequence(indexerData);
    }else if(robotData.controlData.saIntake){ // you are intaking
        intakeSequence(indexerData);
    }else{
        indexerBelt.Set(0);
        indexerWheel.Set(0);
    }

}

void Indexer::manual(const RobotData &robotData, IndexerData &indexerData){
    if(robotData.controlData.mIndexerBackwards){ //run belt and wheel backwards
        indexerBelt.Set(-IndexerBeltSpeed);
        indexerWheel.Set(-IndexerWheelSpeed);
    }else if(robotData.controlData.mIndexer){ //run belt and wheel forwards
        indexerBelt.Set(IndexerBeltSpeed);
        indexerWheel.Set(IndexerWheelSpeed);    
    }else{
        indexerBelt.Set(0);
        indexerWheel.Set(0);   
    }
}

void Indexer::DisabledInit()
{
    indexerBelt.Set(0);
    indexerWheel.Set(0);
}

// // updates encoder and gyro values
void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    if(robotData.controlData.mIndexerBackwards){    // run belt and wheel backwards
                                                    // secondary B
        indexerBelt.Set(-IndexerBeltSpeed);
        indexerWheel.Set(-IndexerWheelSpeed);
    }else if(robotData.controlData.mIndexer){       // run belt and wheel forwards
                                                    // secondary X
        indexerBelt.Set(IndexerBeltSpeed);
        indexerWheel.Set(IndexerWheelSpeed);    
    }else if (robotData.controllerData.sYBtn) {     // run belt and wheel on exact speed control, separately
                                                    // right is belt, left is wheel                          
        indexerBelt.Set(robotData.controllerData.sRYStick);
        indexerWheel.Set(robotData.controllerData.sLYStick);   

    }  else {
        indexerBelt.Set(0);
        indexerWheel.Set(0);   
    }
}

void Indexer::testControl(const RobotData &robotData){

}

void Indexer::intakeSequence(IndexerData &indexerData){
    if(indexerData.ballCount == 0){

        
        /** 
         * 
         * i still NEED TO ADD COLOR LOGIC but
         * this is the ideal logic
         * when the first sensor senses a ball, then it checks the color sensor and adds a ball of correct/wrong alliance to indexer
         * at this point the driver should be able to reject the ball or just let it go through
         * when the ball passes the second sensor (sensor doesn't sense anything but previously did)
         * then stop the belt
         * ideally, if I made some sort of toggle function that would make this easier and less messy
         * when you stop the belt, you reset all the flags as it is right now
         * but with a toggle function you wouldn't need flags
         * actually, maybe you probably would. we want to know whether or not a sensor has been tripped
         * within a circle cycle, not just if it toggled back to not seeing anything
         * current concern: if intake gets balls fast enough this code might not work
         * but I haven't truly thought that far yet. 
         * 
         **/

        if(getBottomBeam()){ // check if first sensor sensed cargo
            firstSensorTripped = true;
            // need color logic here still
        }

        if (getMidBeam()){
            secondSensorTripped = true;
        }

        if(firstSensorTripped) { // if cargo has been senses
            if(!getMidBeam() && !secondSensorTripped) { // if mid sensor has not been tripped AND was not previously tripped
                indexerBelt.Set(saIndexerBeltIntakeSpeed);
                indexerWheel.Set(saIndexerWheelIntakeSpeed);
            } else if (!getMidBeam() && secondSensorTripped){ // mid sensor is not tripped but it was tripped before
                indexerBelt.Set(0);
                indexerWheel.Set(saIndexerWheelIntakeSpeed); // wheel keeps running
                firstSensorTripped = false; // reset these flags
                secondSensorTripped = false;
                indexerData.ballCount = 1;
            }
        } else { // run both wheel and belt until first sensor senses something
            indexerBelt.Set(saIndexerBeltIntakeSpeed);
            indexerWheel.Set(saIndexerWheelIntakeSpeed);
        }
        
    } else if (indexerData.ballCount == 1){

        /**
         * this is logic for when there is already one ball in the indexer
         * it would be resting above the second sensor, free of the wheel and any sensors
         * so the wheel can easily keep running
         * once the bottom sensor senses a ball logic for adding it to indexer count runs
         * the belt starts running when the second sensor is tripped by the second ball
         * and both the wheel and belt stop as soon as the third sensor is tripped
         * also, the function that senses the ball at the bottom and decides whether or not to reject it
         * will be separate. need to figure out how to integrate that. 
         * there are also definitely extraneous else ifs that i wrote in to better understand the logic. 
         * 
         * 
         **/
        if(getBottomBeam()){ // if bottom sensor sees ball
            firstSensorTripped = true; // check if first sensor sensed cargo
            // insert logic for adding to indexer? color stuff, etc. 
        } 
        
        if (secondSensorTripped){ // if the second sensor has been tripped by the second ball
            if(getTopBeam()){ // top beam is BROKEN, sequence is complete
                indexerBelt.Set(0);
                indexerWheel.Set(0);
                indexerData.ballCount = 2;
            } else {    // mid has been broken but the top beam has not been broken yet
                        // running both belt and wheel
                indexerBelt.Set(saIndexerBeltIntakeSpeed);
                indexerWheel.Set(saIndexerWheelIntakeSpeed);
            }

        } else if(!getMidBeam()){ // until mid beam sees something (second ball) belt does not run
            indexerBelt.Set(0);
            indexerWheel.Set(saIndexerWheelIntakeSpeed);
        } else if (getMidBeam()){ // when mid beam senses something then run both belt and wheel
            indexerBelt.Set(saIndexerBeltIntakeSpeed);
            indexerWheel.Set(saIndexerWheelIntakeSpeed);
            secondSensorTripped = true;
        }
            
        
    } else if (indexerData.ballCount == 2){
        /**
         * shooting code is separate
         * but at this point the indexer will not run anything if given "intake" command
         * 
         * 
         **/
        indexerBelt.Set(0);
        indexerWheel.Set(0);
    }
}

void Indexer::shootSequence(IndexerData &indexerData){   
    indexerData.ballCount = 0;

    indexerBelt.Set(IndexerBeltSpeed);
    indexerWheel.Set(IndexerWheelSpeed); // wheel speed doesn't really matter

    // need case to account for opponent color balls 
}

void Indexer::indexerBeltInit(){
    indexerBelt.RestoreFactoryDefaults();
    indexerBelt.SetInverted(false);
    indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerBelt.SetSmartCurrentLimit(45);
}

void Indexer::indexerWheelInit(){
    indexerWheel.RestoreFactoryDefaults();
    indexerWheel.SetInverted(false);
    indexerWheel.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    indexerWheel.SetSmartCurrentLimit(45);
}

void Indexer::processColor(const RobotData &robotData, IndexerData &indexerData){
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
        
        
    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){


    }
}


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