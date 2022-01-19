#include "subsystems/Indexer.h"
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
    if(robotData.controlData.saEjectBalls){ //run belt and wheel backwards
        indexerBelt.Set(-IndexerBeltSpeed);
        indexerWheel.Set(-IndexerWheelSpeed);
    }else if(robotData.shooterData.readyShoot){
        indexerData.ballCount = 0;
        indexerBelt.Set(IndexerBeltSpeed);
        indexerWheel.Set(IndexerWheelSpeed);
    
    }else if(robotData.controlData.saIntake){ //if intaking run belt and wheels forward
        //once the first prox sensor senses first ball, run indexer (ball count@1)
            //index the first ball until the first prox sensor runs false
                //intake second ball, once the first prox sensor senses second ball run indexer until first ball hits second sensor (ball count@2)
        if(proxIndexerBottom.Get()){
            indexerBelt.Set(saIndexerBeltIntakeSpeed);
            indexerWheel.Set(saIndexerWheelIntakeSpeed);
        }
        
    }else{
        indexerBelt.Set(0);
        indexerWheel.Set(0);
    }

    if(robotData.controlData.saIntake){ // you are intaking
        if(indexerData.ballCount == 0){
            if(!proxIndexerBottom.Get()) { // false if it senses somthing
                if (proxIndexerMiddle.Get()){ // until mid sensor is tripped, run both belt and wheel
                    indexerBelt.Set(saIndexerBeltIntakeSpeed);
                    indexerWheel.Set(saIndexerWheelIntakeSpeed);
                } else if (!proxIndexerMiddle.Get()){ // when mid sensor is tripped, stop belt
                    indexerBelt.Set(saIndexerBeltIntakeSpeed);
                    indexerWheel.Set(saIndexerWheelIntakeSpeed);
                }
            }
            if(proxIndexerMiddle.Get()) { // UNTIL mid sensor is tripped
                if (!proxIndexerBottom.Get()){ // if bottom sensor is tripped run both wheel and belt
                    indexerBelt.Set(saIndexerBeltIntakeSpeed);
                    indexerWheel.Set(saIndexerWheelIntakeSpeed);
                } 
            }
        } else if (indexerData.ballCount == 1){

        } else if (indexerData.ballCount ==2){

        }
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

// updates encoder and gyro values
void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    
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