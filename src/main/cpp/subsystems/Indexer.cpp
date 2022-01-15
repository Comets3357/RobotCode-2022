#include "subsystems/Indexer.h"
#include "RobotData.h"

void Indexer::RobotInit()
{
    indexerBeltInit();
    indexerWheelInit();
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
    if(robotData.controlData.saEjectBalls){
        indexerBelt.Set(-mIndexerBeltSpeed);
        indexerWheel.Set(-mIndexerWheelSpeed);
    }else if(robotData.controlData.saIntake){
        indexerBelt.Set(saIndexerBeltIntakeSpeed);
        indexerWheel.Set(saIndexerWheelIntakeSpeed);
    }else{
        indexerBelt.Set(0);
        indexerWheel.Set(0);
    }
}

void Indexer::manual(const RobotData &robotData, IndexerData &indexerData){
    if(robotData.controlData.mIndexerBackwards){
        indexerBelt.Set(-mIndexerBeltSpeed);
        indexerWheel.Set(-mIndexerWheelSpeed);
    }else if(robotData.controlData.mIndexer){
        indexerBelt.Set(mIndexerBeltSpeed);
        indexerWheel.Set(mIndexerWheelSpeed);    
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