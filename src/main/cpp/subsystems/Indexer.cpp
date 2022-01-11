#include "subsystems/Indexer.h"
#include "RobotData.h"

void Indexer::RobotInit()
{
    indexerBelt.RestoreFactoryDefaults();

    indexerBelt.SetInverted(true);

    indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    indexerBelt.SetSmartCurrentLimit(45);   
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

}

void Indexer::manual(const RobotData &robotData, IndexerData &indexerData){
    
}

void Indexer::DisabledInit()
{
    
}

// updates encoder and gyro values
void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    
}
