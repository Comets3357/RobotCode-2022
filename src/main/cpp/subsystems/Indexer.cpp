#include "subsystems/Indexer.h"
#include "RobotData.h"

void Indexer::RobotInit()
{
    
   
}

void Indexer::RobotPeriodic(const RobotData &robotData, IndexerData &indexerData)
{
    updateData(robotData, indexerData);

}

void Indexer::DisabledInit()
{
    
}

// updates encoder and gyro values
void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    
}
