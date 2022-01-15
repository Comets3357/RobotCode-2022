#include "subsystems/Indexer.h"
#include "RobotData.h"

void Indexer::RobotInit()
{
     
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

void Indexer::indexerBeltInit(){
    indexerBelt.RestoreFactoryDefaults();

    indexerBelt.SetInverted(false);

    indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    indexerBelt_pidController.SetP(wkP);
    indexerBelt_pidController.SetI(wkI);
    indexerBelt_pidController.SetD(wkD);
    indexerBelt_pidController.SetIZone(wkIz);
    indexerBelt_pidController.SetFF(wkFF);
    indexerBelt_pidController.SetOutputRange(wkMinOutput, wkMaxOutput);

    indexerBelt.SetSmartCurrentLimit(45);
}

void Indexer::indexerWheelInit(){
    indexerWheel.RestoreFactoryDefaults();

    indexerWheel.SetInverted(false);

    indexerWheel.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    indexerBelt_pidController.SetP(wkP);
    indexerBelt_pidController.SetI(wkI);
    indexerBelt_pidController.SetD(wkD);
    indexerBelt_pidController.SetIZone(wkIz);
    indexerBelt_pidController.SetFF(wkFF);
    indexerBelt_pidController.SetOutputRange(wkMinOutput, wkMaxOutput);

    indexerWheel.SetSmartCurrentLimit(45);

}