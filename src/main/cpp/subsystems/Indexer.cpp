// #include "subsystems/Indexer.h"
// #include "RobotData.h"

// void Indexer::RobotInit()
// {
//     indexerBelt.RestoreFactoryDefaults();

//     indexerBelt.SetInverted(true);

//     indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

//     indexerBelt.SetSmartCurrentLimit(45);   
// }

// void Indexer::RobotPeriodic(const RobotData &robotData, IndexerData &indexerData)
// {
//     updateData(robotData, indexerData);
//     if (robotData.controlData.manualMode)
//     {
//         manual(robotData, indexerData);
//     }
//     else
//     {
//         semiAuto(robotData, indexerData);
//     }
// }

// void Indexer::semiAuto(const RobotData &robotData, IndexerData &indexerData){
//     if(robotData.controlData.saEjectBalls){ //run belt and wheel backwards
//         indexerBelt.Set(-IndexerBeltSpeed);
//         indexerWheel.Set(-IndexerWheelSpeed);
//     // }else if((robotData.shooterData.readyShoot && robotData.controlData.finalShoot) || robotData.shooterData.wrongBallReady){
//     //     indexerData.ballCount = 0;
//     //     indexerBelt.Set(IndexerBeltSpeed);
//     //     indexerWheel.Set(IndexerWheelSpeed);
    
//     }else if(robotData.controlData.saIntake){ //if intaking run belt and wheels forward
//         //once the first prox sensor senses first ball, run indexer (ball count@1)
//             //index the first ball until the first prox sensor runs false
//                 //intake second ball, once the first prox sensor senses second ball run indexer until first ball hits second sensor (ball count@2)
//         if(proxIndexerFront.Get()){
//             indexerBelt.Set(saIndexerBeltIntakeSpeed);
//             indexerWheel.Set(saIndexerWheelIntakeSpeed);
//         }
        
        
//     }else{
//         indexerBelt.Set(0);
//         indexerWheel.Set(0);
//     }
// }

// void Indexer::manual(const RobotData &robotData, IndexerData &indexerData){
    
// }

// void Indexer::DisabledInit()
// {
    
// }

// // updates encoder and gyro values
// void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
// {
    
// }
