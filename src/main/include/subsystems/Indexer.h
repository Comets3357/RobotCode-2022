#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/DigitalInput.h>

struct RobotData;

struct IndexerData
{
    int ballCount = 0;
};

class Indexer
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, IndexerData &indexerData);
    
    void DisabledInit();

private:
    void updateData(const RobotData &robotData, IndexerData &indexerData);
    void manual(const RobotData &robotData, IndexerData &indexerData);
    void semiAuto(const RobotData &robotData, IndexerData &indexerData);

    void indexerBeltInit();
    void indexerWheelInit();

    // need to make constants for these indexes??
    frc::DigitalInput proxIndexerBottom{1};
    frc::DigitalInput proxIndexerMiddle{2};
    frc::DigitalInput proxIndexerTop{3};


    const double IndexerWheelSpeed = 0.2;
    const double IndexerBeltSpeed = 0.2;
    const double saIndexerWheelIntakeSpeed = 0.2;
    const double saIndexerBeltIntakeSpeed = 0.2;


    //CHANGE MOTOr ID STUFF  (just outline lol don't take your life too seriously:))
    rev::CANSparkMax indexerBelt = rev::CANSparkMax(indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerBeltEncoder = indexerBelt.GetEncoder();
    rev::SparkMaxPIDController indexerBelt_pidController = indexerBelt.GetPIDController();

    rev::CANSparkMax indexerWheel = rev::CANSparkMax(indexerWheelID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerWheelEncoder = indexerWheel.GetEncoder();
    rev::SparkMaxPIDController indexerWheel_pidController = indexerWheel.GetPIDController();

};