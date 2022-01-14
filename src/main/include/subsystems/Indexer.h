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

    /**sensors if needed
    frc::DigitalInput indexerfront{2};
    **/


    //CHANGE MOTOr ID STUFF  (just outline lol don't take your life too seriously:))
    rev::CANSparkMax indexerBelt = rev::CANSparkMax(indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerBeltEncoder = indexerBelt.GetEncoder();
    rev::SparkMaxPIDController indexerBelt_pidController = indexerBelt.GetPIDController();



};