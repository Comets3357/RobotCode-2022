#pragma once

#include "Constants.h"
#include "common/ColorSensor.h"

#include <frc/DriverStation.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/DigitalInput.h>

#include <deque>

struct RobotData;

enum Cargo
{
    cargo_Alliance,
    cargo_Opponent
};

struct IndexerData
{
    int ballCount = 0;
    std::deque<Cargo> indexerContents;
    
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

    void processColor(const RobotData &robotData, IndexerData &indexerData);

    void indexerBeltInit();
    void indexerWheelInit();
    void intakeSequence(IndexerData &indexerData);
    void shootSequence(IndexerData &indexerData);

    void testControl(const RobotData &robotData);

    bool getBottomBeam();
    bool getMidBeam();
    bool getTopBeam();

    // need to make constants for these indexes??
    frc::DigitalInput bottomBeamBreak{1};
    frc::DigitalInput midBeamBreak{2};
    frc::DigitalInput topBeamBreak{3};

    bool firstSensorTripped = false;
    bool secondSensorTripped = false;


    const double IndexerWheelSpeed = 0.2;
    const double IndexerBeltSpeed = 0.2;
    const double saIndexerWheelIntakeSpeed = 0.2;
    const double saIndexerBeltIntakeSpeed = 0.2;

    ColorSensor colorSensor{}; //rev v3, for detecting ball color

    rev::CANSparkMax indexerBelt = rev::CANSparkMax(indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerBeltEncoder = indexerBelt.GetEncoder();
    rev::SparkMaxPIDController indexerBelt_pidController = indexerBelt.GetPIDController();

    rev::CANSparkMax indexerWheel = rev::CANSparkMax(indexerWheelID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerWheelEncoder = indexerWheel.GetEncoder();
    rev::SparkMaxPIDController indexerWheel_pidController = indexerWheel.GetPIDController();

};