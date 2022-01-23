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
    cargo_Opponent,
    cargo_Unassigned 
};

struct IndexerData
{
    int ballCount = 0;
    std::deque<Cargo> indexerContents;

    // not a toggle, just what's actually 
    bool bottomSensor = false;
    bool midSensor = false;
    bool topSensor = false;
    
    bool bottomSensorToggledOn = false;

    bool reverseIndexer = false;
    
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
    void intakeSequence(IndexerData &indexerData);
    void shootSequence(const RobotData &robotData, IndexerData &indexerData);

    void incrementCount(const RobotData &robotData, IndexerData &indexerData);
    void decrementCount(const RobotData &robotData, IndexerData &indexerData, bool reverse);
    void saBeltControl(const RobotData &robotData, IndexerData &indexerData);
    void saWheelControl(const RobotData &robotData, IndexerData &indexerData);

    void testControl(const RobotData &robotData);

    void count(const RobotData &robotData, IndexerData &indexerData);

    bool getBottomBeam();
    bool getMidBeam();
    bool getTopBeam();

    // get if it was toggled to state specified in bool broken
    bool getBottomBeamToggled(bool broken);
    // bool getMidBeamToggled(bool broken); // not in use
    bool getTopBeamToggled(bool broken);

    // need to make constants for these indexes??
    frc::DigitalInput bottomBeamBreak{1};
    frc::DigitalInput midBeamBreak{2};
    frc::DigitalInput topBeamBreak{3};

    bool firstSensorTripped = false;
    bool secondSensorTripped = false;

    bool prevBottomBeam = false;
    bool prevMidBeam = false;
    bool prevTopBeam = false;


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