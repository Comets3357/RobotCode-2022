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
    std::deque<Cargo> indexerContents;
    bool wrongBall = false;
};


class Indexer
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, IndexerData &indexerData);
    void DisabledInit();
    void updateData(const RobotData &robotData, IndexerData &indexerData);

private:
    void manual(const RobotData &robotData, IndexerData &indexerData);
    void semiAuto(const RobotData &robotData, IndexerData &indexerData);
    void testControl(const RobotData &robotData);
    void debuggingStuff(const RobotData &robotData, IndexerData &indexerData);

    void indexerBeltInit();
    void indexerWheelInit();

    void incrementCount(const RobotData &robotData, IndexerData &indexerData);
    void newCargo(const RobotData &robotData, IndexerData &indexerData);
    void assignCargoColor(const RobotData &robotData, IndexerData &indexerData);
    void decrementCount(const RobotData &robotData, IndexerData &indexerData, bool reverse);
    void mDecrement(const RobotData &robotData, IndexerData &indexerData);
    void count(const RobotData &robotData, IndexerData &indexerData);

    void saBeltControl(const RobotData &robotData, IndexerData &indexerData);
    void saWheelControl(const RobotData &robotData, IndexerData &indexerData);

    bool getBottomBeam();
    bool getMidBeam();
    bool getTopBeam();

    // get if it was toggled to state specified in bool broken
    bool getBottomBeamToggled(bool broken);
    // bool getMidBeamToggled(bool broken); // not in use
    bool getTopBeamToggled(bool broken);

    
    
    frc::DigitalInput bottomBeamBreak{bottomBeamBreakPort};
    frc::DigitalInput midBeamBreak{midBeamBreakPort};
    frc::DigitalInput topBeamBreak{topBeamBreakPort};

    bool prevBottomBeam = false;
    // bool prevMidBeam = false;
    bool prevTopBeam = false;

    // debounce counters to time debounce
    int bottomDebounceCount = 0;
    int topDebounceCount = 0;

    const double indexerWheelSpeed = 0.3;
    const double indexerBeltSpeed = 0.8;
    const double saIndexerWheelIntakeSpeed = 0.3;
    const double saIndexerBeltIntakeSpeed = 0.8;

    // ColorSensor colorSensor{}; //rev v3, for detecting ball color

    rev::CANSparkMax indexerBelt = rev::CANSparkMax(indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerBeltEncoder = indexerBelt.GetEncoder();
    rev::SparkMaxPIDController indexerBelt_pidController = indexerBelt.GetPIDController();

    rev::CANSparkMax indexerWheel = rev::CANSparkMax(indexerWheelID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerWheelEncoder = indexerWheel.GetEncoder();
    rev::SparkMaxPIDController indexerWheel_pidController = indexerWheel.GetPIDController();

};