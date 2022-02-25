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
    bool topBeamToggledOn; // sensed a ball
    bool topBeamToggledOff; // stopped sensing a ball
  
};


class Indexer
{

public:
    void RobotInit();
    void AutonomousInit(IndexerData &indexerData);
    void RobotPeriodic(const RobotData &robotData, IndexerData &indexerData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, IndexerData &indexerData);
    void TestPeriodic(const RobotData &robotData, IndexerData &indexerData);

private:
    void updateData(const RobotData &robotData, IndexerData &indexerData);
    void manual(const RobotData &robotData, IndexerData &indexerData);
    void semiAuto(const RobotData &robotData, IndexerData &indexerData);

    void debuggingStuff(const RobotData &robotData, IndexerData &indexerData);

    void incrementCount(const RobotData &robotData, IndexerData &indexerData);
    void newCargo(const RobotData &robotData, IndexerData &indexerData);
    void assignCargoColor(const RobotData &robotData, IndexerData &indexerData);
    void decrementCount(const RobotData &robotData, IndexerData &indexerData, bool reverse);
    void mDecrement(const RobotData &robotData, IndexerData &indexerData);
    void count(const RobotData &robotData, IndexerData &indexerData);

    void saBeltControl(const RobotData &robotData, IndexerData &indexerData);
    void saWheelControl(const RobotData &robotData, IndexerData &indexerData);
    bool pauseBelt(const RobotData &robotData, IndexerData &indexerData);


    bool getBottomBeam();
    bool getMidBeam();
    bool getTopBeam();

    // get if it was toggled to state specified in bool broken
    bool getBottomBeamToggled(bool broken);
    bool getTopBeamToggled(bool broken); // not in use
    void updateTopBeamToggled(IndexerData &indexerData);

    void indexerBeltInit();
    void indexerWheelInit();

    
    
    frc::DigitalInput bottomBeamBreak{bottomBeamBreakPort};
    frc::DigitalInput midBeamBreak{midBeamBreakPort};
    frc::DigitalInput topBeamBreak{topBeamBreakPort};

    bool prevBottomBeam = false;
    bool prevTopBeam = false;

    // debounce counters to time debounce
    int bottomDebounceCount = 0;
    int topDebounceCount = 0;
    int pauseBeltCount = 0;

    bool runWheel = false; // checks if one ball has left shooter so that you can run the wheel and get the other ball out

    const double indexerWheelSpeed = 0.6;
    const double indexerBeltSpeed = 0.8;
    const double indexerIntakingBeltSpeed = 0.6;

    // ColorSensor colorSensor{}; //rev v3, for detecting ball color

    rev::CANSparkMax indexerBelt = rev::CANSparkMax(indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerBeltEncoder = indexerBelt.GetEncoder();
    rev::SparkMaxPIDController indexerBelt_pidController = indexerBelt.GetPIDController();

    rev::CANSparkMax indexerWheel = rev::CANSparkMax(indexerWheelID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerWheelEncoder = indexerWheel.GetEncoder();
    rev::SparkMaxPIDController indexerWheel_pidController = indexerWheel.GetPIDController();

};