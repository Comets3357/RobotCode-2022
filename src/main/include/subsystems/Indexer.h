#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/DigitalInput.h>

#include <deque>

struct RobotData;

// when you're having the deque show what's in the indexer
enum Cargo
{
    cargo_Alliance,
    cargo_Opponent,
    cargo_Unassigned 
};

struct IndexerData
{
    std::deque<Cargo> indexerContents; // what's in the indexer

    bool autoRejectTop;     // true if we are auto ejecting an opponent ball out the top of the indexer
    bool autoRejectBottom;  // true if we are auto ejecting an opponent ball out the bottom of the indexer
    
    bool eBallCountZero; // event boolean for when ball count goes from 1 to 0
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

    // void debuggingStuff(const RobotData &robotData, IndexerData &indexerData);

    void incrementCount(const RobotData &robotData, IndexerData &indexerData);
    void newCargo(const RobotData &robotData, IndexerData &indexerData);
    void assignCargoColor(const RobotData &robotData, IndexerData &indexerData);
    void decrementCount(const RobotData &robotData, IndexerData &indexerData, bool reverse);
    void mDecrement(const RobotData &robotData, IndexerData &indexerData);
    void count(const RobotData &robotData, IndexerData &indexerData);

    bool pauseBelt(const RobotData &robotData, IndexerData &indexerData); 
    void saBeltControl(const RobotData &robotData, IndexerData &indexerData);
    void saWheelControl(const RobotData &robotData, IndexerData &indexerData);
    

    void rejectDetection(const RobotData &robotData, IndexerData &indexerData);

    // basic sensor getters
    bool getBottomBeam();
    bool getMidBeam();
    bool getTopBeam();

    // for sensor toggles
    void updateSensors();
    bool getTopBeamToggledOff();
    bool getTopBeamToggledOn();
    bool getBottomBeamToggledOff();
    bool getBottomBeamToggledOn();

    void indexerBeltInit();
    void indexerWheelInit();

    void debuggingStuff(const RobotData &robotData, IndexerData &indexerData);
    
    frc::DigitalInput bottomBeamBreak{bottomBeamBreakPort};
    frc::DigitalInput midBeamBreak{midBeamBreakPort};
    frc::DigitalInput topBeamBreak{topBeamBreakPort};

    bool prevBottomBeam = false;
    bool currentBottomBeam = false;
    bool prevTopBeam = false;
    bool currentTopBeam = false;

    // variables for auto rejection of opponent balls
    bool ejectTop = false;

    // debounce counters to time debounce
    int bottomDebounceCount = 0;
    int topDebounceCount = 0;
    int pauseBeltCount = 0;
    int autoRejectBottomCount = 0;

    const double indexerWheelSpeed = 0.6;
    const double indexerShootingBeltSpeed = 0.8;
    const double indexerIntakingBeltSpeed = 0.27;

    int lastTickBallCount;

    rev::CANSparkMax indexerBelt = rev::CANSparkMax(indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerBeltEncoder = indexerBelt.GetEncoder();
    rev::SparkMaxPIDController indexerBelt_pidController = indexerBelt.GetPIDController();

    rev::CANSparkMax indexerWheel = rev::CANSparkMax(indexerWheelID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder indexerWheelEncoder = indexerWheel.GetEncoder();
    rev::SparkMaxPIDController indexerWheel_pidController = indexerWheel.GetPIDController();

};