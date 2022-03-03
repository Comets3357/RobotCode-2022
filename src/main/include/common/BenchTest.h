#pragma once

#include "RobotData.h"
#include "Constants.h"

struct RobotData;

enum BenchTestStage {
    BenchTestStage_Climb,
    BenchTestStage_Drivebase,
    BenchTestStage_Indexer,
    BenchTestStage_Intake,
    BenchTestStage_Shooter
};
 
struct BenchTestData {
    int testStage = BenchTestStage::BenchTestStage_Climb; //sets the starting stage of the bench test
    float currentSpeed = 0; //sets the speed of the motor we're currently testing
    int stage = 0; //sets which motor we're currently testing
    bool PIDMode = false; //toggles pid testing
};

class BenchTest {

    public:
        void RobotInit();
        void RobotPeriodic(const RobotData &robotData);
        void DisabledInit();
        void TestPeriodic(const RobotData &robotData, BenchTestData &benchTestData);

    private:
    
};