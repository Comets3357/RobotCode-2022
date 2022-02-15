#pragma once

#include <frc/Timer.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>

struct TimerData
{
    double secSinceInit = 0;
    double secSinceEnabled = 0;  // use this most of the time
    double secRemaining;
};

class Timer
{

public:
    void RobotInit(TimerData &timerData);
    void EnabledPeriodic(TimerData &timerData);
    void EnabledInit(TimerData &timerData);
    void DisabledInit(TimerData &timerData);

private:
    frc::Timer timer{};
    
};