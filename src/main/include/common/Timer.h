#pragma once

//#include <frc/Timer.h>

struct TimerData
{
    double secSinceInit = 0;
    double secSinceEnabled = 0;  // use this most of the time
};

class Timer
{

public:
    void RobotInit(TimerData &timerData);
    void RobotPeriodic(TimerData &timerData);
    void DisabledInit();

private:
    // frc::Timer timer{};
    // bool enabledSPointSet = false;
    // double enabledStartPoint = 0;
    // double teleopStartPoint = 0;
};