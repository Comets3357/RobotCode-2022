#include "common/Timer.h"

#include <frc/DriverStation.h>

// void Timer::RobotInit(TimerData &timerData)
// {
//     timer.Reset();
//     timer.Start();
// }

void Timer::EnabledInit(TimerData &timerData) {
    timer.Reset();
    timer.Start();
    timerData.secSinceEnabled = timer.Get().to<double>();
}

void Timer::EnabledPeriodic(TimerData &timerData)
{
    timerData.secSinceEnabled = timer.Get().to<double>();
}

// void Timer::DisabledInit(TimerData &timerData)
// {
//     enabledSPointSet = false;
//     timerData.secSinceEnabled = 0;
// }