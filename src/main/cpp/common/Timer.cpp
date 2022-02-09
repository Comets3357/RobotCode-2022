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
    timerData.secSinceEnabled = 0;
}

void Timer::EnabledPeriodic(TimerData &timerData)
{
    timerData.secSinceEnabled = timer.Get().to<double>();
    frc::SmartDashboard::PutNumber("timerData.secSinceEnabled", timerData.secSinceEnabled);
}

// void Timer::DisabledInit(TimerData &timerData)
// {
//     enabledSPointSet = false;
//     timerData.secSinceEnabled = 0;
// }