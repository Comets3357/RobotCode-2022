#include "common/LEDs.h"

void LEDs::RobotInit()
{

}

void LEDs::TeleopInit()
{

}
void LEDs::RobotPeriodic()
{
    arduino.Write(1, 1);
}