#include "controller/Controller.h"

/**
 * Notes:
 * 
 * GetRawButton and GetRawButtonPressed are not the same,
 * GetRawButton is for current state, GetRawButtonPressed is for toggle
 * button index starts at 1
 */

void Controller::TeleopPeriodic(const RobotData &robotData, ControllerData &controllerData, ControlData &controlData)
{
    updateBtnData(controllerData);
    updateControlData(robotData, controllerData, controlData);
}

bool Controller::getBtn(int js, int index)
{
    if (js == 1)
    {
        return secondary.GetRawButton(index);
    }
    else
    {
        return primary.GetRawButton(index);
    }
}

bool Controller::getBtnToggled(int js, int index)
{
    if (js == 1)
    {
        return secondary.GetRawButtonPressed(index);
    }
    else
    {
        return primary.GetRawButtonPressed(index);
    }
}

int Controller::getPOV(int js, int index)
{
    if (js == 1)
    {
        return secondary.GetPOV(index);
    }
    else
    {
        return primary.GetPOV(index);
    }
}

double Controller::getAxis(int js, int index)
{
    if (js == 1)
    {
        return secondary.GetRawAxis(index);
    }
    else
    {
        return primary.GetRawAxis(index);
    }
}

// for updating states of button variables
void Controller::updateBtnData(ControllerData &controllerData)
{
    // primary controls:

    if (frc::DriverStation::GetJoystickName(0) == "FrSky Taranis Joystick")
    {
        // if using the flight stick, axises are inverted compared to xbox
        controllerData.pLYStick = getAxis(0, 0);
        controllerData.pRYStick = getAxis(0, 2);
    }
    else
    {
        controllerData.pLYStick = -getAxis(0, 1);
        controllerData.pRYStick = -getAxis(0, 5);
    }

    controllerData.pLShoulderSwitch = getBtn(0, 2);
    controllerData.pRShoulderSwitch = getBtn(0, 1);

    //secondary controls:

    controllerData.sLXStick = -getAxis(1, 1);
    controllerData.sLYStick = -getAxis(1, 1);
    controllerData.sRXStick = -getAxis(1, 5);
    controllerData.sRYStick = -getAxis(1, 5);

    controllerData.sLStickBtn = getBtn(1, 9);
    controllerData.sRStickBtn = getBtn(1, 10);

    controllerData.sLTrigger = getAxis(1, 2);
    controllerData.sRTrigger = getAxis(1, 3);
    controllerData.sLBumper = getBtn(1, 5);
    controllerData.sRBumper = getBtn(1, 6);

    controllerData.sABtn = getBtn(1, 1);
    controllerData.sBBtn = getBtn(1, 2);
    controllerData.sXBtn = getBtn(1, 3);
    controllerData.sYBtn = getBtn(1, 4);

    controllerData.sABtnToggled = getBtnToggled(1, 1);
    controllerData.sBBtnToggled = getBtnToggled(1, 2);
    controllerData.sXBtnToggled = getBtnToggled(1, 3);
    controllerData.sYBtnToggled = getBtnToggled(1, 4);

    controllerData.sLCenterBtn = getBtn(1, 7);
    controllerData.sRCenterBtn = getBtn(1, 8);

    controllerData.sLCenterBtnToggled = getBtnToggled(1, 7);
    controllerData.sRCenterBtnToggled = getBtnToggled(1, 8);

    controllerData.sDPad = getPOV(1, 0);

    controllerData.sRTriggerToggled = getBtnToggled(1,3);
    controllerData.sLTriggerToggled = getBtnToggled(1,3);


}