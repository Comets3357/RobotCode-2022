#include "controller/Controller.h"

// for updating states of control variables (to be accessed by other subsystems)
void Controller::updateControlData(const ControllerData &controllerData, ControlData &controlData)
{
    // states:
    controlData.shift = controllerData.sLBumper;
    if (controllerData.sRCenterBtnToggled)
    {
        controlData.manualMode = !controlData.manualMode;
    }
    if (controllerData.sRCenterBtnToggled)
    {
        controlData.climbMode = !controlData.climbMode;
    }

    // controls:

    // drivebase:
    // note: when pRShoulderSwitch is held, driving is sensitive to turning, while not held (default driving mode) driving is less sensitive to turning and good for quick staright movements and steady arcs (won't turn super easily)
    controlData.turnResponsive = controllerData.pRShoulderSwitch;
    if (controlData.turnResponsive)
    {
        controlData.maxStraight = 1;
        controlData.maxTurn = 1;
    }
    else
    {
        controlData.maxStraight = 1;
        controlData.maxTurn = 0.3;
    }

    controlData.dbInverted = controllerData.pLShoulderSwitch;
    // if you're inverted then you swtich sides for driving so it's intuitive
    if (controlData.dbInverted)
    {
        controlData.lDrive = -controllerData.pRYStick;
        controlData.rDrive = -controllerData.pLYStick;
    }
    else
    {
        controlData.lDrive = controllerData.pLYStick;
        controlData.rDrive = controllerData.pRYStick;
    }




    controlData.mIntakeDown = controllerData.sLYStick;
    controlData.mIntakeUp = controllerData.sYBtn;
    controlData.mIntakeRollers = controllerData.sRYStick;
    controlData.mzeroing = controllerData.sYBtn; // INTAKE ZEROING

    controlData.mHood = controllerData.sRYStick;
    controlData.mFlyWheel = controllerData.sABtn;

    controlData.mIndexerBackwards = controllerData.sBBtn;
    controlData.mIndexer = controllerData.sXBtn;



    controlData.saIntake = controllerData.sRBumper;
    controlData.saIntakeBackward = controllerData.sABtn;

    controlData.saShooting = controllerData.sXBtn;
    controlData.saEjectBalls = controllerData.sBBtn;
    //controlData.launchPadShot = controllerData.sRCenterBtn;
    //controlData.hubShot = controllerData.sLCenterBtn;
    //controlData.wrongBall = controllerData.sYBtn;

  
}