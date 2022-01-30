#include "controller/Controller.h"
#include "RobotData.h"

// for updating states of control variables (to be accessed by other subsystems)
void Controller::updateControlData(const RobotData &robotData, const ControllerData &controllerData, ControlData &controlData)
{
    // states:
    controlData.shift = controllerData.sLBumper;
    
    switch (controllerData.sDPad) {
        case -1:
            break;
        case 0: // up
            controlData.mode = mode_teleop_manual;
            break;
        case 90:    // right
            controlData.mode = mode_teleop_sa;
            break;
        case 180:   // down
            controlData.mode = mode_climb_sa;
            break;
        case 270:   // left
            controlData.mode = mode_climb_manual;
            break;
        default:
            break;
            
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



    // manual:

    controlData.mIntakeDown = controllerData.sRBumper;
    controlData.mIntakeUp = controllerData.sRBumper && controlData.shift;
    controlData.mIntakeRollersIn = controllerData.sRTrigger > 0.5;
    controlData.mIntakeRollersOut = controllerData.sRTrigger > 0.5 && controlData.shift;
    
    controlData.mZeroHood = controllerData.sLStickBtn;
    controlData.mZeroTurret = controllerData.sRStickBtn;
    controlData.mHood = controllerData.sLYStick;
    controlData.mTurret = controllerData.sRXStick;
    controlData.mShooterWheelForward = controllerData.sXBtn;
    controlData.mShooterWheelBackward = controllerData.sXBtn && controlData.shift;

    controlData.mSideWheelForward = controllerData.sBBtn;
    controlData.mSideWheelBackward = controllerData.sBBtn && controlData.shift;
    controlData.mCenterWheelForward = controllerData.sABtn;
    controlData.mCenterWheelBackward = controllerData.sABtn && controlData.shift;
    controlData.mIndexerUp = controllerData.sYBtn;
    controlData.mIndexerDown = controllerData.sYBtn && controlData.shift;
    controlData.mDecrementCargo = controllerData.sLCenterBtnToggled;
    controlData.mIncrementCargo = controllerData.sRCenterBtnToggled;

    

    // semi-auto:

    
    controlData.saIntake = controllerData.sRTrigger > 0.5;
    controlData.saIntakeBackward = controllerData.sLTrigger > 0.5;

    controlData.saEjectBalls = controllerData.sABtn;

    controlData.saShooting = controllerData.sXBtn;
    controlData.saFinalShoot = controllerData.sYBtn;

    // secondary y to set readyshoot to true in testing

    controlData.upperHubShot = controllerData.sRBumperToggled;
    controlData.fenderShot = controllerData.sABtn && controlData.shift;
    controlData.sideWallShot = controllerData.sBBtn && controlData.shift;
    controlData.wallLaunchPadShot = controllerData.sXBtn && controlData.shift;
    controlData.cornerLaunchPadShot = controllerData.sYBtn && controlData.shift;
    //controlData.hubShot = controllerData.sLCenterBtn; // DEPRECATED -brian

    
    // if(robotData.indexerData.indexerContents.front() == Cargo::cargo_Opponent){
    //     controlData.wrongBall = true;
    // } else if(robotData.indexerData.indexerContents.front() == Cargo::cargo_Alliance){
    //     controlData.wrongBall = false;
    // } else if (robotData.indexerData.indexerContents.front() == Cargo::cargo_Unassigned){
    //     controlData.wrongBall = true; 
        // change to button for driver control?
    // }
    //controlData.finalShoot;
    

  
}