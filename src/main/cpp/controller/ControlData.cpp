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
            controlData.mode = mode_teleop_sa;
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

    
    controlData.mIntakeDown = controllerData.sRBumper /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mIntakeUp = controllerData.sRBumper && controlData.shift /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mIntakeRollersIn = controllerData.sRTrigger > 0.5 /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mIntakeRollersOut = controllerData.sRTrigger > 0.5 && controlData.shift /* && (controlData.mode == mode_teleop_manual) */;
    
    controlData.mZeroHood = controllerData.sLStickBtn /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mZeroTurret = controllerData.sRStickBtn /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mHood = controllerData.sLYStick/*  && (controlData.mode == mode_teleop_manual) */;
    controlData.mTurret = controllerData.sRXStick /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mShooterWheelForward = controllerData.sXBtn /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mShooterWheelBackward = controllerData.sXBtn && controlData.shift/*  && (controlData.mode == mode_teleop_manual) */;

    controlData.mSideWheelForward = controllerData.sBBtn/*  && (controlData.mode == mode_teleop_manual) */;
    controlData.mSideWheelBackward = controllerData.sBBtn && controlData.shift /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mCenterWheelForward = controllerData.sABtn /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mCenterWheelBackward = controllerData.sABtn && controlData.shift/*  && (controlData.mode == mode_teleop_manual) */;
    controlData.mIndexerUp = controllerData.sYBtn/*  && (controlData.mode == mode_teleop_manual) */;
    controlData.mIndexerDown = controllerData.sYBtn && controlData.shift/*  && (controlData.mode == mode_teleop_manual) */;
    controlData.mDecrementCargo = controllerData.sLCenterBtnToggled /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mIncrementCargo = controllerData.sRCenterBtnToggled /* && (controlData.mode == mode_teleop_manual) */;

    

    // semi-auto:

    
    controlData.saIntake = controllerData.sRTrigger > 0.5/*  && (controlData.mode == mode_teleop_sa) */;
    controlData.saIntakeBackward = controllerData.sLTrigger > 0.5 /* && (controlData.mode == mode_teleop_sa) */;

    controlData.saEjectBalls = controllerData.sABtn && !controlData.shift/*  && (controlData.mode == mode_teleop_sa) */;

    controlData.saShooting = controllerData.sXBtnToggled && !controlData.shift/* && (controlData.mode == mode_teleop_sa) */;
    controlData.saFinalShoot = controllerData.sYBtn && !controlData.shift/* && (controlData.mode == mode_teleop_sa); */;

    // secondary y to set readyshoot to true in testing

    if (controllerData.sRBumperToggled) {
        controlData.upperHubShot = !controlData.upperHubShot;
    }
    if (controllerData.sBBtnToggled) {
        controlData.shootUnassignedAsOpponent = !controlData.shootUnassignedAsOpponent;
    }
    controlData.fenderShot = controllerData.sABtnToggled && controlData.shift /* && (controlData.mode == mode_teleop_sa) */;
    controlData.sideWallShot = controllerData.sBBtnToggled && controlData.shift/*  && (controlData.mode == mode_teleop_sa) */;
    controlData.wallLaunchPadShot = controllerData.sXBtnToggled && controlData.shift/*  && (controlData.mode == mode_teleop_sa) */;
    controlData.cornerLaunchPadShot = controllerData.sYBtnToggled && controlData.shift /* && (controlData.mode == mode_teleop_sa) */;
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