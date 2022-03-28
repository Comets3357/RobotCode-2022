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
            controlData.mode = mode_climb_manual;
            break;
        case 270:   // left
            controlData.mode = mode_climb_sa;
            break;
        default:
            controlData.mode = mode_teleop_sa;
            break;
            
    }

    frc::SmartDashboard::PutNumber("Y", controllerData.sRYStick);
    frc::SmartDashboard::PutNumber("X", controllerData.sRXStick);



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

    if(controllerData.sLCenterBtnToggled){
        controlData.staticTurret = !controlData.staticTurret;
    }

    // manual:

    //ADD CONTROLLER BIND
    if(controllerData.sLStickBtnToggled){
        controlData.saDistanceOffset = controlData.saDistanceOffset + 6;
    }else if(controllerData.sRStickBtnToggled){
        controlData.saDistanceOffset = controlData.saDistanceOffset - 6;
    }
    

    
    controlData.mIntakeDown = controllerData.sRBumper /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mIntakeUp = controllerData.sRBumper && controlData.shift /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mIntakeRollersIn = controllerData.sRTrigger > 0.5 /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mIntakeRollersOut = controllerData.sRTrigger > 0.5 && controlData.shift /* && (controlData.mode == mode_teleop_manual) */;
    
    controlData.mZeroHood = controllerData.sRStickBtn /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mZeroTurret = controllerData.sLStickBtn /* && (controlData.mode == mode_teleop_manual) */;
    controlData.mHood = controllerData.sRYStick/*  && (controlData.mode == mode_teleop_manual) */;
    controlData.mTurret = controllerData.sLXStick /* && (controlData.mode == mode_teleop_manual) */;
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

    
    controlData.saIntake = (controllerData.sRTrigger > 0.5) && !controlData.shift;/*  && (controlData.mode == mode_teleop_sa) */;
    controlData.saIntakeBackward = controllerData.sLTrigger > 0.5 /* && (controlData.mode == mode_teleop_sa) */;
    controlData.saIntakeIdle = (controllerData.sRTrigger > 0.5) && controlData.shift;
    frc::SmartDashboard::PutBoolean("saIntakeIdle", controlData.saIntakeIdle);

    controlData.saEjectBalls = controllerData.sABtn && !controlData.shift/*  && (controlData.mode == mode_teleop_sa) */;

    controlData.saShooting = controllerData.sXBtnToggled && !controlData.shift/* && (controlData.mode == mode_teleop_sa) */;
    controlData.saFinalShoot = controllerData.sYBtn && !controlData.shift/* && (controlData.mode == mode_teleop_sa); */;
    
    if(controllerData.sRCenterBtnToggled){
        controlData.staticTurret = !controlData.staticTurret;
    }

    // secondary y to set readyshoot to true in testing

    // disabled because there has not yet been a case where we want to shoot low hub
    if (controllerData.sRBumperToggled) {
        // controlData.upperHubShot = !controlData.upperHubShot;
    }
    if (controllerData.sBBtnToggled) {
        controlData.autoRejectOpponentCargo = !controlData.autoRejectOpponentCargo;
    }
    
    controlData.fenderShot = controllerData.sABtnToggled && controlData.shift /* && (controlData.mode == mode_teleop_sa) */;
    controlData.sideWallShot = controllerData.sBBtnToggled && controlData.shift/*  && (controlData.mode == mode_teleop_sa) */;
    controlData.wallLaunchPadShot = controllerData.sXBtnToggled && controlData.shift/*  && (controlData.mode == mode_teleop_sa) */;
    controlData.cornerLaunchPadShot = controllerData.sYBtnToggled && controlData.shift /* && (controlData.mode == mode_teleop_sa) */;

    //TURRET DIRECTION converts joystick (x,y) into degrees (0 is right) UNIT CIRCLE
    double x = -controllerData.sLXStick;
    double y = controllerData.sLYStick;

    //check to make sure you're out of the deadzone
    if(x > 0.1 || y > 0.1 || x < -0.1 || y < -0.1){
        controlData.usingTurretDirection = true;
        //does the conversions and accounts for different quadrants of the unit circle
        if((x <0.1 && x > -0.1)  && y > 0){ //covers if @ up direction
            controlData.saTurretDirectionController = 90;
        }else if(x > 0 && y >= 0){ //covers 0 ~ 89 degrees
            controlData.saTurretDirectionController = (std::atan(y/x)*(180/pi));
        }else if((y >= 0 && x < 0) || (y < 0 && x < 0)){  //covers 91 ~ 269 degrees
            controlData.saTurretDirectionController = (std::atan(y/x)*(180/pi)) + 180;
        }else if((x <0.1 && x > -0.1) && y < 0){
            controlData.saTurretDirectionController = 270;
        }else if(y < 0 && x > 0){ //covers 271 ~ 359 degrees 
            controlData.saTurretDirectionController = (std::atan(y/x)*(180/pi)) + 360;
        }

        
        controlData.saTurretDirectionController = (controlData.saTurretDirectionController - 90);
        if(controlData.saTurretDirectionController < 0){
            controlData.saTurretDirectionController += 360;
        }

    } else{
        controlData.usingTurretDirection = false;
    }

    //if we need 0 degrees to be up we can just add 90 to all these numbers right???
    

    

    
    // if(robotData.indexerData.indexerContents.front() == Cargo::cargo_Opponent){
    //     controlData.wrongBall = true;
    // } else if(robotData.indexerData.indexerContents.front() == Cargo::cargo_Alliance){
    //     controlData.wrongBall = false;
    // } else if (robotData.indexerData.indexerContents.front() == Cargo::cargo_Unassigned){
    //     controlData.wrongBall = true; 
        // change to button for driver control?
    // }
    //controlData.finalShoot;
  
    //CLIMB
    controlData.sapauseSequence = controllerData.sLStickBtn;
    controlData.sacancelSequence = controllerData.sRStickBtn;
    controlData.saclimbTraversalSequence = controllerData.sLCenterBtn;
    controlData.saclimbHeightSequence = controllerData.sRCenterBtn;
    controlData.saclimbInit = controllerData.sBBtn;
    controlData.climbZeroing = controllerData.sABtnToggled;

    //BENCH TEST

    //toggle buttons, part of index 2 (third controller - aka test controller) on drivers station
    //we're not making it in the normal structure because that would be a lot of work - tananya
    controlData.incrementMotor = controllerData.testBButton; // b: increments what motor/encoder/thing that you're testing
    controlData.incrementSpeed = controllerData.testXButton; // x: increases the speed
    controlData.PIDModeToggle = controllerData.testYButton; //toggles pid mode (if we want to test pids)
    controlData.incrementSubsystem = controllerData.testAButton; //increments the subsystem

    //sets the value of the variables used in robot.cpp based upon the toggle button variables
    if (!controlData.autoBenchTest){ //can't do manual bench test in automatic bench test
        if (controllerData.testRBumper) controlData.manualBenchTest = !controlData.manualBenchTest; //a: starts and stops manual bench test
    }
    
    if (!controlData.manualBenchTest){ //can't do automatic bench test in manual bench test
        if (controllerData.testLBumper) controlData.autoBenchTest = !controlData.autoBenchTest; //left bumper: starts and stops automatic bench test
    }
}

void Controller::updateShootMode(const RobotData &robotData, ControlData &controlData) {
    // pressing a shoot button will set the robot to be in the associated shooting mode. if you press the button again, it will toggle that shoot mode off.

    if (controlData.mode != mode_teleop_sa) {
        controlData.shootMode = shootMode_none;
        return;
    }

    if (robotData.controlData.saShooting) {
        if (controlData.shootMode == shootMode_vision) {
            controlData.shootMode = shootMode_none;
        } else {controlData.shootMode = shootMode_vision; }
    }

    if (robotData.controlData.fenderShot) {
        if (controlData.shootMode == shootMode_fender) {
            controlData.shootMode = shootMode_none;
        } else { controlData.shootMode = shootMode_fender; }
    }

    if (robotData.controlData.sideWallShot) {
        if (controlData.shootMode == shootMode_sideWall) {
            controlData.shootMode = shootMode_none;
        } else { controlData.shootMode = shootMode_sideWall; }
    }

    if (robotData.controlData.wallLaunchPadShot) {
        if (controlData.shootMode == shootMode_wallLaunchPad) {
            controlData.shootMode = shootMode_none;
        } else { controlData.shootMode = shootMode_wallLaunchPad; }
    }

    if (robotData.controlData.cornerLaunchPadShot) {
        if (controlData.shootMode == shootMode_cornerLaunchPad) {
            controlData.shootMode = shootMode_none;
        } else { controlData.shootMode = shootMode_cornerLaunchPad; }
    }



    // shut off shooting if all balls have exited (happens once upon ball count going to zero)
    // if (robotData.indexerData.eBallCountZero) {
        // controlData.shootMode = shootMode_none;
    // }
    // disabled at at muskegon, inconsistent ball counting
}