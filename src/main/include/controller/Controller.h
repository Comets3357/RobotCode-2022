#pragma once

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

struct RobotData;

enum Mode {
    mode_teleop_sa,
    mode_teleop_manual,
    mode_climb_sa,
    mode_climb_manual
};

struct ControlData
{
    // states:
    Mode mode{mode_teleop_sa};
    bool shift = false;

    // drivebase:
    double lDrive;
    double rDrive;
    bool turnResponsive;
    bool dbInverted;
    double maxStraight = 1;
    double maxTurn = 0.4;

    //intake:
    bool mIntakeDown; 
    //brings the intake down MANUAL (hold)
    bool mIntakeUp; // (hold)
    bool mIntakeRollersIn; //runs intake forward MANUAL (axis)
    bool mIntakeRollersOut; //runs intake backward MANUAL (axis)
    bool mZeroHood; //set the hood encoder to zero MANUAL
    bool mZeroTurret; //set the turret encoder to zero MANUAL
    bool saIntake; //runs the intake rollers and brings intake down and the indexer to intake balls SEMIAUTO
    bool saIntakeBackward; //runs the intake backwards SEMIAUTO

    //indexer:
    bool mSideWheelForward;
    bool mSideWheelBackward;
    bool mCenterWheelForward;
    bool mCenterWheelBackward;
    bool mIndexerDown; //runs indexer backwards MANUAL
    bool mIndexerUp; //runs indexer foward MANUAL
    bool saEjectBalls; //runs intake and indexer backwards to eject balls SEMIAUTO
    bool mDecrementCargo; // manually decrements the amount of cargo in the indexer from the front of the deque
    bool mIncrementCargo; // manually increments the amount of cargo in the indexer from the front of the deque

    //shooter:
    bool saShooting; //gets hood at right angle, shooter wheel up to speed SEMIAUTO
    bool saFinalShoot; //makes belts run to actually fire balls SEMIAUTO

    bool upperHubShot = true;
    bool cornerLaunchPadShot; //fixed long shot from the launch pad to upper hub SEMIAUTO
    bool wallLaunchPadShot;
    bool sideWallShot;
    bool fenderShot;
    bool hubShot;//fixed close shot to lower hub from infront of hub SEMIAUTO
    bool shootUnassignedAsOpponent = false;
    bool wrongBall; //if the ball isn't our alliance color eject ball out shooter SEMIAUTO  DEPRECATED (using ControlData.shootUnassignedAsOpponent)
    bool mShooterWheelForward; //get flywheel running MANUAL
    bool mShooterWheelBackward; //get flywheel running backward MANUAL
    double mHood; //moves hood up or down MANUAL (axis)
    double mTurret; // moves turret left or right MANUAL (axis)

    //climb:
    bool saclimbTraversalSequence;
    bool saclimbHeightSequence;
    bool sacancelSequence;
    bool sapivotArmsIn;
    bool sapivotArmsOut;
    bool saretractElevator;
    bool saextendElevator;
    bool saclimbInit;
};

struct ControllerData
{
    // btn data:
    // L = left, R = right, p = primary, s = secondary, Btn = button

    // primary:

    double pLXStick = 0;
    double pLYStick = 0;
    double pRXStick = 0;
    double pRYStick = 0;

    bool pLShoulderSwitch = false;
    bool pRShoulderSwitch = false;

    // secondary:

    double sLXStick = 0;
    double sLYStick = 0;
    double sRXStick = 0;
    double sRYStick = 0;

    bool sLStickBtn = false;
    bool sRStickBtn = false;

    double sLTrigger = 0;
    double sRTrigger = 0;
    bool sLBumper = false;
    bool sLBumperToggled = false;
    bool sRBumper = false;
    bool sRBumperToggled = false;

    bool sXBtn = false;
    bool sYBtn = false;
    bool sABtn = false;
    bool sBBtn = false;

    bool sABtnToggled = false;
    bool sBBtnToggled = false;
    bool sXBtnToggled = false;
    bool sYBtnToggled = false;

    bool sLCenterBtn = false;
    bool sRCenterBtn = false;

    bool sLCenterBtnToggled = false;
    bool sRCenterBtnToggled = false;

    bool sLTriggerToggled = false;
    bool sRTriggerToggled = false;

    int sDPad = -1;
};

class Controller
{

    public:
        void TeleopPeriodic(const RobotData &robotData, ControllerData &controllerData, ControlData &controlData);

    private:
        /**
         * Don't touch "Controller.cpp" that is for the direct access to joystick buttons
         * when writing code and assigning it to specific button or axis, 
         * write to it through "ControlData.cpp"
         * */

        void updateBtnData(ControllerData &controllerData);
        void updateControlData(const RobotData &robotData, const ControllerData &controllerData, ControlData &controlData);

        // basic btn getters:
        bool getBtn(int js, int index);
        bool getBtnToggled(int js, int index);
        int getPOV(int js, int index);
        double getAxis(int js, int index);

        frc::Joystick primary{0};
        frc::Joystick secondary{1};

};

