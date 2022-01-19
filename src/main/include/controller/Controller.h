#pragma once

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

struct RobotData;

struct ControlData
{
    // states:
    bool manualMode = false;
    bool climbMode = false;
    bool shift = false;

    // drivebase:
    double lDrive;
    double rDrive;
    bool turnResponsive;
    bool dbInverted;
    double maxStraight = 1;
    double maxTurn = 0.4;

    //intake:
    double mIntakeDown; 
    bool mIntakeUp; //brings the intake down (default is up) MANUAL

    //brings the intake down (default is up) MANUAL
    double mIntakeRollers; //runs intake forwards MANUAL
    double mzeroing; //runs intake rollers backwards MANUAL
    bool saIntake; //runs the intake rollers and brings intake down and the indexer to intake balls SEMIAUTO
    bool saIntakeBackward; //runs the intake backwards SEMIAUTO

    //indexer:
    bool mIndexerBackwards; //runs indexer backwards MANUAL
    bool mIndexer; //runs indexer foward MANUAL
    bool saEjectBalls; //runs intake and indexer backwards to eject balls SEMIAUTO

    //shooter:
    bool saShooting; //gets hood at right angle, shooter wheel up to speed SEMIAUTO
    bool finalShoot; //makes belts run to actually fire balls SEMIAUTO
    bool launchPadShot; //fixed long shot from the launch pad to upper hub SEMIAUTO
    bool hubShot;//fixed close shot to lower hub from infront of hub SEMIAUTO
    bool wrongBall; //if the ball isn't our alliance color eject ball out shooter SEMIAUTO 
    bool mFlyWheel; //get flywheel running MANUAL
    double mHood; //moves hood up or down MANUAL

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
    bool sRBumper = false;

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
    void updateControlData(const ControllerData &controllerData, ControlData &controlData);

    // basic btn getters:
    bool getBtn(int js, int index);
    bool getBtnToggled(int js, int index);
    int getPOV(int js, int index);
    double getAxis(int js, int index);

    frc::Joystick primary{0};
    frc::Joystick secondary{1};
};

