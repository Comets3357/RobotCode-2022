//Tells the compiler to not include the header files multiple times
#pragma once

// //Header files from FRC
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>

//Header files from us
#include "controller/Controller.h"
#include "RobotData.h"

#include "subsystems/Drivebase.h"
#include "common/LEDs.h"

//Robot class inherits from TimedRobot
class Robot : public frc::TimedRobot
{

public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;

private:
    RobotData robotData{};

    // other
    Controller controller{};
    Gyro gyro{};
    Limelight limelight{};
    OtherComponents otherComponents{};
    Timer timer{};
    LEDs LED{};
    

    // subsystems
    Drivebase drivebase{};
};