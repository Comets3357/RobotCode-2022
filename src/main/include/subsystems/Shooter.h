#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>


struct RobotData;

struct ShooterData
{

};

class Shooter
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
    void DisabledInit();

private:
    void updateData(const RobotData &robotData, ShooterData &shooterData);

    void manual(const RobotData &robotData, ShooterData &shooterData);
    void semiAuto(const RobotData &robotData, ShooterData &shooterData);

    //CHANGE MOTOr ID STUFF  (just outline lol don't take your life too seriously:))
    rev::CANSparkMax shooterWheel = rev::CANSparkMax(31, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder shooterWheelEncoder = shooterWheel.GetEncoder();
    rev::SparkMaxPIDController shooterWheel_pidController = shooterWheel.GetPIDController();



};