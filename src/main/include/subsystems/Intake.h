#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
// sparkmax??



struct RobotData;

struct IntakeData
{

};

class Intake
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, IntakeData &intakeData);
    void DisabledInit();

private:
    void updateData(const RobotData &robotData, IntakeData &intakeData);

    void manual(const RobotData &robotData, IntakeData &intakeData);
    void semiAuto(const RobotData &robotData, IntakeData &intakeData);

    //CHANGE MOTOr ID STUFF  (just outline )
    rev::CANSparkMax intakeRollers = rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeRollersEncoder = intakeRollers.GetEncoder();
    rev::SparkMaxPIDController intakeRollers_pidController = intakeRollers.GetPIDController();



};