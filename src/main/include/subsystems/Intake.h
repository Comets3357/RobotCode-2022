#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/Preferences.h>

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

    void rollersInit();
    void pivotInit();
    void mecanumInit();


    const double intakePivotSpeed = 0.2;
    const double intakeRollerSpeed = 0.2;
    const double intakeMecanumSpeed = 0.2;

    double armDownPosition = frc::Preferences::GetDouble("ArmDownPosition", 12.0);

    //CHANGE MOTOr ID STUFF  (just outline )
    rev::CANSparkMax intakeRollers = rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeRollersEncoder = intakeRollers.GetEncoder();
    rev::SparkMaxPIDController intakeRollers_pidController = intakeRollers.GetPIDController();

    rev::CANSparkMax intakePivot = rev::CANSparkMax(intakePivotID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakePivotEncoder = intakePivot.GetEncoder();
    rev::SparkMaxPIDController intakePivot_pidController = intakePivot.GetPIDController();

    rev::CANSparkMax intakeMecanum = rev::CANSparkMax(intakeMecanumID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeMecanumEncoder = intakeMecanum.GetEncoder();
    rev::SparkMaxPIDController intakeMecanum_pidController = intakeMecanum.GetPIDController();

};