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
#include <frc/DutyCycleEncoder.h>

#include <frc/shuffleboard/Shuffleboard.h>

struct RobotData;

struct IntakeData
{
    bool intakeIdle;
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
    double absoluteToREV(double value);
    bool intakeIdle(const RobotData &robotData, IntakeData &intakeData);


    double intakePivotSpeed = 0.1;
    double intakeRollerSpeed = 0.67;
    double intakeMecanumSpeed = 0.2;
    double intakeRollersEjectSpeed = 0.5;
    double armDownPosition = 0.428;
    int tickCount = 0;
    bool zeroedIntake = true;

    int idleCount = 0;


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

    frc::DutyCycleEncoder intakePivotEncoder2 = frc::DutyCycleEncoder{intakeAbsoluteEncoderPort};

};