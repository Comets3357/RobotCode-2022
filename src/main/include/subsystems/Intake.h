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
#include <frc/DutyCycle.h>
#include <frc/DigitalSource.h>

struct RobotData;

struct IntakeData
{
    bool intakeIdle;
    bool absEncoderInRange = true;

    //bench test
    float intakeBenchTestSpeed = 0;
    float benchTestIntakePivotSpeed = 0;
    float benchTestIntakeRollersSpeed = 0;
    float benchTestSingulatorSpeed = 0;
    bool topDeadStop = false;
    bool bottomDeadStop = false;
};

class Intake
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, IntakeData &intakeData);
    void TestPeriodic(const RobotData &robotData, IntakeData &intakeData);
    void TestInit();
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, IntakeData &intakeData);
    void updateData(const RobotData &robotData, IntakeData &intakeData);

private:
    void manual(const RobotData &robotData, IntakeData &intakeData);
    void semiAuto(const RobotData &robotData, IntakeData &intakeData);

    //inits
    void rollersInit();
    void pivotInit();
    void singulatorInit();

    //converting
    double absoluteToREV(double value);

    bool intakeIdle(const RobotData &robotData, IntakeData &intakeData);

    //bench test
    bool encoderPluggedIn();
    bool encoderInRange();
    void checkDeadStop(IntakeData &intakeData);

    //speeds
    double intakePivotSpeed = 0.1;
    double intakeRollerSpeed = 0.9;
    double intakesingulatorSpeed = -0.6;
    double intakeRollersEjectSpeed = 0.5;

    int idleCount = 0;
    //used to update rev encoder with abs encoder
    int tickCount = 0;


    //rollers
    rev::CANSparkMax intakeRollers = rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeRollersEncoder = intakeRollers.GetEncoder();

    //pivot
    rev::CANSparkMax intakePivot = rev::CANSparkMax(intakePivotID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakePivotEncoderRev = intakePivot.GetEncoder();
    rev::SparkMaxPIDController intakePivot_pidController = intakePivot.GetPIDController();

    //singulator
    rev::CANSparkMax intakeSingulator = rev::CANSparkMax(intakeSingulatorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeSingulatorEncoder = intakeSingulator.GetEncoder();

    //abs pivot encoder
    frc::DigitalInput m_input{intakeAbsoluteEncoderPort};
    //THIS IS THE ABSOLUTE ENCODER
    frc::DutyCycle intakePivotEncoderAbs = frc::DutyCycle{m_input};

};