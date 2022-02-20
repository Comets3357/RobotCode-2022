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

#include <frc/shuffleboard/Shuffleboard.h>

struct RobotData;

struct IntakeData
{
    bool intakeIdle;
    bool absEncoderInRange = true;
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
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, IntakeData &intakeData);
    void updateData(const RobotData &robotData, IntakeData &intakeData);


private:
    void manual(const RobotData &robotData, IntakeData &intakeData);
    void semiAuto(const RobotData &robotData, IntakeData &intakeData);

    void rollersInit();
    void pivotInit();
    void singulatorInit();
    void mecanumInit();
    double absoluteToREV(double value);
    bool intakeIdle(const RobotData &robotData, IntakeData &intakeData);

    //bench test
    bool encoderPluggedIn(const IntakeData &intakeData);
    bool encoderInRange(const IntakeData &intakeData);
    void checkDeadStop(IntakeData &intakeData);

    double intakePivotSpeed = 0.1;
    double intakeRollerSpeed = 0.9;
    double intakeMecanumSpeed = 0.2;
    double singulatorSpeed = -0.6;
    double intakeRollersEjectSpeed = 0.5;
    double armDownPosition = 0.428;
    int tickCount = 0;
    bool zeroedIntake = true;

    int idleCount = 0;


    //CHANGE MOTOr ID STUFF  (just outline )
    rev::CANSparkMax intakeRollers = rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeRollersEncoder = intakeRollers.GetEncoder();

    rev::CANSparkMax intakePivot = rev::CANSparkMax(intakePivotID, rev::CANSparkMax::MotorType::kBrushless);
    //THIS IS THE REV ENCODER
    rev::SparkMaxRelativeEncoder intakePivotEncoderRev = intakePivot.GetEncoder();
    rev::SparkMaxPIDController intakePivot_pidController = intakePivot.GetPIDController();

    rev::CANSparkMax intakeSingulator = rev::CANSparkMax(intakeSingulatorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeSingulatorEncoder = intakeSingulator.GetEncoder();

    frc::DigitalInput m_input{intakeAbsoluteEncoderPort};
    //THIS IS THE ABSOLUTE ENCODER
    frc::DutyCycle intakePivotEncoderAbs = frc::DutyCycle{m_input};

};