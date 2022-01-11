#include "subsystems/Intake.h"
#include "RobotData.h"

void Intake::RobotInit()
{
    intakeRollers.RestoreFactoryDefaults();
    intakePivot.RestoreFactoryDefaults();

    intakeRollers.SetInverted(true);
    intakePivot.SetInverted(false);

    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    intakeRollers_pidController.SetP(wkP);
    intakeRollers_pidController.SetI(wkI);
    intakeRollers_pidController.SetD(wkD);
    intakeRollers_pidController.SetIZone(wkIz);
    intakeRollers_pidController.SetFF(wkFF);
    intakeRollers_pidController.SetOutputRange(wkMinOutput, wkMaxOutput);

    intakePivot_pidController.SetP(wkP);
    intakePivot_pidController.SetI(wkI);
    intakePivot_pidController.SetD(wkD);
    intakePivot_pidController.SetIZone(wkIz);
    intakePivot_pidController.SetFF(wkFF);
    intakePivot_pidController.SetOutputRange(wkMinOutput, wkMaxOutput);

    intakeRollers.SetSmartCurrentLimit(45);
    intakePivot.SetSmartCurrentLimit(45);

    intakeRollersEncoder.SetPosition(0);
    intakePivotEncoder.SetPosition(0);
   
}

void Intake::RobotPeriodic(const RobotData &robotData, IntakeData &intakeData)
{
    updateData(robotData, intakeData);
    if (robotData.controlData.manualMode)
    {
        manual(robotData, intakeData);
    }
    else
    {
        semiAuto(robotData, intakeData);
    }

}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData){

}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData){
    
}

void Intake::DisabledInit()
{
    
}

// updates encoder and gyro values
void Intake::updateData(const RobotData &robotData, IntakeData &intakeData)
{
    
}

