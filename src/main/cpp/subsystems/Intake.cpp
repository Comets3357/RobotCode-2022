#include "subsystems/Intake.h"
#include "RobotData.h"

void Intake::RobotInit()
{
    Intake::pivotInit();
    Intake::rollersInit();
    Intake::mecanumInit();

    intakePivotEncoder.SetPosition(0);
    intakeRollersEncoder.SetPosition(0);
    intakeMecanumEncoder.SetPosition(0);

    intakePivot.Set(0);
    intakeRollers.Set(0);
    intakeMecanum.Set(0);
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

    if(intakeRollers.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakeRollers.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakeRollers.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        Intake::rollersInit();
    }
    if(intakeMecanum.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakeMecanum.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakeMecanum.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        Intake::mecanumInit();
    }
    if(intakePivot.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakePivot.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakePivot.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        Intake::pivotInit();
    }

}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData){
    if (robotData.controlData.saIntake) //you are intaking
    {
        // pivot down
        if (intakePivotEncoder.GetPosition() < armDownPosition)
        {
            intakePivot.Set(intakePivotSpeed);
            intakeRollers.Set(intakeRollerSpeed);
            intakeMecanum.Set(intakeMecanumSpeed);
        }
        // once you're down
        else
        {
            intakePivot.Set(0);
        }
    }
    else if (robotData.controlData.saIntakeBackward)
    {
        intakeRollers.Set(-intakeRollerSpeed);
        //intakeMecanum.Set(-intakeMecanumSpeed);
    }
    else if (robotData.controlData.saEjectBalls) //rollers backwards, pivot down
    {
        intakeRollers.Set(-intakeRollersEjectSpeed);
        if (intakePivotEncoder.GetPosition() < armDownPosition)
        {
            intakePivot.Set(intakePivotSpeed);
        }
        // once you're down
        else
        {
            intakePivot.Set(0);
        }
    }
    else //default case, everything up and not running
    {
        intakeRollers.Set(0);
        intakeMecanum.Set(0);

        if (intakePivotEncoder.GetPosition() > 0)
        {
            intakePivot.Set(-intakePivotSpeed);
        }
        else
        {
            intakePivot.Set(0);
        }
    }
}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData){
    if(robotData.controlData.mIntakeDown){
        if (intakePivotEncoder.GetPosition() < armDownPosition)
        {
            intakePivot.Set(intakePivotSpeed);
        }
        // once you're down
        else
        {
            intakePivot.Set(0);
        }
    }else{
         if (intakePivotEncoder.GetPosition() > 0)
        {
            intakePivot.Set(-intakePivotSpeed);
        }
        else
        {
            intakePivot.Set(0);
        }
    }

    if (robotData.controlData.mIntakeRollers)
    {
        intakeRollers.Set(intakeRollerSpeed);
        intakeMecanum.Set(intakeMecanumSpeed);

    }
    else if (robotData.controlData.mIntakeRollersBackward)
    {
        intakeRollers.Set(-intakeRollerSpeed);
        //intakeMecanum.Set(-intakeMecanumSpeed);

    }
    else
    {
        intakeRollers.Set(0);
        intakeMecanum.Set(0);

    }
}

void Intake::DisabledInit()
{
    intakeRollers.Set(0);
    intakeMecanum.Set(0);
    intakePivot.Set(0);
}

// updates encoder and gyro values
void Intake::updateData(const RobotData &robotData, IntakeData &intakeData)
{
    
}

void Intake::rollersInit(){
    intakeRollers.RestoreFactoryDefaults();

    intakeRollers.SetInverted(true);

    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // intakeRollers_pidController.SetP(pkP);
    // intakeRollers_pidController.SetI(pkI);
    // intakeRollers_pidController.SetD(pkD);
    // intakeRollers_pidController.SetIZone(pkIz);
    // intakeRollers_pidController.SetFF(pkFF);
    // intakeRollers_pidController.SetOutputRange(pkMinOutput, pkMaxOutput);

    intakeRollers.SetSmartCurrentLimit(45);

}

void Intake::pivotInit(){
    intakePivot.RestoreFactoryDefaults();

    intakePivot.SetInverted(false);

    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // intakePivot_pidController.SetP(wkP);
    // intakePivot_pidController.SetI(wkI);
    // intakePivot_pidController.SetD(wkD);
    // intakePivot_pidController.SetIZone(wkIz);
    // intakePivot_pidController.SetFF(wkFF);
    // intakePivot_pidController.SetOutputRange(wkMinOutput, wkMaxOutput);

    intakePivot.SetSmartCurrentLimit(45);
}

void Intake::mecanumInit(){
    intakeMecanum.RestoreFactoryDefaults();

    intakeMecanum.SetInverted(false);

    intakeMecanum.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // intakeMecanum_pidController.SetP(mkP);
    // intakeMecanum_pidController.SetI(mkI);
    // intakeMecanum_pidController.SetD(mkD);
    // intakeMecanum_pidController.SetIZone(mkIz);
    // intakeMecanum_pidController.SetFF(mkFF);
    // intakeMecanum_pidController.SetOutputRange(mkMinOutput, mkMaxOutput);

    intakeMecanum.SetSmartCurrentLimit(45);

}



