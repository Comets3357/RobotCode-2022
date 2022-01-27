#include "RobotData.h"
#include <iostream>
#include <cmath>

void Intake::RobotInit()
{
    Intake::pivotInit();
    Intake::rollersInit();
    Intake::mecanumInit();

    intakePivotEncoder.SetPosition(0);
    intakeRollersEncoder.SetPosition(0);
    intakeMecanumEncoder.SetPosition(0);

    //intakePivotEncoder2.Reset();

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

    if(tickCount > 40){
        intakePivotEncoder.SetPosition(absoluteToREV(intakePivotEncoder2.GetDistance()));
        tickCount = (tickCount+1)%50;
    }else{
        tickCount = (tickCount+1)%50;
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
        intakePivot_pidController.SetReference(10, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

        intakeRollers.Set(intakeRollerSpeed);
        intakeMecanum.Set(intakeMecanumSpeed);
        
    }
    else if (robotData.controlData.saIntakeBackward)
    {
        intakeRollers.Set(-intakeRollerSpeed);
        //intakeMecanum.Set(-intakeMecanumSpeed);
    }
    else if (robotData.controlData.saEjectBalls) //rollers backwards, pivot down
    {
        intakeRollers.Set(-intakeRollersEjectSpeed);
        intakePivot_pidController.SetReference(10, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else //default case, everything up and not running
    {
        intakeRollers.Set(0);
        intakeMecanum.Set(0);

        intakePivot_pidController.SetReference(0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition, 1);
        //intakePivotEncoder.SetPosition(0);

    }
}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData){
    // if(robotData.controlData.mIntakeDown){
    //     intakePivot_pidController.SetReference(12, rev::ControlType::kPosition,0);
    
    // }else{
    //     intakePivot_pidController.SetReference(0, rev::ControlType::kPosition, 1);

    // }

    //intakeRollers.Set(robotData.controlData.mIntakeRollersBackward*.55);
    if(robotData.controlData.mIntakeRollers){
        intakeRollers.Set(intakeRollerSpeed);
        //intakeRollers.Set(rollerSpeed.GetDouble(0.05));
    }else{
        intakeRollers.Set(0);
    }

    intakePivot.Set(robotData.controlData.mIntakeDown*0.2);
    
    // if(robotData.controlData.mzeroing){
    //     intakePivotEncoder.SetPosition(0);
    // }

    // if (robotData.controlData.mIntakeRollers)
    // {
    //     // intakeRollers.Set(intakeRollerSpeed);
    //     // intakeMecanum.Set(intakeMecanumSpeed);
    //     intakeRollers.Set(robotData.controlData.mIntakeRollers*.55);
    // }
    // else if (robotData.controlData.mIntakeRollersBackward)
    // {
    //     //intakeRollers.Set(-intakeRollerSpeed);
    //     //intakeMecanum.Set(-intakeMecanumSpeed);
    // }
    // else
    // {
    //     intakeRollers.Set(0);
    //     intakeMecanum.Set(0);
    // }
}

void Intake::DisabledInit()
{
    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    intakeRollers.Set(0);
    intakeMecanum.Set(0);

    intakePivot.Set(0);
}

// updates encoder and gyro values
void Intake::updateData(const RobotData &robotData, IntakeData &intakeData)
{
    frc::SmartDashboard::PutNumber("Pivot built in Pos", intakePivotEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Pivot absolute Pos", intakePivotEncoder2.GetDistance());
    frc::SmartDashboard::PutNumber("Changed pos", absoluteToREV(intakePivotEncoder2.GetDistance()));
    frc::SmartDashboard::PutNumber("pivot speed", intakePivot.Get());

    frc::SmartDashboard::PutNumber("roller speed", intakeRollers.Get());
    frc::SmartDashboard::PutBoolean("manual", robotData.controlData.manualMode);

    frc::SmartDashboard::PutNumber("tick count", tickCount);

    intakeData.intakeIdle = intakeIdle(robotData, intakeData);
    frc::SmartDashboard::PutBoolean("idle?", intakeData.intakeIdle);
    frc::SmartDashboard::PutNumber("idle count", idleCount);


}

bool Intake::intakeIdle(const RobotData &robotData, IntakeData &intakeData){

    if (robotData.controlData.saIntake || robotData.controlData.saIntakeBackward){
        // if nothing is commanding the intake then idle counnt is 25
        idleCount = 25;
        return false; // hasn't 
    } else if(idleCount > 0){ // the intake is idling
        idleCount--;
        return false;
    } else {
        return true;
    }

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

    //down
    intakePivot_pidController.SetP(0.74,0);
    intakePivot_pidController.SetI(0,0);
    intakePivot_pidController.SetD(0.2,0);
    intakePivot_pidController.SetIZone(0,0);
    intakePivot_pidController.SetFF(0,0);
    intakePivot_pidController.SetOutputRange(-0.205, 0.16,0);

    //up
    intakePivot_pidController.SetP(0.99,1);
    intakePivot_pidController.SetI(0,1);
    intakePivot_pidController.SetD(0.3,1);
    intakePivot_pidController.SetIZone(0,1);
    intakePivot_pidController.SetFF(0,1);
    intakePivot_pidController.SetOutputRange(-.2, 0.15,1);

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

double Intake::absoluteToREV(double value){
    return (value*-71.1 -4.83);
}



