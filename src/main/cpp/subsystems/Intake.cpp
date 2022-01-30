#include "RobotData.h"
#include <iostream>
#include <cmath>

void Intake::RobotInit()
{
    pivotInit();
    rollersInit();
    singulatorInit();
                
    intakePivotEncoder.SetPosition(0);
    intakeRollersEncoder.SetPosition(0);
    intakeSingulatorEncoder.SetPosition(0);

    intakePivot.Set(0);
    intakeRollers.Set(0);
    intakeSingulator.Set(0);
}

void Intake::rollersInit(){
    intakeRollers.RestoreFactoryDefaults();
    intakeRollers.SetInverted(true);
    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    intakeRollers.SetSmartCurrentLimit(45);
}

void Intake::pivotInit(){
    intakePivot.RestoreFactoryDefaults();
    intakePivot.SetInverted(false);
    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //down
    intakePivot_pidController.SetP(0.2,0);
    intakePivot_pidController.SetI(0,0);
    intakePivot_pidController.SetD(0,0);
    intakePivot_pidController.SetIZone(0,0);
    intakePivot_pidController.SetFF(0,0);
    intakePivot_pidController.SetOutputRange(-0.5, 0.5,0);

    intakePivot.SetSmartCurrentLimit(45);
}

void Intake::singulatorInit(){
    intakeSingulator.RestoreFactoryDefaults();
    intakeSingulator.SetInverted(true);
    intakeSingulator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    intakeSingulator.SetSmartCurrentLimit(15);
}

void Intake::RobotPeriodic(const RobotData &robotData, IntakeData &intakeData)
{
    updateData(robotData, intakeData);

    if (robotData.controlData.mode == mode_teleop_manual)
    {
        if(!zeroedIntake){
            intakePivot_pidController.SetReference(0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition, 1);
            if(intakePivotEncoder.GetPosition() < 0.5){
                zeroedIntake = true;
            }
        }else{
            manual(robotData, intakeData);

        }

    }
    else if (robotData.controlData.mode == mode_teleop_sa)
    {
        semiAuto(robotData, intakeData);
    }

    if(tickCount > 45){
        intakePivotEncoder.SetPosition(absoluteToREV(intakePivotEncoder2.GetOutput()));
        tickCount = (tickCount+1)%50;
    }else{
        tickCount = (tickCount+1)%50;
    }

    if(intakeRollers.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakeRollers.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakeRollers.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        rollersInit();
    }
    if(intakePivot.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakePivot.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakePivot.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        pivotInit();
    }

}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData){
    if(intakePivotEncoder.GetPosition() > 0.5){
        zeroedIntake = false;
    }

    if (robotData.controlData.saIntake) //you are intaking
    {
        // pivot down
        intakePivot_pidController.SetReference(14, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

        //run rollers
        intakeRollers.Set(intakeRollerSpeed);
        intakeSingulator.Set(singulatorSpeed);
        
    }
    //intake down and rollers backwards
    else if (robotData.controlData.saIntakeBackward)
    {
        intakeRollers.Set(-intakeRollerSpeed);
        intakePivot_pidController.SetReference(14, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else if (robotData.controlData.saEjectBalls) //rollers backwards, pivot down
    {
        intakeRollers.Set(-intakeRollersEjectSpeed);
        intakeSingulator.Set(-singulatorSpeed);
        intakePivot_pidController.SetReference(14, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else //default case, everything up and not running
    {
        if(!intakeIdle(robotData, intakeData)){
            intakeSingulator.Set(singulatorSpeed);
        }else{
            intakeSingulator.Set(0);
        }
        intakeRollers.Set(0);

        intakePivot_pidController.SetReference(0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
    }
}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData){
    //intake extended
    if(robotData.controlData.mIntakeUp){
        intakePivot.Set(-intakePivotSpeed);

    //intake retracted
    }else if(robotData.controlData.mIntakeDown){
        intakePivot.Set(intakePivotSpeed);

    //not intaking
    }else{
        intakePivot.Set(0);
    }

    //intake rollers running inward
    if(robotData.controlData.mIntakeRollersIn){
        intakeRollers.Set(intakeRollerSpeed);
    
    //intake rollers running outward
    }else if(robotData.controlData.mIntakeRollersOut){
        intakeRollers.Set(-intakeRollerSpeed);
    
    //no intake rollers running
    }else{
        intakeRollers.Set(0);
    }
    
    if(robotData.controlData.mZeroHood){
        intakePivotEncoder.SetPosition(0);
    }

}

void Intake::DisabledInit()
{
    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    intakeRollers.Set(0);
    intakePivot.Set(0);
    intakeSingulator.Set(0);
}

// updates encoder and gyro values
void Intake::updateData(const RobotData &robotData, IntakeData &intakeData)
{
    frc::SmartDashboard::PutNumber("Pivot built in Pos", intakePivotEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Pivot absolute Pos", intakePivotEncoder2.GetOutput());
    frc::SmartDashboard::PutNumber("Changed pos", absoluteToREV(intakePivotEncoder2.GetOutput()));


    intakeData.intakeIdle = intakeIdle(robotData, intakeData);
    frc::SmartDashboard::PutBoolean("idle?", intakeData.intakeIdle);
    frc::SmartDashboard::PutNumber("idle count", idleCount);
    frc::SmartDashboard::PutBoolean("shift", robotData.controlData.shift);

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

double Intake::absoluteToREV(double value){
    // double slope = (revOut - revIn)/(absOut - absIn);
    // double b = revIn - (slope*absIn);
    // return ((value*slope) - b);
    return (value*-103+96.3);
}



