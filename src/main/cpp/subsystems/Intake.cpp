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

    intakePivot.SetSmartCurrentLimit(15);
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

    if(intakeRollers.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakeRollers.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakeRollers.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        rollersInit();
    }
    if(intakePivot.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakePivot.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakePivot.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        pivotInit();
    }

    // add fault case for singulator?

}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData){
    // if(intakePivotEncoder.GetPosition() > 0.5){
    //     zeroedIntake = false;
    // }

    if(intakePivotEncoder2.GetOutput() > 0.03){ //checks to see if the encoder is reading zero because if it is that means the encoder was most likley unplugged and the current values are wrong and we don't want to run any motors
        //constantly updates the intake rev encoder based on the absolute encoder values 
        if(tickCount > 45){
            intakePivotEncoder.SetPosition(absoluteToREV(intakePivotEncoder2.GetOutput()));
            tickCount = (tickCount+1)%50;
        }else{
            tickCount = (tickCount+1)%50;
        }

    }else{
        // intakeRollers.Set(0);
        // intakeSingulator.Set(0);
        // intakePivot.Set(0);
    }  

    if (robotData.controlData.saIntake) //you are intaking
    {
        // pivot down
        intakePivot_pidController.SetReference(15, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

        //run rollers, singulator
        intakeRollers.Set(intakeRollerSpeed);
        intakeSingulator.Set(singulatorSpeed);
        
    }
    //intake down and rollers backwards
    else if (robotData.controlData.saIntakeBackward)
    {
        intakeRollers.Set(-intakeRollerSpeed);
        intakePivot_pidController.SetReference(15, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else if (robotData.controlData.saEjectBalls) //rollers backwards, pivot down
    {
        intakeRollers.Set(-intakeRollersEjectSpeed);
        intakeSingulator.Set(-singulatorSpeed);
        intakePivot_pidController.SetReference(15, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else //default case, everything up and not running
    {
        if(!intakeData.intakeIdle){ // run the singulator while the intake is not idle
            intakeSingulator.Set(singulatorSpeed);
        }else{
            intakeSingulator.Set(0);
        }

        intakePivot_pidController.SetReference(0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
        intakeRollers.Set(0);
    }
}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData){
    //intake extended
    // intakePivot.Set(robotData.controllerData.sLYStick);
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
    intakeData.intakeIdle = intakeIdle(robotData, intakeData);
    frc::SmartDashboard::PutNumber("Pivot built in Pos", intakePivotEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Pivot absolute Pos", intakePivotEncoder2.GetOutput());
    frc::SmartDashboard::PutNumber("Changed pos", absoluteToREV(intakePivotEncoder2.GetOutput()));

    frc::SmartDashboard::PutBoolean("idle?", intakeData.intakeIdle);
    frc::SmartDashboard::PutNumber("idle count", idleCount);

    frc::SmartDashboard::PutBoolean("shift", robotData.controlData.shift);
    frc::SmartDashboard::PutNumber("mode", robotData.controlData.mode);
    
    frc::SmartDashboard::PutBoolean("upper hub shot", robotData.controlData.upperHubShot);

}

bool Intake::intakeIdle(const RobotData &robotData, IntakeData &intakeData){


    if (robotData.controlData.saIntake || robotData.controlData.saIntakeBackward){
        // if something is commanding the intake then idle count is 25
        idleCount = 25;
        return false; // hasn't 
    } else if(idleCount > 0){ // nothing is commanding the intake, and the idle count is counting down
        idleCount--;
        return false;
    } else { // noothing has been commanding the intake for 25 ticks, intake is considered idle
        return true;
    }

}

double Intake::absoluteToREV(double value){
    double slope = (revOut - revIn)/(absOut - absIn);
    double b = revIn - (slope*absIn);
    return ((value*slope) + b);
    //return (value*-121+74.8);
}