#include "RobotData.h"
#include <iostream>
#include <cmath>

void Intake::RobotInit()
{
    pivotInit();
    rollersInit();
    singulatorInit();
                
    intakePivotEncoderRev.SetPosition(0);
    intakeRollersEncoder.SetPosition(0);
    intakeSingulatorEncoder.SetPosition(0);

    intakePivot.Set(0);
    intakeRollers.Set(0);
    intakeSingulator.Set(0);
}

void Intake::rollersInit(){
    intakeRollers.RestoreFactoryDefaults();
    intakeRollers.SetInverted(true);
    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    intakeRollers.SetSmartCurrentLimit(45);
}

void Intake::pivotInit(){
    intakePivot.RestoreFactoryDefaults();
    intakePivot.SetInverted(false);
    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //down
    intakePivot_pidController.SetP(0.23);
    intakePivot_pidController.SetI(0);
    intakePivot_pidController.SetD(0.01);
    intakePivot_pidController.SetIZone(0);
    intakePivot_pidController.SetFF(0);
    intakePivot_pidController.SetOutputRange(-0.3, 0.2);

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

    if(robotData.controlData.mode == mode_climb_manual || robotData.controlData.mode == mode_climb_sa){
        intakePivot_pidController.SetReference(0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
        intakeRollers.Set(0);
        intakeSingulator.Set(0);

    }else{
        if (robotData.controlData.mode == mode_teleop_manual)
        {
            //checks to see if the intake is down when switched to manual mode, and if it is bring it up before manual functionality 
            // if(!zeroedIntake){
            //     intakePivot_pidController.SetReference(0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
            //     if(intakePivotEncoderRev.GetPosition() < 0.3){
            //         zeroedIntake = true;
            //     }
            // }else{ 

            // }
            manual(robotData, intakeData);

            
        }
        else if (robotData.controlData.mode == mode_teleop_sa)
        {
            semiAuto(robotData, intakeData);
        }
    }
    

    //if anything is broken or not working, reset the motor and it's init functions
    if(intakeRollers.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakeRollers.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakeRollers.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        rollersInit();
    }
    if(intakePivot.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakePivot.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakePivot.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
        pivotInit();
    }

    // add fault case for singulator?

}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData){
    //used to check if the intake is up or down
    if(intakePivotEncoderRev.GetPosition() > 0.5){
        zeroedIntake = false;
    }

    encoderPluggedIn(intakeData);

//INTAKE FUNCTIONALITY
    if (robotData.controlData.saIntake) //you are intaking
    {
        // pivot down
        intakePivot_pidController.SetReference(revOut - 0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

        //run rollers, singulator
        intakeRollers.Set(intakeRollerSpeed);
        intakeSingulator.Set(singulatorSpeed);
        
    }
    //intake down and rollers backwards
    else if (robotData.controlData.saIntakeBackward)
    {
        intakeRollers.Set(-intakeRollerSpeed);
        intakePivot_pidController.SetReference(revOut - 0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else if (robotData.controlData.saEjectBalls) //rollers backwards, pivot down
    {
        intakeRollers.Set(-intakeRollersEjectSpeed);
        intakeSingulator.Set(-singulatorSpeed);
        intakePivot_pidController.SetReference(revOut - 0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else //default case, everything up and not running
    {
        if(!intakeData.intakeIdle){ // run the singulator while the intake is not idle (basically run it for a second after the intake stops)
            intakeSingulator.Set(singulatorSpeed);
        }else{
            intakeSingulator.Set(0);
        }

        //encoderPluggedIn(intakeData); 
        if(intakePivotEncoderAbs.GetOutput() == absIn){
            intakePivotEncoderRev.SetPosition(0);
        }

        //bring up the intake
        intakePivot_pidController.SetReference(0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
        intakeRollers.Set(0);
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

     if(robotData.controlData.mZeroHood)
    {
        intakePivotEncoderRev.SetPosition(0);
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
    frc::SmartDashboard::PutNumber("Pivot built in Pos", intakePivotEncoderRev.GetPosition());
    frc::SmartDashboard::PutNumber("Pivot absolute Pos", intakePivotEncoderAbs.GetOutput());
    //frc::SmartDashboard::PutNumber("Changed pos", absoluteToREV(intakePivotEncoder2.GetOutput()));

    //frc::SmartDashboard::PutBoolean("idle?", intakeData.intakeIdle);
    //frc::SmartDashboard::PutNumber("idle count", idleCount);

    // frc::SmartDashboard::PutNumber("mode", robotData.controlData.mode);
    
}

bool Intake::intakeIdle(const RobotData &robotData, IntakeData &intakeData){
    if (robotData.controlData.saIntake || robotData.controlData.saIntakeBackward){
        // if something is commanding the intake then idle count is 25
        idleCount = 50;
        return false; // hasn't 
    } else if(idleCount > 0){ // nothing is commanding the intake, and the idle count is counting down
        idleCount--;
        return false;
    } else { // noothing has been commanding the intake for 25 ticks, intake is considered idle
        return true;
    }

}

//converts the intake from absolute values to values the rev can read
double Intake::absoluteToREV(double value){
    double slope = (revOut - revIn)/(absOut - absIn);
    double b = revIn - (slope*absIn);
    return ((value*slope) + b);
}

//checks to see if the encoder is reading zero because if it is that means the encoder was most likley unplugged and the current values are wrong and we don't want to run any motors
void Intake::encoderPluggedIn(const IntakeData &intakeData){
    if (intakePivotEncoderAbs.GetOutput() > 0.03){
        //constantly updates the intake rev encoder based on the absolute encoder values 
        if (tickCount > 30){
            intakePivotEncoderRev.SetPosition(absoluteToREV(intakePivotEncoderAbs.GetOutput()));
            tickCount = (tickCount + 1) % 40;
        } else {
            tickCount = (tickCount + 1) % 40;
        }

    } else {
    }
}