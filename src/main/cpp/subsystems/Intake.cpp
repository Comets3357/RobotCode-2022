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
<<<<<<< Updated upstream
=======

    intakePivotEncoder2.Reset();

    // intakePivot.Set(0);
    // intakeRollers.Set(0);
    // intakeMecanum.Set(0);
>>>>>>> Stashed changes
}

void Intake::RobotPeriodic(const RobotData &robotData, IntakeData &intakeData)
{
    updateData(robotData, intakeData);
    if (robotData.controlData.manualMode)
    {
        semiAuto(robotData, intakeData);

    }
    else
    {
        manual(robotData, intakeData);

    }

    // if(x % 5 == 0){
    //     intakePivotEncoder.SetPosition(((intakePivotEncoder2.GetDistance()*(-98.48485))-55.15152));
    //     x++;
    // }else{
    //     x++;
    // }

    // if(intakeRollers.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakeRollers.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakeRollers.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
    //     Intake::rollersInit();
    // }
    // if(intakeMecanum.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakeMecanum.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakeMecanum.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
    //     Intake::mecanumInit();
    // }
    // if(intakePivot.GetFault(rev::CANSparkMax::FaultID::kHasReset)||intakePivot.GetFault(rev::CANSparkMax::FaultID::kMotorFault)|intakePivot.GetFault(rev::CANSparkMax::FaultID::kBrownout)){
    //     Intake::pivotInit();
    // }

    frc::SmartDashboard::PutNumber("Pivot built in Pos", intakePivotEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Pivot absolute Pos", intakePivotEncoder2.GetDistance());
    frc::SmartDashboard::PutNumber("pivot speed", intakePivot.Get());

    frc::SmartDashboard::PutNumber("roller speed", intakeRollers.Get());
    frc::SmartDashboard::PutBoolean("manual", robotData.controlData.manualMode);

    



}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData){
     if (robotData.controlData.saIntake)
    {
        // pivot down

        intakePivot_pidController.SetReference(12, rev::ControlType::kPosition,0);

        // if (intakePivotEncoder.GetPosition() > 1)
        // {
        //     intakePivot.Set(0.03);
        //     intakeRollers.Set(intakeRollerSpeed);
        //     intakeMecanum.Set(intakeMecanumSpeed);
        // }
        intakeRollers.Set(intakeRollerSpeed);
        intakeMecanum.Set(intakeMecanumSpeed);
        // once you're down
        // else
        // {
        //     intakePivot.Set(0);
        // }
    }
    // otherise, bring the intake back up slowly
    else if (robotData.controlData.saIntakeBackward)
    {
        intakeRollers.Set(-intakeRollerSpeed);
        //intakeMecanum.Set(-intakeMecanumSpeed);
    }
<<<<<<< Updated upstream
    // else if (robotData.controlData.saEjectBallsBackwards)
    // {
    //     intakeRollers.Set(-intakeRollersEjectSpeed);
    // }
    else
=======
    else if (robotData.controlData.saEjectBalls) //rollers backwards, pivot down
    {
        intakeRollers.Set(-intakeRollersEjectSpeed);
        intakePivot_pidController.SetReference(12, rev::ControlType::kPosition,0);

        // if (intakePivotEncoder2.GetDistance() > armDownPosition)
        // {
        //     intakePivot.Set(0.05);
        // }
        // // once you're down
        // else
        // {
        //     intakePivot.Set(0);
        // }
    }
    else //default case, everything up and not running
>>>>>>> Stashed changes
    {
        intakeRollers.Set(0);
        intakeMecanum.Set(0);

        // if (intakePivotEncoder2.GetDistance() < 0.56)
        // {
        //     intakePivot.Set(-intakePivotSpeed);
        // }
        // else
        // {
        //     intakePivot.Set(0);
        // }
        intakePivot_pidController.SetReference(0.5, rev::ControlType::kPosition, 1);
        //intakePivotEncoder.SetPosition(0);

    }
}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData){
<<<<<<< Updated upstream
    if(robotData.controlData.mIntakeDown){
        if (intakePivotEncoder.GetPosition() < 12)
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
=======
    // if(robotData.controlData.mIntakeDown){
    //     if (intakePivotEncoder2.GetDistance() < armDownPosition)
    //     {
    //         intakePivot.Set(intakePivotSpeed);
    //     }
    //     // once you're down
    //     else
    //     {
    //         intakePivot.Set(0);
    //     }
    // }else{
    //      if (intakePivotEncoder2.GetDistance() > 0)
    //     {
    //         intakePivot.Set(-intakePivotSpeed);
    //     }
    //     else
    //     {
    //         intakePivot.Set(0);
    //     }
    // }

    // if(robotData.controlData.mIntakeDown){
    //     intakePivot.Set(0.1); 
    // }else if(robotData.controlData.mIntakeUp){
    //     intakePivot.Set(-0.1); 
    // }else{
    //     intakePivot.Set(0);
    // }
>>>>>>> Stashed changes


    

    //intakeRollers.Set(robotData.controlData.mIntakeRollersBackward*.55);
    intakeRollers.Set(robotData.controlData.mIntakeRollers*.75);
    intakePivot.Set(robotData.controlData.mIntakeDown*0.2);
    
    if(robotData.controlData.mzeroing){
        intakePivotEncoder.SetPosition(0);
    }

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
    // frc::SmartDashboard::PutNumber("Pivot built in Pos", intakePivotEncoder.GetPosition());
    // frc::SmartDashboard::PutNumber("Pivot absolute Pos", intakePivotEncoder2.GetDistance());
    // frc::SmartDashboard::PutNumber("pivot speed", intakePivot.Get());

}

void Intake::rollersInit(){
    intakeRollers.RestoreFactoryDefaults();

    intakeRollers.SetInverted(true);

    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    intakeRollers_pidController.SetP(pkP);
    intakeRollers_pidController.SetI(pkI);
    intakeRollers_pidController.SetD(pkD);
    intakeRollers_pidController.SetIZone(pkIz);
    intakeRollers_pidController.SetFF(pkFF);
    intakeRollers_pidController.SetOutputRange(pkMinOutput, pkMaxOutput);

    intakeRollers.SetSmartCurrentLimit(45);

    intakeRollersEncoder.SetPosition(0);
}

void Intake::pivotInit(){
    intakePivot.RestoreFactoryDefaults();

    intakePivot.SetInverted(false);

    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

<<<<<<< Updated upstream
    intakePivot_pidController.SetP(wkP);
    intakePivot_pidController.SetI(wkI);
    intakePivot_pidController.SetD(wkD);
    intakePivot_pidController.SetIZone(wkIz);
    intakePivot_pidController.SetFF(wkFF);
    intakePivot_pidController.SetOutputRange(wkMinOutput, wkMaxOutput);
=======
    intakePivot_pidController.SetP(0.048,0);
    intakePivot_pidController.SetI(0,0);
    intakePivot_pidController.SetD(0.025,0);
    intakePivot_pidController.SetIZone(0,0);
    intakePivot_pidController.SetFF(0,0);
    intakePivot_pidController.SetOutputRange(-1, 1,0);

    intakePivot_pidController.SetP(0.042,1);
    intakePivot_pidController.SetI(0,1);
    intakePivot_pidController.SetD(0.03,1);
    intakePivot_pidController.SetIZone(0,1);
    intakePivot_pidController.SetFF(0,1);
    intakePivot_pidController.SetOutputRange(-1, 1,1);
>>>>>>> Stashed changes

    intakePivot.SetSmartCurrentLimit(45);
}

void Intake::mecanumInit(){
    intakeMecanum.RestoreFactoryDefaults();

    intakeMecanum.SetInverted(false);

    intakeMecanum.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    intakeMecanum_pidController.SetP(mkP);
    intakeMecanum_pidController.SetI(mkI);
    intakeMecanum_pidController.SetD(mkD);
    intakeMecanum_pidController.SetIZone(mkIz);
    intakeMecanum_pidController.SetFF(mkFF);
    intakeMecanum_pidController.SetOutputRange(mkMinOutput, mkMaxOutput);

    intakeMecanum.SetSmartCurrentLimit(45);

   
}



