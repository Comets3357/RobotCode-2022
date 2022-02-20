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

//BENCH TEST CODE
void Intake::TestPeriodic(const RobotData &robotData, IntakeData &intakeData){
    frc::SmartDashboard::PutBoolean("Intake abs encoder working", encoderPluggedIn(intakeData));
    frc::SmartDashboard::PutBoolean("Intake abs encoder reading in correct range", encoderInRange(intakeData));
    frc::SmartDashboard::PutNumber("Intake encoder position", intakePivotEncoderRev.GetPosition());
    frc::SmartDashboard::PutBoolean("Intake hit top dead stop", intakeData.topDeadStop);
    frc::SmartDashboard::PutBoolean("Intake hit bottom dead stop", intakeData.bottomDeadStop);
    frc::SmartDashboard::PutNumber("Intake actual abs encoder value", intakePivotEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("Intake expected top absolute encoder value", absIn);
    frc::SmartDashboard::PutNumber("Intake expected bottom absolute encoder value", absOut);
    frc::SmartDashboard::PutNumber("Intake pivot speed", intakeData.benchTestIntakePivotSpeed);
    
    checkDeadStop(intakeData);

    //runs the bench test sequence
    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && robotData.controlData.startBenchTest){ //checks if we're testing intake
        if (encoderPluggedIn(intakeData) && encoderInRange(intakeData)){ //checks if the encoder is working
            if (robotData.benchTestData.stage == 0){
                //pivot down
                intakeData.benchTestIntakePivotSpeed = .05; //sets the pivot speed
                intakeData.benchTestIntakeRollersSpeed = 0; //sets the rollers speed
                intakeData.benchTestSingulatorSpeed = 0; //sets the singulator (sideways wheel) speed
            } else if (robotData.benchTestData.stage == 1){
                //pivot up
                intakeData.benchTestIntakePivotSpeed = -.05;
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = 0;
            } else if (robotData.benchTestData.stage == 2){
                //zero everything - probably should be function
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = 0;
                intakePivotEncoderRev.SetPosition(0);
            } else if (robotData.benchTestData.stage == 3){
                //run rollers fowards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = robotData.benchTestData.currentSpeed;
                intakeData.benchTestSingulatorSpeed = 0;
                intakePivotEncoderRev.SetPosition(0);
            } else if (robotData.benchTestData.stage == 4){
                //run rollers backwards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = -robotData.benchTestData.currentSpeed;
                intakeData.benchTestSingulatorSpeed = 0;
            } else if (robotData.benchTestData.stage == 5){
                //run singulator forwards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = robotData.benchTestData.currentSpeed; 
            } else if (robotData.benchTestData.stage == 6){
                //run singulator backwards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = -robotData.benchTestData.currentSpeed;
            } else if (robotData.benchTestData.PIDMode && robotData.benchTestData.stage > 6){ //starts testing in pid mode
                if (robotData.benchTestData.stage == 7){
                    // bring pivot down
                    intakeData.benchTestIntakePivotSpeed = 0;
                    intakeData.benchTestIntakeRollersSpeed = 0;
                    intakeData.benchTestSingulatorSpeed = 0;
                    intakePivot_pidController.SetReference(revOut, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
                } else if (robotData.benchTestData.stage == 8){
                    // bring pivot up
                    intakeData.benchTestIntakePivotSpeed = 0;
                    intakeData.benchTestIntakeRollersSpeed = 0;
                    intakeData.benchTestSingulatorSpeed = 0;
                    intakePivot_pidController.SetReference(revIn, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
                } else {
                    intakeData.benchTestIntakePivotSpeed = 0;
                    intakeData.benchTestIntakeRollersSpeed = 0;
                    intakeData.benchTestSingulatorSpeed = 0;
                }
            } else {
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = 0;
            }
        }

        //sets the speed of the motors (unless the pivot hit a dead stop)
        if (!intakeData.topDeadStop && !intakeData.bottomDeadStop){
            intakePivot.Set(intakeData.benchTestIntakePivotSpeed);
        } else {
            intakePivot.Set(0);
        }

        intakeRollers.Set(intakeData.benchTestIntakeRollersSpeed);
        intakeSingulator.Set(intakeData.benchTestSingulatorSpeed);
    } else {
        intakeData.benchTestIntakePivotSpeed = 0; //if not testing intake, then the speed of the motors is set to 0
        intakeData.benchTestIntakeRollersSpeed = 0;
        intakeData.benchTestSingulatorSpeed = 0;
    }
}

//checks to see if the encoder is reading zero because if it is that means the encoder was most likley unplugged and the current values are wrong and we don't want to run any motors
bool Intake::encoderPluggedIn(const IntakeData &intakeData){
    if (intakePivotEncoderAbs.GetOutput() > 0.03){
        //constantly updates the intake rev encoder based on the absolute encoder values 
        if (tickCount > 45){
            intakePivotEncoderRev.SetPosition(absoluteToREV(intakePivotEncoderAbs.GetOutput()));
            tickCount = (tickCount + 1) % 50;
        } else {
            tickCount = (tickCount + 1) % 50;
        }

        return true; //returns true to indicate that the encoder is functioning
    } else {
        return false;
    }
}

//checks if the encoder is reading values in the incorrect range, and if the values aren't reasonable, then the motors stop running in the bench test function
bool Intake::encoderInRange(const IntakeData &intakeData){
    if (intakePivot.Get() > 0 && intakePivotEncoderAbs.GetOutput() < absOut - .01){
        intakePivot.Set(0);
        return false;
    } else if (intakePivot.Get() < 0 && intakePivotEncoderAbs.GetOutput() > absIn + .01){
        intakePivot.Set(0);
        return false;
    } else {
        return true;
    }
}

//checks if the motor has hit a dead stop
void Intake::checkDeadStop(IntakeData &intakeData){
    if (intakeData.benchTestIntakePivotSpeed > 0 && intakePivotEncoderAbs.GetOutput() < absOut + .01){
        intakeData.topDeadStop = false;
        intakeData.bottomDeadStop = true;
    } else if (intakeData.benchTestIntakePivotSpeed < 0 && intakePivotEncoderAbs.GetOutput() > absIn - .01){
        intakeData.topDeadStop = true;
        intakeData.bottomDeadStop = false;
    } else {
        intakeData.topDeadStop = false;
        intakeData.bottomDeadStop = false;
    }
}

void Intake::DisabledPeriodic(const RobotData &robotData, IntakeData &intakeData){
    updateData(robotData, intakeData);
}