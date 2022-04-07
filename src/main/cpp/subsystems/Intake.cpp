#include "RobotData.h"
#include <iostream>
#include <cmath>

void Intake::RobotInit()
{
    pivotInit();
    rollersInit();
    singulatorInit();

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

    intakePivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    intakePivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    intakePivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, revIn);
    intakePivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, revOut);
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


}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData){
    
    //updates rev encoder if abs encoder is working
    encoderPluggedIn();

//INTAKE FUNCTIONALITY
    if (robotData.controlData.saIntake) //intaking
    {
        // pivot down
        intakePivot_pidController.SetReference(revOut - 0.5, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

        //run rollers, singulator
        intakeRollers.Set(intakeRollerSpeed);
        intakeSingulator.Set(intakesingulatorSpeed);
        
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
        intakeSingulator.Set(-intakesingulatorSpeed);
        intakePivot_pidController.SetReference(revOut - 0.1, rev::CANSparkMaxLowLevel::ControlType::kPosition,0);

    }
    else //default case, everything up and not running
    {
        if(!intakeData.intakeIdle){ // run the singulator while the intake is not idle (basically run it for a second after the intake stops)
            intakeSingulator.Set(intakesingulatorSpeed);
        }else{
            intakeSingulator.Set(0);
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

    //side wheel running
    if(robotData.controlData.mSideWheelForward){
        intakeSingulator.Set(intakesingulatorSpeed);
    }else if(robotData.controlData.mSideWheelBackward){
        intakeSingulator.Set(-intakesingulatorSpeed);
    }else{
        intakeSingulator.Set(0);
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
    if (robotData.controlData.saIntakeIdle) {
        return true;
    }
    else if (robotData.controlData.saIntake || robotData.controlData.saIntakeBackward){
        // if something is commanding the intake then idle count is 100
        idleCount = 100;
        return false; // hasn't 
    } else if(idleCount > 0){ // nothing is commanding the intake, and the idle count is counting down
        idleCount--;
        return false;
    } else { // noothing has been commanding the intake for 100 ticks, intake is considered idle
        return true;
    }

}

//converts the intake from absolute values to values the rev can read
double Intake::absoluteToREV(double value){
    double slope = (revOut - revIn)/(absOut - absIn);
    double b = revIn - (slope*absIn);
    return ((value*slope) + b);
}

void Intake::DisabledPeriodic(const RobotData &robotData, IntakeData &intakeData){
    updateData(robotData, intakeData);
}

/**
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * BENCH TEST CODE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 **/

void Intake::TestInit(){
    //sets pids for bench test
    // intakePivot_pidController.SetP(0.23, 0);
    // intakePivot_pidController.SetOutputRange(-0.3, 0.2, 0);
}

void Intake::TestPeriodic(const RobotData &robotData, IntakeData &intakeData){
    //diagnosing issues with smart dashboard
    frc::SmartDashboard::PutBoolean("Intake abs encoder working", encoderPluggedIn());
    frc::SmartDashboard::PutBoolean("Intake abs encoder reading in correct range", encoderInRange());
    frc::SmartDashboard::PutNumber("Intake encoder position", intakePivotEncoderRev.GetPosition());
    frc::SmartDashboard::PutBoolean("Intake hit inner dead stop", intakeData.topDeadStop);
    frc::SmartDashboard::PutBoolean("Intake hit outer dead stop", intakeData.bottomDeadStop);
    frc::SmartDashboard::PutNumber("Intake actual abs encoder value", intakePivotEncoderAbs.GetOutput());
    frc::SmartDashboard::PutNumber("Intake expected top absolute encoder value", absIn);
    frc::SmartDashboard::PutNumber("Intake expected bottom absolute encoder value", absOut);
    frc::SmartDashboard::PutNumber("Intake pivot speed", intakeData.benchTestIntakePivotSpeed);

    //calls dead stop functions so the motors know when to stop
    checkDeadStop(intakeData);

    //runs the bench test sequence
    if (robotData.benchTestData.testStage == BenchTestStage::BenchTestStage_Intake && (robotData.controlData.manualBenchTest || robotData.controlData.autoBenchTest)){ //checks if we're testing intake
        if (encoderPluggedIn() && encoderInRange()){ //checks if the encoder is working
            if (robotData.benchTestData.stage == 0){
                //pivot down
                if (!robotData.benchTestData.PIDMode){
                    intakeData.benchTestIntakePivotSpeed = .05; //sets the pivot speed
                    intakeData.benchTestIntakeRollersSpeed = 0; //sets the rollers speed
                    intakeData.benchTestSingulatorSpeed = 0; //sets the singulator (sideways wheel) speed
                } else {
                    intakeData.benchTestIntakeRollersSpeed = 0;
                    intakeData.benchTestSingulatorSpeed = 0;
                    intakePivot_pidController.SetReference(revOut, rev::CANSparkMaxLowLevel::ControlType::kPosition); //runs the intake out with PIDs
                }
            } else if (robotData.benchTestData.stage == 1){
                //pivot up
                if (!robotData.benchTestData.PIDMode){
                    intakeData.benchTestIntakePivotSpeed = -.05;
                    intakeData.benchTestIntakeRollersSpeed = 0;
                    intakeData.benchTestSingulatorSpeed = 0;
                } else {
                    intakeData.benchTestIntakeRollersSpeed = 0;
                    intakeData.benchTestSingulatorSpeed = 0;
                    intakePivot_pidController.SetReference(revIn, rev::CANSparkMaxLowLevel::ControlType::kPosition); //runs the intake in with PIDs
                }
            } else if (robotData.benchTestData.stage == 2){
                //run rollers fowards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = robotData.benchTestData.currentSpeed;
                intakeData.benchTestSingulatorSpeed = 0;
            } else if (robotData.benchTestData.stage == 3){
                //run rollers backwards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = -robotData.benchTestData.currentSpeed;
                intakeData.benchTestSingulatorSpeed = 0;
            } else if (robotData.benchTestData.stage == 4){
                //run singulator forwards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = robotData.benchTestData.currentSpeed; 
            } else if (robotData.benchTestData.stage == 5){
                //run singulator backwards
                intakeData.benchTestIntakePivotSpeed = 0;
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = -robotData.benchTestData.currentSpeed;
            } else {
                intakeData.benchTestIntakePivotSpeed = 0; //if the intake stage isn't within 0 to 5, then the speeds get set to 0
                intakeData.benchTestIntakeRollersSpeed = 0;
                intakeData.benchTestSingulatorSpeed = 0;
                intakePivot.Set(0);
                intakeRollers.Set(0);
                intakeSingulator.Set(0);
            }
        } else {
            intakeData.benchTestIntakePivotSpeed = 0; //if the sensors aren't working, then the speeds get set to 0
            intakeData.benchTestIntakeRollersSpeed = 0;
            intakeData.benchTestSingulatorSpeed = 0;
            intakePivot.Set(0);
            intakeRollers.Set(0);
            intakeSingulator.Set(0);
        }

        //if statement to make sure the speed doesn't interfere with PID mode
        if (!robotData.benchTestData.PIDMode){
            //sets the speed of the motors (unless the pivot hit a dead stop)
            if (!intakeData.topDeadStop && !intakeData.bottomDeadStop){
                intakePivot.Set(intakeData.benchTestIntakePivotSpeed);
            } else {
                intakePivot.Set(0); //sets the intake speed to 0 if the motor is at a dead stop
            }
        }

        intakeRollers.Set(intakeData.benchTestIntakeRollersSpeed);
        intakeSingulator.Set(intakeData.benchTestSingulatorSpeed);
    } else {
        intakeData.benchTestIntakePivotSpeed = 0; //if not testing intake, then speeds get set to 0
        intakeData.benchTestIntakeRollersSpeed = 0;
        intakeData.benchTestSingulatorSpeed = 0;
        intakePivot.Set(0);
        intakeRollers.Set(0);
        intakeSingulator.Set(0);
    }
}

//checks to see if the encoder is reading zero because if it is that means the encoder was most likley unplugged and the current values are wrong and we don't want to run any motors
bool Intake::encoderPluggedIn(){
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
bool Intake::encoderInRange(){
    if (intakePivotEncoderAbs.GetOutput() < absOut - .01){
        return false;
    } else if (intakePivotEncoderAbs.GetOutput() > absIn + .01){
        return false;
    } else {
        return true;
    }
}

//sets the limits and sets variables to the limits to let the TestPeriodic function know when to stop running the motors
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