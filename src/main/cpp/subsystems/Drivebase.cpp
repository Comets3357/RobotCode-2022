#include "subsystems/Drivebase.h"
#include "RobotData.h"

void Drivebase::RobotInit()
{
    dbLF.ConfigFactoryDefault();
    dbLC.ConfigFactoryDefault();
    dbLB.ConfigFactoryDefault();
    dbRF.ConfigFactoryDefault();
    dbRC.ConfigFactoryDefault();
    dbRB.ConfigFactoryDefault();
    

    dbLC.Follow(dbLF);
    dbLB.Follow(dbLF);
    dbRC.Follow(dbRF);
    dbRB.Follow(dbRF);

    dbLF.SetInverted(true);
    dbLC.SetInverted(true);
    dbLB.SetInverted(true);

    dbRF.SetInverted(false);    
    dbRC.SetInverted(false);
    dbRB.SetInverted(false);

    dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbLC.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbLB.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRC.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRB.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);

    // NEED TO SET CURRENT LIMIT
    /**
  * Configure the current limits that will be used
  * Stator Current is the current that passes through the motor stators.
  *  Use stator current limits to limit rotor acceleration/heat production
  * Supply Current is the current that passes into the controller from the supply
  *  Use supply current limits to prevent breakers from tripping
  *
  *                                                               enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  */
    dbLF.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbLC.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbLB.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbRF.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbRC.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    dbRB.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 45, 50, 1.0));

    dbLF.Config_kP(0, 0.039271);
    dbLF.Config_kD(0, 0);

    dbRF.Config_kP(0, 0.039271);
    dbRF.Config_kD(0, 0);


    dbLF.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    dbRF.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
   
}

void Drivebase::RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    updateData(robotData, drivebaseData);

    if (frc::DriverStation::IsEnabled())
    {
        dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbLC.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbLB.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbRC.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        dbRB.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    }

    teleopControl(robotData);

    frc::SmartDashboard::PutNumber("rEncoder", dbLF.GetSelectedSensorPosition());
}

void Drivebase::DisabledInit()
{
    
    dbLF.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    dbRF.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    dbLF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbLC.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbLB.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRF.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRC.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    dbRB.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
}

// updates encoder and gyro values
void Drivebase::updateData(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    // //add back wheel encoders at some point
    // drivebaseData.currentLDBPos = dbLMEncoder.GetPosition();
    // drivebaseData.currentRDBPos = dbRMEncoder.GetPosition();

    // drivebaseData.lDriveVel = dbRMEncoder.GetVelocity();
    // drivebaseData.rDriveVel = dbLMEncoder.GetVelocity();
}
// driving functions:

// adjusts for the deadzone and converts joystick input to velocity values for PID
void Drivebase::teleopControl(const RobotData &robotData)
{

    // dbLF.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 2*2048*10);
    // dbRF.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 2*2048*10);
    

    double tempLDrive = robotData.controlData.lDrive;
    double tempRDrive = robotData.controlData.rDrive;

    // converts from tank to arcade drive, limits the difference between left and right drive
    double frontBack = robotData.controlData.maxStraight * (tempLDrive + tempRDrive) / 2;
    double leftRight = robotData.controlData.maxTurn * (tempRDrive - tempLDrive) / 2;

    //deadzone NOT needed for drone controller
    if (tempLDrive <= -0.08 || tempLDrive >= 0.08)
    {
        tempLDrive = (frontBack - leftRight);
    }
    else
    {
        tempLDrive = 0;
    }

    if (tempRDrive <= -0.08 || tempRDrive >= 0.08)
    {
        tempRDrive = (frontBack + leftRight);
    }
    else
    {
        tempRDrive = 0;
    }

    //set as percent vbus
    dbLF.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, tempLDrive);
    dbRF.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, tempRDrive);
}