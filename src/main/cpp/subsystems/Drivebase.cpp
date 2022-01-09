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