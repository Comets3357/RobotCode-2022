#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalSource.h>

struct RobotData;

struct ShooterData
{
    bool readyShoot;
    float currentTurretAngle;

    //for rolling average of turret offset 
    std::deque<double> offsetPos;
    double avgTurretOffsetPos = 0;

    //Bench test
    bool hoodTopDeadStop = false;
    bool hoodBottomDeadStop = false;
    bool turretTopDeadStop = false;
    bool turretBottomDeadStop = false;
    float benchTestShooterHoodSpeed = 0;
    float benchTestFlyWheelSpeed = 0;
    float benchTestTurretSpeed = 0;
};

class Shooter{

    public:
        void RobotInit(ShooterData &shooterData);
        void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
        void DisabledInit();
        void DisabledPeriodic(const RobotData &robotData, ShooterData &shooterData);
        void EnabledInit(ControlData &controlData, ShooterData &shooterData);
        void TestPeriodic(const RobotData &robotData, ShooterData &shooterData);
    
    private:
        void manual(const RobotData &robotData, ShooterData &shooterData);
        void semiAuto(const RobotData &robotData, ShooterData &shooterData);
        void updateData(const RobotData &robotData, ShooterData &shooterData);

        //converting hood
        double hoodRevtoAngle(double value);
        double hoodAngletoRev(double value);
        double HoodabsoluteToREV(double value);

        //converting turret
        double turretConvertFromAngleToAbs(double angle);
        double turretAbsoluteToREV(double value);
        double turretGyroOffset(double value);
        double turretRevtoAngle(double rev);
        double averageTurretGyroOffset(const RobotData &robotData, ShooterData &shooterData);
        double arbFF = 0;

        //init 
        void flyWheelInit();
        void hoodRollerInit();
        void shooterHoodInit();
        void shooterTurretInit();
        
        //gets and sets
        double getWheelVel();
        void setShooterWheel(double speed, double pidSlot);
        void setTurret_Pos(double pos, ShooterData &shooterData);

        //checks
        void checkReadyShoot(ShooterData &shooterData);
        bool encoderPluggedInTurret();
        bool encoderPluggedInHood();
        void saTurret(const RobotData &robotData, ShooterData &shooterData);
        void turretControlTurn(float controlTurretDirection, const RobotData &robotData, ShooterData &shooterData);


        //FIXED SHOTS
        void outerLaunch(const RobotData &robotData);
        void innerLaunch(const RobotData &robotData);
        void wall(const RobotData &robotData);
        void fender(const RobotData &robotData);

        //bench test
        bool encoderInRangeHood();
        bool encoderInRangeTurret();
        void checkHoodDeadStop(ShooterData &shooterData);
        void checkTurretDeadStop(ShooterData &shooterData);

        //shooter velocity min threshold
        int readyShootLimit;
        bool isTurretStatic;
        bool isZeroed_Turret = true; //checks if the abs ecoder is zeroed at the beginning is a flag
        bool isZeroed_Hood = true;

        //Flywheel Lead
        rev::CANSparkMax flyWheel = rev::CANSparkMax(shooterWheelID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder flyWheelLeadEncoder = flyWheel.GetEncoder();
        rev::SparkMaxPIDController flyWheelLead_pidController = flyWheel.GetPIDController();

        //lip roller
        rev::CANSparkMax hoodRoller = rev::CANSparkMax(hoodRollerID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder hoodRollerEncoderRev = hoodRoller.GetEncoder();
        rev::SparkMaxPIDController hoodRoller_pidController = hoodRoller.GetPIDController();

        //hood, rev encoder, pid
        rev::CANSparkMax shooterHood = rev::CANSparkMax(shooterHoodID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder shooterHoodEncoderRev = shooterHood.GetEncoder();
        rev::SparkMaxPIDController shooterHood_pidController = shooterHood.GetPIDController();

        rev::CANSparkMax shooterTurret = rev::CANSparkMax(shooterTurretID, rev::CANSparkMax::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder shooterTurretEncoderRev = shooterTurret.GetEncoder();
        rev::SparkMaxPIDController shooterTurret_pidController = shooterTurret.GetPIDController();

        //turret abs encoder
        frc::DigitalInput m_inputTurret{TurretAbsoluteEncoderPort};
        frc::DutyCycle shooterTurretEncoderAbs = frc::DutyCycle{m_inputTurret};
        
        //hood abs encoder
        frc::DigitalInput m_inputHood{HoodAbsoluteEncoderPort};
        frc::DutyCycle shooterHoodEncoderAbs = frc::DutyCycle{m_inputHood};

};