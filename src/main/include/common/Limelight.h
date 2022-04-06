#pragma once
#include "Constants.h"
#include "common/VisionLookup.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/span.h>
#include <cmath>
#include <deque>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>


struct RobotData;

struct LimelightData
{
    double xOffset;
    double yOffset;
    bool validTarget;

    int upperVal;
    int lowerVal;

    double upperValPos;
    double lowerValPos;

    double lowerValVel;
    double upperValVel;

    double desiredHoodPos;
    double desiredVel;
    double desiredHoodRollerVel;

    //how many degrees the turret needs to move in order to hit the target
    double turretDifference;
    double desiredTurretAngle;
    double correctDistance;

    //shooter corrections:
    //FINAL CORRECT DISTANCE
    double distanceOffset;
    double angleOffset;

    std::deque<double> desiredTurretAngles;
    double avgDesiredTurretAngle = 0;

    float hoodFlywheelRatio;

    bool unwrapping = false;


};

class Limelight
{

public:
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData, VisionLookup &visionLookup);

private:
    double distanceToTarget();
    void shooterOffset(const RobotData &robotData, LimelightData &limelightData);
    //double correctDistance(double angleOffset, double originalDistance);

    double getHoodPOS(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData);
    double getWheelVelocity(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData);
    double getTurretPOS(VisionLookup &visionLookup, LimelightData &limelightData, const RobotData &robotData);
    double getHoodRollerVel(LimelightData &limelightData, const RobotData &robotData);
    double getTurretTurnAngle(LimelightData &limelightData, const RobotData &robotData);
    float unwrappingVal = 0; //acts as a snapshot for when we're trying to wrap around

    //Intermediate step, is the LIMELIGHTS distance from target, not shooters
    double limelightDistance;

    void averageDesiredTurret(const RobotData &robotData, LimelightData &limelightData);

    //network table for limelight
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

};