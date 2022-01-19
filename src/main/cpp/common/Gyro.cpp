#include "common/Gyro.h"

void Gyro::RobotInit() {
    gyro.Calibrate();
    // gyro.SetAngleAdjustment(gyro.GetYaw());
    // gyro.ZeroYaw();
    // frc::SmartDashboard::PutString("lskfjl", "Calibrated1");
}

void Gyro::TeleopInit() {
    gyro.ZeroYaw();
    // frc::SmartDashboard::PutString("asfljkdfdf", "Zeroed");
}

void Gyro::AutonomousInit() {
    gyro.ZeroYaw();
    // frc::SmartDashboard::PutString("asfljkdfdf", "Zeroed");
}

void Gyro::RobotPeriodic(GyroData &gyroData) {

    // gyro.ZeroYaw();

    gyroData.rawYaw = -gyro.GetAngle();
    gyroData.rawPitch = gyro.GetPitch();
    gyroData.rawRoll = gyro.GetRoll();

    frc::SmartDashboard::PutNumber("rawYaw", gyroData.rawYaw);
    frc::SmartDashboard::PutNumber("rawPitch", gyroData.rawPitch);
    frc::SmartDashboard::PutNumber("rawRoll", gyroData.rawRoll);

}