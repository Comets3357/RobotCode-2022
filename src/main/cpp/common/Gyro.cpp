#include "common/Gyro.h"

void Gyro::RobotInit() {
    //gyro.Calibrate();
    // gyro.SetAngleAdjustment(gyro.GetYaw());
    // gyro.ZeroYaw();
    // frc::SmartDashboard::PutString("lskfjl", "Calibrated1");
}

void Gyro::TeleopInit(GyroData &gyroData) {
    gyro.ZeroYaw();
    gyroData.rawYaw = 0;
    gyroData.rawPitch = 0;
    gyroData.rawRoll = 0;
    // frc::SmartDashboard::PutString("asfljkdfdf", "Zeroed");
}

void Gyro::AutonomousInit(GyroData &gyroData) {
    gyro.ZeroYaw();
    gyroData.rawYaw = 0;
    gyroData.rawPitch = 0;
    gyroData.rawRoll = 0;
    gyroData.angularMomentum = 0;
    // frc::SmartDashboard::PutString("asfljkdfdf", "Zeroed");
}

void Gyro::RobotPeriodic(GyroData &gyroData) {

    gyroData.rawYaw = -gyro.GetAngle();
    gyroData.rawPitch = gyro.GetPitch();
    gyroData.rawRoll = gyro.GetRoll();
    gyroData.angularMomentum = gyro.GetRawGyroY();
    frc::SmartDashboard::PutNumber("angleMomentum",gyroData.angularMomentum);
    frc::SmartDashboard::PutNumber("yaw",gyro.GetYaw());
    frc::SmartDashboard::PutNumber("pitch",gyroData.rawPitch);
    frc::SmartDashboard::PutNumber("roll",gyroData.rawRoll);

    // frc::SmartDashboard::PutNumber("rawYaw", gyroData.rawYaw);
    // frc::SmartDashboard::PutNumber("rawPitch", gyroData.rawPitch);
    // frc::SmartDashboard::PutNumber("rawRoll", gyroData.rawRoll);

}

