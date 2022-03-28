#pragma once

#include <frc/smartdashboard/smartdashboard.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <string>

struct RobotData;

enum CargoColor {
    cargo_Blue,
    cargo_Red,
    cargo_Unknown
};

struct ColorSensorData {
    // frc::Color currentColor;
    int colorValue = CargoColor::cargo_Unknown;
};

class ColorSensor {
    public: 
        void RobotInit();
        void RobotPeriodic(const RobotData &robotData, ColorSensorData &colorSensorData);
        void DisabledPeriodic(const RobotData &robotData, ColorSensorData &colorSensorData);

    private:
        void updateData(const RobotData &robotData, ColorSensorData &colorSensorData);
        // frc::Color detectedColor;
}; 