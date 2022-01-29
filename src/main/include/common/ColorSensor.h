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
    frc::Color currentColor;
    int colorValue = CargoColor::cargo_Unknown;
};

class ColorSensor {
    public: 
        void RobotInit();
        void RobotPeriodic(RobotData &RobotData);
        void Disabled();

    private:
        frc::Color detectColor();
        void semiAutoMode(RobotData &robotData);
        void manualMode(RobotData &robotData);
        void updateData(RobotData &robotData, ColorSensorData &colorSensorData);

        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 m_colorSensor{i2cPort};
        rev::ColorMatch m_colorMatcher;

        frc::Color detectedColor;
        double IR;
        std::string colorString;
        double confidence = 0.0;
        frc::Color matchedColor;
        uint32_t proximity;

        //tune these
        static constexpr frc::Color kBlueCargo = frc::Color(0.1763, 0.4508, 0.3728);
        static constexpr frc::Color kRedCargo = frc::Color(0.3546, 0.4361, 0.2093);
}; 