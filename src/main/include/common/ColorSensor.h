#pragma once

#include <frc/smartdashboard/smartdashboard.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include "RobotData.h"
#include <string>


class ColorSensor {
    
    public: 
        void RobotInit();
        void RobotPeriodic(RobotData &RobotData);
        void Disabled();

        frc::Color detectColor(RobotData &robotData);
        void semiAutoMode(RobotData &robotData);
        void manualMode(RobotData &robotData);


    private:

        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 m_colorSensor{i2cPort};
        rev::ColorMatch m_colorMatcher;

        //tune these
        static constexpr frc::Color kBlueCargo = frc::Color(0.1763, 0.4508, 0.3728);
        static constexpr frc::Color kRedCargo = frc::Color(0.3546, 0.4361, 0.2093);

}; 