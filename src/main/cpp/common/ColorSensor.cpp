#include "common/ColorSensor.h"
#include "RobotData.h"

void ColorSensor::RobotInit(){
    
}

// right now it's just the default example code... 
void ColorSensor::RobotPeriodic(const RobotData &robotData, ColorSensorData &colorSensorData){
    
    // frc::SmartDashboard::PutNumber("cargo enum", colorSensorData.colorValue);
    if (robotData.arduinoData.ColorData == 11) {
      colorSensorData.colorValue = CargoColor::cargo_Blue;
    } else if (robotData.arduinoData.ColorData == 12) {
      colorSensorData.colorValue = CargoColor::cargo_Red;
    } else { // the value it'll be is 10
      colorSensorData.colorValue = CargoColor::cargo_Unknown;
      // frc::smartDashboard::PutBoolean("sensed Blue?", false);
      // frc::smartDashboard::PutBoolean("sensed Red?", false);
    }

}

void ColorSensor::DisabledPeriodic(const RobotData &robotData, ColorSensorData &colorSensorData){
    if (robotData.arduinoData.ColorData == 11) {
        colorSensorData.colorValue = CargoColor::cargo_Blue;
    } else if (robotData.arduinoData.ColorData == 12) {
        colorSensorData.colorValue = CargoColor::cargo_Red;
    } else { // the value it'll be is 10
        colorSensorData.colorValue = CargoColor::cargo_Unknown;
        // frc::smartDashboard::PutBoolean("sensed Blue?", false);
        // frc::smartDashboard::PutBoolean("sensed Red?", false);
    }
}



void ColorSensor::updateData(const RobotData &robotData, ColorSensorData &colorSensorData){
    // colorSensorData.currentColor = detectColor();
    // frc::SmartDashboard::PutBoolean("sensed Blue?", robotData.arduinoData.ColorData == 11);
    // frc::SmartDashboard::PutBoolean("sensed Red?", robotData.arduinoData.ColorData == 12);
    // frc::SmartDashboard::PutBoolean("sensed Nothing?", robotData.arduinoData.ColorData == 10);
    // frc::SmartDashboard::PutBoolean("not data", robotData.arduinoData.ColorData);
}
