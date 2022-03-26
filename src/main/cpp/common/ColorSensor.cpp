#include "common/ColorSensor.h"
#include "RobotData.h"

void ColorSensor::RobotInit(){
    
}

// right now it's just the default example code... 
void ColorSensor::RobotPeriodic(const RobotData &robotData, ColorSensorData &colorSensorData){
    

    if (robotData.ledsData.ColorData == 11) {
      colorSensorData.colorValue = CargoColor::cargo_Blue;
    } else if (robotData.ledsData.ColorData == 12) {
      colorSensorData.colorValue = CargoColor::cargo_Red;
    } else { // the value it'll be is 10
      colorSensorData.colorValue = CargoColor::cargo_Unknown;
      // frc::smartDashboard::PutBoolean("sensed Blue?", false);
      // frc::smartDashboard::PutBoolean("sensed Red?", false);
    }

}

void ColorSensor::Disabled(){

}



void ColorSensor::updateData(const RobotData &robotData, ColorSensorData &colorSensorData){
    // colorSensorData.currentColor = detectColor();
    frc::SmartDashboard::PutBoolean("sensed Blue?", robotData.ledsData.ColorData == 11);
    frc::SmartDashboard::PutBoolean("sensed Red?", robotData.ledsData.ColorData == 12);
    frc::SmartDashboard::PutBoolean("sensed Nothing?", robotData.ledsData.ColorData == 10);
    frc::SmartDashboard::PutBoolean("not data", robotData.ledsData.ColorData);
}
