#include "common/ColorSensor.h"
#include "RobotData.h"

void ColorSensor::RobotInit(){
    m_colorMatcher.AddColorMatch(kBlueCargo);
    m_colorMatcher.AddColorMatch(kRedCargo);
    
    // uint8_t pulses = 5;
    // m_colorSensor.ConfigureProximitySensorLED(rev::ColorSensorV3::LEDPulseFrequency::k60kHz, rev::ColorSensorV3::LEDCurrent::kPulse125mA, pulses);
}

// right now it's just the default example code... 
void ColorSensor::RobotPeriodic(RobotData &robotData){
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    detectedColor = m_colorSensor.GetColor();
    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = m_colorSensor.GetIR();
    
    /**
     * Run the color match algorithm on our detected color
     */
    confidence = 0.0;
    matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (detectedColor.blue >= detectedColor.red + .15) {
      colorString = "Blue";
      robotData.colorSensorData.colorValue = CargoColor::cargo_Blue;
      // // frc::smartDashboard::PutBoolean("sensed Blue?", true);
      // frc::smartDashboard::PutBoolean("sensed Red?", false);
    } else if (detectedColor.red >= detectedColor.blue + .15) {
      colorString = "Red";
      robotData.colorSensorData.colorValue = CargoColor::cargo_Red;
      // frc::smartDashboard::PutBoolean("sensed Blue?", false);
      // frc::smartDashboard::PutBoolean("sensed Red?", true);
    } else {
      colorString = "Unknown";
      robotData.colorSensorData.colorValue = CargoColor::cargo_Unknown;
      // frc::smartDashboard::PutBoolean("sensed Blue?", false);
      // frc::smartDashboard::PutBoolean("sensed Red?", false);
    }

    //Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
    // frc::smartDashboard::PutNumber("Red", detectedColor.red);
    // frc::smartDashboard::PutNumber("Green", detectedColor.green);
    // frc::smartDashboard::PutNumber("Blue", detectedColor.blue);
    // frc::smartDashboard::PutNumber("IR", IR);
    
    //gets distance from object
    proximity = m_colorSensor.GetProximity();

    //displays proximity and color values to the smart dashboard
    // frc::smartDashboard::PutNumber("Proximity", proximity);
    // frc::smartDashboard::PutString("colorString", colorString);
}

void ColorSensor::Disabled(){

}

void ColorSensor::semiAutoMode(RobotData &robotData){
  
}

void ColorSensor::manualMode(RobotData &robotData){

}

void ColorSensor::updateData(RobotData &robotData, ColorSensorData &colorSensorData){
    colorSensorData.currentColor = detectColor();
}

frc::Color ColorSensor::detectColor(){
    return detectedColor;
}