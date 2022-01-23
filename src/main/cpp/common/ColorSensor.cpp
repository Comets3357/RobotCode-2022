#include "common/ColorSensor.h"
#include "RobotData.h"

void ColorSensor::RobotInit()
{
    m_colorMatcher.AddColorMatch(kBlueCargo);
    m_colorMatcher.AddColorMatch(kRedCargo);

}

// right now it's just the default example code... 
void ColorSensor::RobotPeriodic(RobotData &RobotData)
{
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

    if (matchedColor == kBlueCargo) {
      colorString = "Blue";
    } else if (matchedColor == kRedCargo) {
      colorString = "Red";
    }  else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("IR", IR);
    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    proximity = m_colorSensor.GetProximity();

    frc::SmartDashboard::PutNumber("Proximity", proximity);


}

void ColorSensor::Disabled()
{

}

void ColorSensor::semiAutoMode(RobotData &robotData)
{
  
}

void ColorSensor::manualMode(RobotData &robotData)
{

}

void ColorSensor::updateData(RobotData &robotData, ColorSensorData &colorSensorData)
{
    colorSensorData.currentColor = detectColor();
}

frc::Color ColorSensor::detectColor()
{
    return detectedColor;
}