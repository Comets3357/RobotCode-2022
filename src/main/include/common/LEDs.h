#include <frc/I2C.h>

struct LEDsData
{
    
};



class LEDs
{
public:
    void RobotInit();
    void TeleopInit();
    void RobotPeriodic();

private:
    //in constructor port, deviceaddress
    frc::I2C arduino = frc::I2C(frc::I2C::Port::kOnboard, 1);
};