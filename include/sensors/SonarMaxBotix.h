/////////////////////////////////////////////////////////////////////
/// \class SonarMaxBotix SonarMaxBotix.h
/// \brief Support for MaxBotix Sonar
/////////////////////////////////////////////////////////////////////

#include <mbed.h>

class SonarMaxBotix
{
public:

    SonarMaxBotix(PinName pin);
    
    float           distance_analog(void);
    // float           distance_pwm(void);
    
private:

    PinName        _pin;        

protected:

    float          _analogSensitivity = 4.88; 
    float          _mV2inch = 9.8; 
    float          _inch2cm = 2.54;
    float          _pulse2inch = 147.0;


};