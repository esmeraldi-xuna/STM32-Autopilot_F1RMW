#include "SonarMaxBotix.h"

SonarMaxBotix::SonarMaxBotix(PinName pin)
{
    _pin = pin;
    AnalogIn MaxBotix_pin(pin); 
    
}

float SonarMaxBotix::distance_analog(void)
{
    float distAnalog = (SonarMaxBotix::_pin)*SonarMaxBotix::_analogSensitivity/SonarMaxBotix::_mV2inch*SonarMaxBotix::_inch2cm;
    return distAnalog;
}

// float SonarMaxBotix::distance_pwm(void)
// {
//     float pulseDuration = pulseIn(SonarMaxBotix::_pin,HIGH); // pulseIn returns a time interval in microseconds
//     float distPwm = (float)pulseDuration/SonarMaxBotix::_pulse2inch*SonarMaxBotix::_inch2cm; 
//     return distPwm; 
// }
