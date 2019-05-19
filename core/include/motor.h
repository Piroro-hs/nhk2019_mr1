#if !defined(__MOTOR__H__)
#define __MOTOR__H__

#include <mbed.h>

class Motor {
  private:
    PwmOut pwm;
    DigitalOut phase;
  public:
    Motor(PinName pwm_pin, PinName phase_pin);
    void drive(float pow);
    void brake();
};

#endif // __MOTOR__H__
