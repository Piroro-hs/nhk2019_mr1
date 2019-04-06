#if !defined(__MOTOR__H__)
#define __MOTOR__H__

#include <mbed.h>

class Motor {
  private:
    PwmOut pwm;
    DigitalOut phase;
  public:
    Motor(PinName, PinName);
    void drive(float);
    void brake();
};

#endif // __MOTOR__H__
