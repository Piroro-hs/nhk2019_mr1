#include "motor.h"

#include <mbed.h>

Motor::Motor(PinName pwm_pin, PinName phase_pin) : pwm(PwmOut(pwm_pin)), phase(DigitalOut(phase_pin)) {
  pwm.period_us(1000);
}

void Motor::drive(float pow) {
  if (pow >= 0){
    phase.write(1);
    pwm.write(pow);
  } else {
    phase.write(0);
    pwm.write(-pow);
  }
}

void Motor::brake() {
  pwm.write(0);
}
