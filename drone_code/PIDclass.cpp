#include "PIDclass.h"

PIDclass::PIDclass(double kp, double ki, double kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;

  pid_p=0;
  pid_i=0;
  pid_d=0;
}

void PIDclass::calculate_PID(unsigned long elapsedTime, float input, float feedback){
  float error = feedback - input;
  pid_p = _kp*error;
  if(-3 <error <3) pid_i = pid_i+(_ki*error);  

  pid_d = _kd*((error - previous_error)/elapsedTime);
  PID = 4*(pid_p + pid_i + pid_d);

  previous_error = error;
}
