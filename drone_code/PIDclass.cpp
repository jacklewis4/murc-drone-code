#include "PIDclass.h"

PIDclass::PIDclass(double kp, double ki, double kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;

  pid_p=0;
  pid_i=0;
  pid_d=0;
}

void PIDclass::calculate_PID(unsigned long elapsedTime, double throttle, float error){
  pid_p = _kp*error;
  if(-3 <error <3) pid_i = pid_i+(_ki*error);  

  pid_d = _kd*((error - previous_error)/elapsedTime);
  PID = 4*(pid_p + pid_i + pid_d);
  
  if(PID < -1000) PID = -1000;
  if(PID > 1000) PID = 1000;
  
  pwmLeft = throttle + PID;
  pwmRight = throttle - PID;
  
  if(pwmRight < 1000) pwmRight = 1000;
  else if(pwmRight > 2000) pwmRight = 2000;
  if(pwmLeft < 1000) pwmLeft = 1000;
  else if(pwmLeft > 2000) pwmLeft = 2000;
}
