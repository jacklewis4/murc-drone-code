#ifndef PIDclass_h
#define PIDclass_h
class PIDclass {
  public:
    PIDclass(double kp, double ki, double kd);
    void calculate_PID(unsigned long elapsedTime, double throttle, float desired_angle);
    float pwmLeft;
    float pwmRight;
    
  private:
    double _kp;
    double _ki;
    double _kd;
    float PID;
    float previous_error;
    float pid_p;
    float pid_i;
    float pid_d;
};
#endif
