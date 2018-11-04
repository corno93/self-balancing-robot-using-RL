/**
  PID functions for arduino control loop
  @author Alex Cornelio

*/


#include "PID.hpp"
#include "fixedpoint.hpp"

/**
  Constructor for PID class
*/
PID::PID(fixed_point_t kp_, fixed_point_t ki_, fixed_point_t kd_)
{
  kp = kp_;
  ki = ki_;
  kd = kd_;
  integral_sum = 0;
  error_prev = 0;
}

/**
  Destructor for PID class
*/
PID::~PID()
{
  
}

/**
  Initalise the integral sum and previous error to 0 for PID class
*/
void PID::init() {
  
  integral_sum = 0;
  error_prev = 0;
}

/**
  Compute and return PID value of type fixed_point_t
*/
fixed_point_t PID::updatePID(fixed_point_t actual, fixed_point_t ref)
{ 
    fixed_point_t error, error_kp, error_ki, error_kd, pid_cmd_serial, pid_cmd, derivative;

    error = ref - actual;                       //get RPM error
    
    error_kp = fp_mul(kp, error);

    integral_sum += fp_mul(error, dt);          //integral error
    error_ki = fp_mul(ki, integral_sum);

    derivative = fp_mul(error - error_prev, dt_i);  //derivative
    error_kd = fp_mul(kd, derivative);

    error_prev = error;                         //save prev error

    return (error_kp + error_ki + error_kd);
}



