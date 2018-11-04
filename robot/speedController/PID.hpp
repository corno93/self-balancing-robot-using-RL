/**
  PID header file
  @author Alex Cornelio

*/

#ifndef HEADER_PID
  #define HEADER_PID

#include "fixedpoint.hpp"

// time interval (200ms) and inverse time interval
#define dt ((fixed_point_t)0x00000033)
#define dt_i ((fixed_point_t)0x00000500)

/**
  PID class declaration
*/
class PID
{
  public:
  fixed_point_t kp;
  fixed_point_t ki;
  fixed_point_t  kd;
  fixed_point_t integral_sum;
  fixed_point_t error_prev;
  

  PID(fixed_point_t, fixed_point_t, fixed_point_t);
  ~PID();
  void init();
  fixed_point_t updatePID(fixed_point_t, fixed_point_t);

};


#endif
