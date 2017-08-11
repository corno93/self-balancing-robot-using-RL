

#ifndef HEADER_PID
  #define HEADER_PID

#include "fixedpoint.h"
#define dt ((fixed_point_t)0x00000033)
#define dt_i ((fixed_point_t)0x00000500)


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
  fixed_point_t updatePID(fixed_point_t, fixed_point_t, char);
  int M1_rpm_to_serial(int);
  int M2_rpm_to_serial(int);

};



#endif
