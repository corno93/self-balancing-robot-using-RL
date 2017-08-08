

#ifndef HEADER_PID
  #define HEADER_PID

#define dt 2

class PID
{
  public:
  int kp;
  int ki;
  int  kd;
  int integral_sum;
  int reference_point;
  int output;
  

  PID(int, int, int);
  ~PID();
  int updatePID(int, int);
  int M1_rpm_to_serial(int);
};



#endif
