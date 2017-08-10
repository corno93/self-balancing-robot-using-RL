

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
  int error_prev;
  

  PID(int, int, int);
  ~PID();
  int updatePID(int, int, char);
  int M1_rpm_to_serial(int);
  int M2_rpm_to_serial(int);

};



#endif
