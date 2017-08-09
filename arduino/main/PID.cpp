

#include "PID.h"


PID::PID(int kp_, int ki_, int kd_)
{
  kp = kp_;
  ki = ki_;
  kd = kd_;

}

PID::~PID()
{
  
}

/*int PID::updatePID(int actual, int ref){
  int error, error_kp, integral,error_ki, error_kd, error_prev, pid_cmd_serial, pid_cmd, derivative;
  
    error = ref - actual;  //get RPM error
    
    error_kp = kp*error; //proportional

    integral+=error*(dt/10);  //integral
    error_ki = ki*integral;

    derivative = (error - error_prev) /dt; 
    error_kd = kd*(derivative/10);

    error_prev = error;   //save next previous error

    //debugging...
    Serial.print("The error is ");Serial.println(error);
//    Serial.print("The error derivative is ");Serial.println(error_d);
 //   Serial.print("The integral is ");Serial.println(integral);

    pid_cmd = (error_kp + error_ki)/10;// + error_kd;              //tuning tech at: http://robotsforroboticists.com/pid-control/
  // Serial.print("motor1_rpmcmd ");Serial.println(motor1_rpmcmd);

    pid_cmd_serial = M1_rpm_to_serial(pid_cmd);
  //  Serial1.write(motor1_serialcmd);

//    Serial.print("pid_cmd_serial from PID: ");Serial.println(pid_cmd_serial);
   // Serial1.write(pid_cmd_serial);
   return pid_cmd_serial;
}*/

int PID::M1_rpm_to_serial(int rpm_cmd)
{
  int serial_cmd;
  serial_cmd = (rpm_cmd + 400.68)/6.3504;     //m1 eqn
  if (serial_cmd > 127)
  {
    serial_cmd = 127;
  }else if (serial_cmd < 1)
  {
    serial_cmd = 1;
  }
  return (serial_cmd);
}

int PID::M2_rpm_to_serial(int rpm_cmd)
{
  int serial_cmd;
  serial_cmd = (rpm_cmd - 1209)/-6.3586;     //m2 eqn 
  if (serial_cmd > 255)
  {
    serial_cmd = 255;
  }else if (serial_cmd < 128)
  {
    serial_cmd = 128;
  }
  return (serial_cmd);
}



