/**
  Controller code. Encapsulates a controller for a single motor
  @author Alex Cornelio

*/

#include "wheelController.hpp"
#include "Arduino.hpp"

#include "fixedpoint.hpp"

/**
  WheelController constructor - initalise PID values using fixed_point_t type inside the PID class
*/
WheelController::WheelController(fixed_point_t kp_, fixed_point_t ki_, fixed_point_t kd_):
  pid(kp_, ki_, kd_)
{
  this->pid.init();
}

/**
  WheelController destructor
*/
WheelController::~WheelController() {}

/**
  Main function of WheelController. Computes PID gains, checks if it is saturated, shifts it to a type int
  with a value between 0 and 256
*/
char WheelController::tick(int actual_rpm, int ref_rpm) {

  fixed_point_t pid_output = this->pid.updatePID(int16_fp(actual_rpm), int16_fp(ref_rpm));
  pid_output = fp_saturate(pid_output, 0x007FFF00);
  int32_t tmp = *(int *)((char *)&pid_output + 1);
  int motor_cmd = (unsigned char)((tmp / 256) + 128);

  return motor_cmd;
}
