#include "WheelController.h"
#include "Arduino.h"

#include "fixedpoint.h"


WheelController::WheelController(fixed_point_t kp_, fixed_point_t ki_, fixed_point_t kd_):
  pid(kp_, ki_, kd_)
{
  this->pid.init();
}

WheelController::~WheelController() {}

char WheelController::tick(int actual_rpm, int ref_rpm) {

//  Serial.println(this->pid.kp, HEX);

  fixed_point_t pid_output = this->pid.updatePID(int16_fp(actual_rpm), int16_fp(ref_rpm));
  pid_output = fp_saturate(pid_output, 0x007FFF00);
  int32_t tmp = *(int *)((char *)&pid_output + 1);
  int motor_cmd = (unsigned char)((tmp / 256) + 126);

  return motor_cmd;
}
