#ifndef HEADER_WHEELCONTROLLER
  #define HEADER_WHEELCONTROLLER

#include "fixedpoint.h"
#include "PID.h"

// Wheel PID parameters hardcoded here -tune these:
#define WHEEL_PID_P (fixed_point_t)0x00001000
#define WHEEL_PID_I (fixed_point_t)0x00000000
#define WHEEL_PID_D (fixed_point_t)0x00000000

// Wheel motor pins.


/**
 * Encapsulates a controller for a single motor/wheel - corresponding
 * PID parameters are defined here.
 */
class WheelController
{
  public:

  // The PID controller to use.
  PID pid;
  
  WheelController(fixed_point_t, fixed_point_t, fixed_point_t);
  ~WheelController();

  // Tick the PID, and return a motor command.
  char tick(int actual_rpm, int ref_rpm);
};



#endif
