/**
  Controller header file
  @author Alex Cornelio

*/

#ifndef HEADER_WHEELCONTROLLER
  #define HEADER_WHEELCONTROLLER

#include "fixedpoint.hpp"
#include "PID.hpp"


/**
  Encapsulates a controller for a single motor
*/
class WheelController
{
  public:

  // The PID controller to use
  PID pid;
  
  WheelController(fixed_point_t, fixed_point_t, fixed_point_t);
  ~WheelController();

  // Tick the PID, and return a motor command.
  char tick(int actual_rpm, int ref_rpm);
};



#endif
