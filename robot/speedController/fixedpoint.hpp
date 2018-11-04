/**
  Fixedpoint arithmetic declartions.
  @author Alex Cornelio

*/

#ifndef HEADER_FIXEDPOINT
  #define HEADER_FIXEDPOINT

  
#include "Arduino.h"

typedef signed long fixed_point_t;

#define FP_BYTES_AFTER_POINT    1

fixed_point_t fp_mul(fixed_point_t l, fixed_point_t r);
fixed_point_t int16_fp(int value);
fixed_point_t fp_saturate(fixed_point_t val, fixed_point_t limit);

#endif
