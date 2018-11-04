/**
  Fixedpoint arithmetic functions. This has been implemented so we can avoid using arduino's float type, which is
  infamously slow. 
  The fixed_point_t type has the first 4 bytes for the mantissa and the last 4 bytes for the characteristic of the number.
  This way, we can maintain accuracy as well as speed - which is vital for a PID loop running at this speed.
  @author Alex Cornelio

*/

#include "fixedpoint.hpp"
#include "stdint.h"
#include "Arduino.h"

/**
  Return a fixed_point_t of two multipled fixed_point_t. Cast inputs as 64bits, multiply and then shift down 8 bits
*/
fixed_point_t fp_mul(fixed_point_t l, fixed_point_t r) {
    
    return ((int64_t)l * (int64_t)r) >> (8 * FP_BYTES_AFTER_POINT);

}

/**
  Convert type int to type fixed_point_t
*/
fixed_point_t int16_fp(int value)
{
  char sign = *((char *)&value + 1) & 0x80;

  if (!sign) {
    return (fixed_point_t)(value) << 8;
  } else {
    return 0xff000000 | ((fixed_point_t)(value) << 8);
  }
}

/**
  Apply limits to the control value so it doesn't over saturate
*/
fixed_point_t fp_saturate(fixed_point_t val, fixed_point_t limit) {
  
  if (val < limit && val > -limit) {
    return val;
  } else if (val < -limit) {
    return -limit;
  } else {
    return limit;
  }
}




