

#include "fixedpoint.h"
#include "stdint.h"
#include "Arduino.h"

fixed_point_t fp_mul(fixed_point_t l, fixed_point_t r) {
    
    int64_t result;
    char sign;

    return ((int64_t)l * (int64_t)r) >> (8 * FP_BYTES_AFTER_POINT);

}

fixed_point_t int16_fp(int value)
{
  char sign = *((char *)&value + 1) & 0x80;

  if (!sign) {
    return (fixed_point_t)(value) << 8;
  } else {
    return 0xff000000 | ((fixed_point_t)(value) << 8);
  }
}

fixed_point_t fp_saturate(fixed_point_t val, fixed_point_t limit) {
  
  if (val < limit && val > -limit) {
    return val;
  } else if (val < -limit) {
    return -limit;
  } else {
    return limit;
  }
}




