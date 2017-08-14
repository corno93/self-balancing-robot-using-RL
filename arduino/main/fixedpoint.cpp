

#include "fixedpoint.h"
#include "stdint.h"
#include "Arduino.h"

fixed_point_t fp_mul(fixed_point_t l, fixed_point_t r) {
    
    int64_t result;
    char sign;

    return ((int64_t)l * (int64_t)r) >> (8 * FP_BYTES_AFTER_POINT);
    
//    /* Compute the 32-bit * 32-bit -> 64-bit integer multiplication. */
//    result = (int64_t)l * (int64_t)r;
//
//    Serial.println(*(long *)((char *)&result), HEX);
//    Serial.println(*(long *)((char *)&result + 4), HEX);
//
//    /* Get the sign of both l and r. */
//    sign_l = *((char *)&l - 3) & 0x80;
//    sign_r = *((char *)&r - 3) & 0x80;
//
//    /* Compute the sign of what the result should be. */
//    char sign_result = 1;
//    if (sign_l && sign_r || !sign_l && !sign_r) { sign_result = 0; }
//
//    /* Save the sign. */
//    sign = *((char *)&result + 8 - 1) & 0x80;
//
//    /* If the sign is negative but it should be positive: */
//    if (sign && !sign_result) {
//      return 0x7FFFFFFF;
//    }
//
//    if (!sign && sign_result) {
//      
//    }
//
//    Serial.println((int)sign);
//
//    /* Shift appropriate number of bytes downwards. */
//    result >>= FP_BYTES_AFTER_POINT * 8;
//
//    /* Restore the sign to the required number of bytes. */
//    if (sign) {
//
//        /* TODO: handle the general case ... */
//        *((char *)&result + sizeof(fixed_point_t) - 1) = 0xFF;
//    } else {
//      
//    }
//
//    return (fixed_point_t)result;
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




