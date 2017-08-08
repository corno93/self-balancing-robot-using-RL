

#include "fixedpoint.h"

fixed_point_t fp_mul(fixed_point_t l, fixed_point_t r) {
    
    fixed_point_t result;
    char sign;
    
    /* Compute the 32-bit * 32-bit -> 32-bit integer multiplication. */
    result = l * r;

    /* Save the sign. */
    sign = *((char *)&result + sizeof(fixed_point_t) - 1) & 0x80;

    /* Shift appropriate number of bytes downwards. */
    result >>= FP_BYTES_AFTER_POINT * 8;

    /* Restore the sign to the required number of bytes. */
    if (sign) {

        /* TODO: handle the general case ... */
        *((char *)&result + sizeof(fixed_point_t) - 1) = 0xFF;
    }

    return result;
}
