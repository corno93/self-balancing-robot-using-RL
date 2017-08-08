

#ifndef HEADER_FIXEDPOINT
  #define HEADER_FIXEDPOINT

typedef signed long fixed_point_t;

#define FP_BYTES_AFTER_POINT    1

/**
 * Computes fixed point multiplication. Multiplies THEN shifts.
 *
 * @param l
 *  The left operand to multiply.
 * @param r
 *  The right operand to multiply.
 *
 * @returns The fixed point result of multiplication.
 */
fixed_point_t fp_mul(fixed_point_t l, fixed_point_t r);

#endif