/*********************************************************************
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *********************************************************************/
#include "rsv_balance_gazebo_control/balance_gazebo_control.h"

namespace balance_control
{

BalanceControl::BalanceControl() {}

/*!
* \brief Resets all state and control variables.
*
* Useful when instantiating and reseting the control.
*/
void BalanceControl::resetControl()
{
  t = 0;
  for (int i=0; i < 4; i++)
  {
    x_hat[i] = 0;
    x_adjust[i] = 0;
    x_r[i] = 0;
  }
  u_output[0] = 0.0;
  u_output[1] = 0.0;
}

/*!
* \brief Integrates control and models.
*
* Integrates control with Euler method.
* \param dt       Step period.
* \param x_desired        Input array[4] for goal state
* \param y_fbk       Array[4] for sensor readings
*/
void BalanceControl::stepControl(double dt, const double (&x_desired)[4], const double (&y_fbk)[4])
{
  t += dt;
  //**************************************************************
  // Set reference state
  //**************************************************************
  x_reference[theta]  = x_desired[theta]  + x_adjust[theta];
  x_reference[dx]     = x_desired[dx]     + x_adjust[dx];
  x_reference[dphi]   = x_desired[dphi]   + x_adjust[dphi];
  x_reference[dtheta] = x_desired[dtheta] + x_adjust[dtheta];

  //**************************************************************
  // State estimation
  //**************************************************************
  // x_hat - x_ref
  float x_hat_x_ref[4];
  x_hat_x_ref[theta]  = x_hat[theta]  - x_reference[theta];
  x_hat_x_ref[dx]     = x_hat[dx]     - x_reference[dx];
  x_hat_x_ref[dphi]   = x_hat[dphi]   - x_reference[dphi];
  x_hat_x_ref[dtheta] = x_hat[dtheta] - x_reference[dtheta];

  // A*(x_hat-x_ref)
  dx_hat[theta]   = A[0][0]*x_hat_x_ref[theta] + A[0][1]*x_hat_x_ref[dx]
                  + A[0][2]*x_hat_x_ref[dphi]  + A[0][3]*x_hat_x_ref[dtheta];
  dx_hat[dx]      = A[1][0]*x_hat_x_ref[theta] + A[1][1]*x_hat_x_ref[dx]
                  + A[1][2]*x_hat_x_ref[dphi]  + A[1][3]*x_hat_x_ref[dtheta];
  dx_hat[dphi]    = A[2][0]*x_hat_x_ref[theta] + A[2][1]*x_hat_x_ref[dx]
                  + A[2][2]*x_hat_x_ref[dphi]  + A[2][3]*x_hat_x_ref[dtheta];
  dx_hat[dtheta]  = A[3][0]*x_hat_x_ref[theta] + A[3][1]*x_hat_x_ref[dx]
                  + A[3][2]*x_hat_x_ref[dphi]  + A[3][3]*x_hat_x_ref[dtheta];

  // + B*u_output
  dx_hat[theta]  += B[0][0]*u_output[tauL] + B[0][1]*u_output[tauR];
  dx_hat[dx]     += B[1][0]*u_output[tauL] + B[1][1]*u_output[tauR];
  dx_hat[dphi]   += B[2][0]*u_output[tauL] + B[2][1]*u_output[tauR];
  dx_hat[dtheta] += B[3][0]*u_output[tauL] + B[3][1]*u_output[tauR];

  // y - C*x_hat - x_desired
  float yC[4];
  yC[0] = y_fbk[theta]  - (C[0][0]*x_hat[theta] + C[0][1]*x_hat[dx] + C[0][2]*x_hat[dphi] + C[0][3]*x_hat[dtheta]);
  yC[1] = y_fbk[dx]     - (C[1][0]*x_hat[theta] + C[1][1]*x_hat[dx] + C[1][2]*x_hat[dphi] + C[1][3]*x_hat[dtheta]);
  yC[2] = y_fbk[dphi]   - (C[2][0]*x_hat[theta] + C[2][1]*x_hat[dx] + C[2][2]*x_hat[dphi] + C[2][3]*x_hat[dtheta]);
  yC[3] = y_fbk[dtheta] - (C[3][0]*x_hat[theta] + C[3][1]*x_hat[dx] + C[3][2]*x_hat[dphi] + C[3][3]*x_hat[dtheta]);

  // L*(y-C*x_hat)
  dx_hat[theta]  += L[0][0]*yC[0] + L[0][1]*yC[1] + L[0][2]*yC[2] + L[0][3]*yC[3];
  dx_hat[dx]     += L[1][0]*yC[0] + L[1][1]*yC[1] + L[1][2]*yC[2] + L[1][3]*yC[3];
  dx_hat[dphi]   += L[2][0]*yC[0] + L[2][1]*yC[1] + L[2][2]*yC[2] + L[2][3]*yC[3];
  dx_hat[dtheta] += L[3][0]*yC[0] + L[3][1]*yC[1] + L[3][2]*yC[2] + L[3][3]*yC[3];

  // Integrate Observer, Euler method:
  x_hat[theta]  += dx_hat[theta]*dt;
  x_hat[dx]     += dx_hat[dx]*dt;
  x_hat[dphi]   += dx_hat[dphi]*dt;
  x_hat[dtheta] += dx_hat[dtheta]*dt;

  //**************************************************************
  // Reference model
  //**************************************************************
  float x_r_x_ref[4];
  x_r_x_ref[theta]  = x_r[theta]  - x_reference[theta];
  x_r_x_ref[dx]     = x_r[dx]     - x_reference[dx];
  x_r_x_ref[dphi]   = x_r[dphi]   - x_reference[dphi];
  x_r_x_ref[dtheta] = x_r[dtheta] - x_reference[dtheta];
  x_r[theta]   += (A_BK[0][0]*x_r_x_ref[theta] + A_BK[0][1]*x_r_x_ref[dx]
                    + A_BK[0][2]*x_r_x_ref[dphi] + A_BK[0][3]*x_r_x_ref[dtheta])*dt;
  x_r[dx]      += (A_BK[1][0]*x_r_x_ref[theta] + A_BK[1][1]*x_r_x_ref[dx]
                    + A_BK[1][2]*x_r_x_ref[dphi] + A_BK[1][3]*x_r_x_ref[dtheta])*dt;
  x_r[dphi]    += (A_BK[2][0]*x_r_x_ref[theta] + A_BK[2][1]*x_r_x_ref[dx]
                    + A_BK[2][2]*x_r_x_ref[dphi] + A_BK[2][3]*x_r_x_ref[dtheta])*dt;
  x_r[dtheta]  += (A_BK[3][0]*x_r_x_ref[theta] + A_BK[3][1]*x_r_x_ref[dx]
                    + A_BK[3][2]*x_r_x_ref[dphi] + A_BK[3][3]*x_r_x_ref[dtheta])*dt;

  // Adjust theta based on reference model
  float e_r_dx;
  // e_r_dx = x_r[dx] - x_hat[dx];
  e_r_dx = x_r[dx] - y_fbk[dx];
  x_adjust[theta] += (.025*e_r_dx)*dt;

  //**************************************************************
  // Feedback control
  //**************************************************************
  u_output[tauL] = -(K[0][0]*(x_hat[theta]-x_reference[theta]) + K[0][1]*(x_hat[dx]-x_reference[dx])
                  + K[0][2]*(x_hat[dphi]-x_reference[dphi]) + K[0][3]*x_hat[dtheta]);
  u_output[tauR] = -(K[1][0]*(x_hat[theta]-x_reference[theta]) + K[1][1]*(x_hat[dx]-x_reference[dx])
                  + K[1][2]*(x_hat[dphi]-x_reference[dphi]) + K[1][3]*x_hat[dtheta]);
}

/*!
* \brief Set up the output array.
*
* Returns the address of the actuator torque array[2]
*/
double *BalanceControl::getControl()
{
  return u_output;
}
}  // namespace balance_control
