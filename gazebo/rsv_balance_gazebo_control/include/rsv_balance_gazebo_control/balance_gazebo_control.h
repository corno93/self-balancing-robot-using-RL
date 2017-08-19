/*********************************************************************
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *********************************************************************/

#ifndef RSV_BALANCE_GAZEBO_CONTROL_BALANCE_GAZEBO_CONTROL_H
#define RSV_BALANCE_GAZEBO_CONTROL_BALANCE_GAZEBO_CONTROL_H

#include "rsv_balance_gazebo_control/control.h"

namespace balance_control
{

enum STATES
{
  theta,
  dx,
  dphi,
  dtheta
};

enum INPUTS
{
  tauL,
  tauR
};

/**
* Controller with simplest interface.
* Just enough to work.
*/
/** @todo How to log internal data of the controller? */
class BalanceControl
{
  public:
    BalanceControl();

    void resetControl();
    void stepControl(double dt, const double (&x_desired)[4], const double (&y_fbk)[4]);
    double *getControl();

  private:
    double t;
    double x_hat[4];
    double dx_hat[4];
    double x_reference[4];
    double x_r[4];
    double x_adjust[4];
    double u_output[2];
};

}  // namespace balance_control

#endif  //  RSV_BALANCE_GAZEBO_CONTROL_BALANCE_GAZEBO_CONTROL_H
