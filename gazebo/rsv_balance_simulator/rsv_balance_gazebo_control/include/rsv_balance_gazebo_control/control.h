/*********************************************************************
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *********************************************************************/

#ifndef RSV_BALANCE_GAZEBO_CONTROL_CONTROL_H
#define RSV_BALANCE_GAZEBO_CONTROL_CONTROL_H

extern const float g_fWheelRadius;
extern const float g_fBaseWidth;    // Half of Base width
extern const float g_fI3;           // I3, horizontal inertia

// System matrices
extern const float A[4][4];          // A matrix
extern const float B[4][2];          // B matrix
extern const float C[4][4];          // C matrix
extern const float L[4][4];          // L matrix
extern const float K[2][4];          // K matrix
extern const float A_BK[4][4];       // A-BK matrix

#endif  // RSV_BALANCE_GAZEBO_CONTROL_CONTROL_H
