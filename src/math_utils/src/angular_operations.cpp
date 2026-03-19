/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sanjiban@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


/* Copyright 2013 Sanjiban Choudhury
 * angular_operations.cpp
 *
 *  Created on: Dec 21, 2013
 *      Author: sanjiban
 */

#include "math_utils/math_utils.h"

namespace ca {

double math_utils::angular_math::WrapToPi(double a1) {
  int m = ( int ) ( a1/ ( 2*M_PI ) );
  a1 = a1 - m*2*M_PI;
  if(a1>M_PI)
    a1-= 2.0*M_PI;
  else if(a1<-M_PI)
    a1+=2.0*M_PI;
  return a1;
}

double math_utils::angular_math::WrapTo2Pi(double a1) {
  return a1 - 2*M_PI * floor(a1 / (2*M_PI));
}

double math_utils::angular_math::TurnDirection ( double command, double current )
{
    command =  WrapToPi(command);
    current = WrapToPi(current);
    double error = command - current;
    if(error < -M_PI)
        error = 2.0 * M_PI + error;
    if(error > M_PI)
        error = -2.0 * M_PI + error;
    return error;
}

}
