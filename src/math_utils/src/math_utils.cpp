/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sanjiban@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


/* Copyright 2013 Sanjiban Choudhury
 * math_utils.cpp
 *
 *  Created on: Nov 3, 2013
 *      Author: sanjiban
 */


#include "math_utils/math_utils.h"

namespace ca {

double math_utils::interpolate_angles(double first_angle, double second_angle, double frac) {
  double diff = fabs(first_angle - second_angle);
  double orient_1 = (diff>M_PI && first_angle < 0) ? first_angle + 2*M_PI : first_angle;
  double orient_2 = (diff>M_PI && second_angle < 0) ? second_angle + 2*M_PI : second_angle;
  return math_utils::angular_math::WrapToPi((1 - frac) * orient_1 + (frac) * orient_2);
}

}


