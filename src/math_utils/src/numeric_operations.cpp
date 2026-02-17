/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sanjiban@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


/* Copyright 2014 Sanjiban Choudhury
 * numeric_operations.cpp
 *
 *  Created on: Jan 6, 2014
 *      Author: sanjiban
 */


#include "math_utils/math_utils.h"


namespace ca {

int math_utils::numeric_operations::Wrap(int kX, int const kLowerBound, int const kUpperBound) {
    int range_size = kUpperBound - kLowerBound + 1;

    if (kX < kLowerBound)
        kX += range_size * ((kLowerBound - kX) / range_size + 1);

    return kLowerBound + (kX - kLowerBound) % range_size;
}

double math_utils::numeric_operations::Limit ( double limit, double value ) {
    if ( value>limit ) {
        return limit;
    } else {
        if ( value<-limit )
            return -limit;
        else
            return value;
    }
}

double math_utils::numeric_operations::SafeDiv(double a, double b) {
  if (fabs(b) > std::numeric_limits<double>::epsilon())
    return a/b;
  else
    return std::numeric_limits<double>::infinity();
}

double math_utils::numeric_operations::SafeSqrt(double v) {
  return std::sqrt(std::max(0.0, v));
}

bool math_utils::numeric_operations::AreDoubleSame(double dFirstVal, double dSecondVal) {
  return std::fabs(dFirstVal - dSecondVal) < std::numeric_limits<double>::epsilon();
}

}
