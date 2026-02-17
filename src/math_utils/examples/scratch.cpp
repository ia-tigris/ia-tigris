/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sanjiban@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


/* Copyright 2015 Sanjiban Choudhury
 * scratch.cpp
 *
 *  Created on: Oct 9, 2015
 *      Author: Sanjiban Choudhury
 */

#include <Eigen/Dense>
#include <iostream>
#include <iterator>     // std::reverse_iterator
#include <vector>       // std::vector
#include "math_utils/math_utils.h"

namespace mu = ca::math_utils;

int main(int argc, char **argv) {

  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  t = mu::vector_math::SafeNormalize(t);

  std::cout << std::endl << t.transpose() << std::endl;


  Eigen::Vector2d p1(0, 0), p2(1, 0), p3(-2, 1);
  std::cout << "Distance: " << mu::vector_math::DistanceSidePlane(p1, p2, p3) << std::endl;

  Eigen::Vector2d val(-3, 4), lim(3.5, 3.5);
  std::cout << "Limited: " << mu::vector_math::Limit(lim, val).transpose() << std::endl;

  std::vector<int> vec = {1, 2, 3, 4};
  std::vector<int>::reverse_iterator rit = vec.rbegin();

  for(; rit != vec.rend(); ++rit ) {
    if( *rit < 2 )
      break;
  }

  std::cout << (rit == vec.rend()) << std::endl;
  vec.erase(rit.base(), vec.end());

  for (auto it2 : vec)
    std::cout << it2 << " ";
  std::cout << std::endl;
}

