/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sanjiban@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


/* Copyright 2013 Sanjiban Choudhury
 * vector_math.cpp
 *
 *  Created on: Dec 21, 2013
 *      Author: sanjiban
 */

#include "math_utils/math_utils.h"

namespace ca {
namespace nu = math_utils::numeric_operations;

double math_utils::vector_math::Angle(const Eigen::Vector3d &vec) {
  return atan2(vec.y(), vec.x());
}

double math_utils::vector_math::Angle(const Eigen::Vector3d  &from, const Eigen::Vector3d &to) {
  return math_utils::angular_math::WrapToPi(Angle(to) - Angle(from));
}

double math_utils::vector_math::Length(const Eigen::Vector3d &vec) {
  return sqrt(vec.x()*vec.x() + vec.y()*vec.y() + vec.z()*vec.z());
}


double math_utils::vector_math::Length2D(const Eigen::Vector3d &vec) {
  return sqrt(vec.x()*vec.x() + vec.y()*vec.y());
}


bool math_utils::vector_math::Equal2D(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2) {
  return math_utils::vector_math::Length2D(vec1 - vec2) < std::numeric_limits<double>::epsilon();
}

double math_utils::vector_math::Cross2D(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2) {
  return vec1.x()*vec2.y() - vec1.y()*vec2.x();
}

Eigen::Vector3d math_utils::vector_math::VectorToLine(const Eigen::Vector3d &line_start, const Eigen::Vector3d &line_end, const Eigen::Vector3d &candidate) {
  return (line_start - candidate) + (((candidate - line_start).dot(line_end - line_start))/((line_end-line_start).dot(line_end-line_start)))*(line_end-line_start);
}


Eigen::Vector3d math_utils::vector_math::VectorToLineSegment(const Eigen::Vector3d &a,const Eigen::Vector3d &b, const Eigen::Vector3d &c, double &rv) {
  Eigen::Vector3d AB(b - a);
  Eigen::Vector3d AC(c - a);
  double ABl2 = AB.dot(AB) + std::numeric_limits<double>::epsilon();
  double r = AC.dot(AB) / ABl2;
  Eigen::Vector3d result;
  if(r<0) {
    result = a - c;
    // Close to  A
  } else if(r>1) {
    // Closer to B
    result = b - c;
  } else {
    Eigen::Vector3d mix((1.0-r) * a + r * b);
    result = mix - c;
  }
  rv = r;
  return result;
}


Eigen::Vector3d math_utils::vector_math::NearestPointInLineSegment(const Eigen::Vector3d &p,const Eigen::Vector3d &l1, const Eigen::Vector3d &l2) {
  //Find nearest point on a line
  //A + dot(AP,AB) / dot(AB,AB) * AB
  //p_res = l1 + dot(l1p,l1l2) / dot(l1l2, l1l2) * l1l2
  Eigen::Vector3d l1l2(l2 - l1);
  Eigen::Vector3d l1p(p - l1);
  double A = l1p.dot(l1l2);
  Eigen::Vector3d tmp(l1l2);
  double B = tmp.dot(l1l2);
  Eigen::Vector3d aux((A/B) * l1l2);
  Eigen::Vector3d p_res(l1 + aux);
  
  //check if p_res between l1 and l2 - if yes then return p_res
  bool in_segment = true;
  if (l1.x() > l2.x()) {
    if ((p_res.x() > l1.x()) || (p_res.x() < l2.x())) in_segment = false;  
  }
  else {
    if ((p_res.x() < l1.x()) || (p_res.x() > l2.x())) in_segment = false;  
  }
  if (l1.y() > l2.y()) {
    if ((p_res.y() > l1.y()) || (p_res.y() < l2.y())) in_segment = false;  
  }
  else {
    if ((p_res.y() < l1.y()) || (p_res.y() > l2.y())) in_segment = false;  
  }
  if (l1.z() > l2.z()) {
    if ((p_res.z() > l1.z()) || (p_res.z() < l2.z())) in_segment = false;  
  }
  else {
    if ((p_res.z() < l1.z()) || (p_res.z() > l2.z())) in_segment = false;  
  }
  if (in_segment) return p_res;

  //not in the line segment - check which line segment extreme is closer
  double aux1 = math_utils::vector_math::Length(l1 - p);
  double aux2 = math_utils::vector_math::Length(l2 - p);
  if (aux1 < aux2) return l1;
  else             return l2;
}


bool math_utils::vector_math::Intersection2D(const Eigen::Vector3d &from_start, const Eigen::Vector3d &from_end, const Eigen::Vector3d &to_start, const Eigen::Vector3d &to_end, Eigen::Vector3d &intersection) {
  if (math_utils::vector_math::Equal2D(from_end, to_start)) {
    intersection = from_end;
    return true;
  }

  if (fabs(math_utils::vector_math::Cross2D(from_end - from_start, to_start - to_end)) < std::numeric_limits<double>::epsilon())
    return false;

  double l1 = math_utils::vector_math::Cross2D(to_start - from_end, to_start - to_end) / math_utils::vector_math::Cross2D(from_end - from_start, to_start - to_end);
  double l2 = math_utils::vector_math::Cross2D(to_start - from_end, from_end - from_start) / math_utils::vector_math::Cross2D(from_end - from_start, to_start - to_end);

  if (l1 < 0 || l2 < 0)
    return false;

  intersection = from_end + l1*(from_end - from_start);
  intersection[2] = (l2*from_end[2] + l1*to_start[2])/(l1+l2);
  return true;
}

bool math_utils::vector_math::Intersection2D(const Eigen::Vector3d &from_start, const Eigen::Vector3d &from_end, const Eigen::Vector3d &to_start, const Eigen::Vector3d &to_end, double offset_limit, Eigen::Vector3d &intersection) {
  if (!math_utils::vector_math::Intersection2D(from_start, from_end, to_start, to_end, intersection))
    return false;
  Eigen::Vector3d dir = math_utils::vector_math::VectorToLine(from_end, to_start, intersection);
  if (math_utils::vector_math::Length(dir) > offset_limit)
    intersection += (1 - offset_limit/math_utils::vector_math::Length(dir))*dir;
  return true;
}

Eigen::Vector3d math_utils::vector_math::Rotate2DByVector(const Eigen::Vector3d &to_rotate, const Eigen::Vector3d &rotate_around) {
  double psi = -math_utils::vector_math::Angle(rotate_around);
  Eigen::MatrixXd rot_2d(3,3);
  rot_2d << cos(psi), -sin(psi), 0,
      sin(psi), cos(psi), 0,
      0, 0, 1;
  return rot_2d*to_rotate;
}

Eigen::Vector3d math_utils::vector_math::Rotate2DByAngle(const Eigen::Vector3d &to_rotate, double psi) {
  Eigen::MatrixXd rot_2d(3,3);
  rot_2d << cos(psi), -sin(psi), 0,
      sin(psi), cos(psi), 0,
      0, 0, 1;
  return rot_2d*to_rotate;
}

double math_utils::vector_math::DistanceSidePlane(const Eigen::Ref<const Eigen::VectorXd>& origin,
                         const Eigen::Ref<const Eigen::VectorXd>& to,
                         const Eigen::Ref<const Eigen::VectorXd>& check) {
  Eigen::VectorXd vec(to - origin);
  vec.normalize();
  const Eigen::VectorXd vec2(check - origin);
  return vec.dot(vec2);
}


double math_utils::vector_math::FractionSidePlane(const Eigen::Ref<const Eigen::VectorXd>& origin,
                         const Eigen::Ref<const Eigen::VectorXd>& to,
                         const Eigen::Ref<const Eigen::VectorXd>& check) {
  const Eigen::VectorXd vec(to - origin);
  const Eigen::VectorXd vec2(check - origin);
  return nu::SafeDiv(vec.dot(vec2), vec.dot(vec));
}

//Eigen::Vector3d math_utils::vector_math::Limit(const Eigen::Vector3d & limit, const Eigen::Vector3d & value) {
//  return Eigen::Vector3d ( nu::Limit ( limit[0],value[0] ), nu::Limit ( limit[1],value[1] ), nu::Limit ( limit[2],value[2] ) );
//}

}
