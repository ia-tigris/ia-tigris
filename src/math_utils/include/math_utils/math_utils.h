/* Copyright 2014 Sanjiban Choudhury
 * math_utils.h
 *
 *  Created on: Oct 13, 2014
 *      Author: Sanjiban Choudhury
 */

#ifndef MATH_UTILS_INCLUDE_MATH_UTILS_MATH_UTILS_H_
#define MATH_UTILS_INCLUDE_MATH_UTILS_MATH_UTILS_H_

#include <Eigen/Core>
#include <vector>

namespace ca {

#define M_2PI 2.0*M_PI
#define CA_DEG2RAD M_PI/180
#define CA_RAD2DEG 180/M_PI

/**
 * \brief Contains various mathematical operations
 */
namespace math_utils {

namespace io {

bool LoadTableFromNormalFile(const std::string &filename, Eigen::MatrixXd &table);

bool LoadTableFromCSVFile(const std::string &filename, Eigen::MatrixXd &table);

bool SaveTableToCSVFile(const std::string &filename, const Eigen::MatrixXd &table);

bool SaveVecTableToCSVFile(const std::string &filename, const std::vector< std::vector<double> > &table);


}

/**
 * \brief Calculates an intermediate between two angles along the same arc
 * @param first_angle
 * @param second_angle
 * @param frac scalar value between 0 - 1, indicating position of intermediate angle
 * @return the intermediate angle
 */
double interpolate_angles(double first_angle, double second_angle, double frac);

/**
 * \brief contains constants
 */
namespace constants {

const double g = 9.81;

}

/**
 * \brief holds standard numeric operations
 */
namespace numeric_operations {

/**
 * \brief Wraps a given number between two boundaries
 * @param kX the number in question
 * @param kLowerBound the lower boundary
 * @param kUpperBound the upper boundary
 * @return the number wrapped
 */
int Wrap(int kX, int const kLowerBound, int const kUpperBound);

/**
 * \brief Returns a bounded version of a given number
 * The number will be bounded by +- some limit
 * @param limit the bound set
 * @param value the value to bound
 * @return the bounded value
 */
double Limit(double limit, double value);

/**
 * \brief Like regular division but checks to avoid division by 0
 * @param a the divisor
 * @param b the dividend
 * @return the result of a / b, with some approximation if b == 0
 */
double SafeDiv(double a, double b);

/**
 * \brief Returns the square root of a number, with checks on sign
 * @param v the number
 * @return The square root of the number, if it is positive, otherwise return 0
 */
double SafeSqrt(double v);

bool AreDoubleSame(double dFirstVal, double dSecondVal);

}

/**
 * \brief Holds operations related to numbers that exist on an arc
 */
namespace angular_math {

/**
 * \brief  Wrap to -pi->pi
 * @param a1 the number to wrap
 * @return the wrapped number
 */
double WrapToPi(double a1);

/**
 * \brief  Wrap to 0->2pi
 * @param a1 the number to wrap
 * @return the wrapped number
 */
double WrapTo2Pi(double a1);

/**
 * \brief Get difference between two angles.
 * @param command the angle to move to
 * @param current the angle to start
 * @return the angular difference between the commanded and current angle
 */
double TurnDirection(double command, double current);

}

/**
 * \brief contains operations involving vectors
 */
namespace vector_math {

/**
 * \brief Find the psi angle of vector
 * @param vec the vector
 * @return the psi angle
 */
double Angle(const Eigen::Vector3d &vec);

/**
 * \brief Find the psi angle between the two vectors
 * @param from first vector
 * @param to second vector
 * @return the pis angle
 */
double Angle(const Eigen::Vector3d &from, const Eigen::Vector3d &to);

/**
 * \brief Find the norm of vector
 * @param vec the vector
 * @return the 3D norm
 */
double Length(const Eigen::Vector3d &vec);

/**
 * \brief Find the norm of vector ignoring the last element
 * @param vec the vector
 * @return the 2D norm
 */
double Length2D(const Eigen::Vector3d &vec);


/**
 * \brief Normalize
 * @param vec the vector
 * @return the normalized vector
 */
//Eigen::Ref<const Eigen::VectorXd> SafeNormalize(const Eigen::Ref<const Eigen::VectorXd> &vec);
template <typename Derived>
Derived SafeNormalize(const Eigen::MatrixBase<Derived>& vec) {
  return vec.norm() > 0 ? vec.normalized() : Derived::Zero();
}

/**
 * \brief Are 2 vectors equal in terms of their first two components
 * @param vec1 first vector
 * @param vec2 second vector
 * @return true if their 2D components are equal (difference
 */
bool Equal2D(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2);

/**
 * \brief Cross product in 2D
 * @param vec1 first vector
 * @param vec2 second vector
 * @return the 2D cross product
 */
double Cross2D(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2);

/**
 * \brief Vector to line from candidate
 * \todo comment
 * @param line_start
 * @param line_end
 * @param candidate
 * @return
 */
Eigen::Vector3d VectorToLine(const Eigen::Vector3d &line_start,
                             const Eigen::Vector3d &line_end,
                             const Eigen::Vector3d &candidate);

/**
 * \todo comment
 * @param a
 * @param b
 * @param c
 * @param rv
 * @return
 */
Eigen::Vector3d VectorToLineSegment(const Eigen::Vector3d &a,
                                    const Eigen::Vector3d &b,
                                    const Eigen::Vector3d &c, double &rv);


/**
 * \brief Find nearest point on a line segment given a point
 * @param p - point p
 * @param l1 - point on line l
 * @param l2 - point on line l
 * @return nearest point on line l from point p
 */
Eigen::Vector3d NearestPointInLineSegment(const Eigen::Vector3d &p,const Eigen::Vector3d &l1, const Eigen::Vector3d &l2);


/**
 * \brief Find the 2D intersection of two vectors.
 *
 * If they don't intersect (parallel) or intersect backwards then false is returned.
 * @param from_start
 * @param from_end
 * @param to_start
 * @param to_end
 * @param intersection (output) the point of intersection
 * @return true if they intersect
 */
bool Intersection2D(const Eigen::Vector3d &from_start,
                    const Eigen::Vector3d &from_end,
                    const Eigen::Vector3d &to_start,
                    const Eigen::Vector3d &to_end,
                    Eigen::Vector3d &intersection);

/**
 * \brief Find the 2D intersection of two vectors.
 *
 * If they don't intersect (parallel) or intersect backwards then false is returned.
 * Intersection is limited to a distance offset from the interim segment.
 * @param from_start
 * @param from_end
 * @param to_start
 * @param to_end
 * @param offset_limit
 * @param intersection (output) point of intersection
 * @return true if they intersect
 */
bool Intersection2D(const Eigen::Vector3d &from_start,
                    const Eigen::Vector3d &from_end,
                    const Eigen::Vector3d &to_start,
                    const Eigen::Vector3d &to_end, double offset_limit,
                    Eigen::Vector3d &intersection);

/**
 * \brief Rotate vector along vertical axis.
 *
 * Angle specified by direction of another vector
 * @param to_rotate the vector to rotate
 * @param rotate_around the vector that specifies the angle to rotate
 * @return the rotated vector
 */
Eigen::Vector3d Rotate2DByVector(const Eigen::Vector3d &to_rotate,
                                 const Eigen::Vector3d &rotate_around);

/**
 * \brief Rotate vector along the vertical axis
 * @param to_rotate the vector to rotate
 * @param angle the angle along the axis to rotate
 * @return the rotated vector
 */
Eigen::Vector3d Rotate2DByAngle(const Eigen::Vector3d &to_rotate, double angle);

/**
 * \brief Computes the distance of a point to a plane
 * @param origin the location of the plane
 * @param to the normal vector of the plane
 * @param check the point
 * @return the distance
 */
double DistanceSidePlane(const Eigen::Ref<const Eigen::VectorXd> &origin,
                         const Eigen::Ref<const Eigen::VectorXd>& to,
                         const Eigen::Ref<const Eigen::VectorXd>& check);

/**
 * \brief Computes the fraction of a point to a plane
 * @param origin the location of the plane
 * @param to the normal vector of the plane
 * @param check the point
 * @return the distance
 */
double FractionSidePlane(const Eigen::Ref<const Eigen::VectorXd> &origin,
                         const Eigen::Ref<const Eigen::VectorXd>& to,
                         const Eigen::Ref<const Eigen::VectorXd>& check);

/**
 * \brief Bound components of vector, individually.
 *
 * They will be bounded by +/- the given limit.
 * @param limit specifies bound for each component
 * @param value the vector we want bounded
 * @return the bounded vector
 */
template <typename Derived>
Derived Limit(const Derived & limit,
                      const Derived & value) {
  return value.cwiseMin(limit).cwiseMax(-limit);
}

}  // namespace vector_math

/**@cond DONT_DOCUMENT */
namespace transform_math {

//Eigen::Vector3d GetStaticVel(const State &sc);

}

/**@endcond */

}  // namespace math_utils

}  // namespace ca

#endif  // MATH_UTILS_INCLUDE_MATH_UTILS_MATH_UTILS_H_ 
