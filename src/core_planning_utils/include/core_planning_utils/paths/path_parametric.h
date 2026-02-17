/** * @author: AirLab / Field Robotics Center
 *
 * @attention Copyright (C) 2016
 * @attention Carnegie Mellon University
 * @attention All rights reserved
 *
 * @attention LIMITED RIGHTS:
 * @attention The US Government is granted Limited Rights to this Data.
 *            Use, duplication, or disclosure is subject to the
 *            restrictions as stated in Agreement AFS12-1642.
 */
/* Copyright 2014 Sanjiban Choudhury
 * path_parametric.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Sanjiban Choudhury
 */

#ifndef PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_PATH_PARAMETRIC_H_
#define PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_PATH_PARAMETRIC_H_


#include <vector>
#include <utility>
#include <Eigen/Dense>
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"

namespace planning_common{
/** \brief Definition of a paremetric path.
 * Any parametric path (waypoint based, splines, reproducing kernel hilbert spaces)
 * should be expressed by an Eigen matrix.
 * Optimization techniques can be done on these paths by perturbing these
 * parameters. For example, fmincon in MATLAB operates on a vector
 * of parameters. This additionally allows a matrix format that
 * is useful when dealing with multidimensional optimization
 *
 * Arrived at this parameterization after wasting a day of thinking. Committing to Eigen for lack of a better reason
 * */

class PathParametric : public ompl::base::Path {
 public:
  /** \brief shared pointer to ca::planning_common::PathParametric*/
  typedef std::shared_ptr<PathParametric> Ptr;

  /** \brief The structure of the gradient of a parameteric path */
  typedef Eigen::MatrixXd Gradient;

  /** \brief Construct a path instance for a given space information */
  PathParametric(const ompl::base::SpaceInformationPtr &si) : ompl::base::Path(si){};

  virtual ~PathParametric(){};

  /** \brief Factory method */

  /** \brief Return the length of a path */
  virtual double length(void) const = 0;

  /** \brief Check if the path is valid */
  virtual bool check(void) const = 0;

  /** \brief Print the path to a stream */
  virtual void print(std::ostream &out) const = 0;

  /** \brief Get parameter matrix
   * @return
   */
  virtual Eigen::MatrixXd GetParameter() const = 0;

  /** \brief Set parameter matrix
   * @param new_parameter Parameter matrix to override existing one.
   */
  virtual void SetParameter (const Eigen::MatrixXd &new_parameter) = 0;

  /** \brief Enforces dynamic constraints (if any). This function is a bit difficult to generalize
   * so only Dubins state space supported at the moment (nothing is done for other state spaces)
   * @param start_constraints Start state that cannot be perturbed
   * @param end_constraints End state that cannot be perturbed
   * @return true if it was successfull, false if not
   */
  virtual bool EnforceDynamics (const ompl::base::State *start_constraints, const ompl::base::State *end_constraints, double tolerance) = 0;

  virtual Eigen::MatrixXd JacobianWorkspace() const = 0;
};

}  // namespace planning_common



#endif  // PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_PATH_PARAMETRIC_H_ 
