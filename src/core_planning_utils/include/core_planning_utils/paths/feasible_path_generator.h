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
/* Copyright 2015 Sanjiban Choudhury
 * feasible_path_generator.h
 *
 *  Created on: Jan 31, 2015
 *      Author: Sanjiban Choudhury
 */

#ifndef PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_FEASIBLE_PATH_GENERATOR_H_
#define PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_FEASIBLE_PATH_GENERATOR_H_

#include "ompl/base/SpaceInformation.h"
#include "core_planning_utils/paths/path_waypoint.h"

namespace ca {
class FeasiblePathGenerator {
 public:
  FeasiblePathGenerator(const ompl::base::SpaceInformationPtr &si)
 : si_(si) {}
  virtual ~FeasiblePathGenerator(){}

  virtual bool GetFeasiblePath(const ompl::base::State *state1, const ompl::base::State *state2, planning_common::PathWaypoint &feasible_path, double resolution) = 0;

  void set_si(const ompl::base::SpaceInformationPtr &si) {si_ = si;}
 protected:
  ompl::base::SpaceInformationPtr si_;
};
}  // namespace ca



#endif  // PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_FEASIBLE_PATH_GENERATOR_H_ 
