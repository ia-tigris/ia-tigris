/**
 * Copyright (c) 2015 Carnegie Mellon University, Sanjiban Choudhury <sanjiban@cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

#ifndef XYZPSI_STATE_SPACE_INCLUDE_XYZPSI_STATE_SPACE_XYZPSI_STATE_SPACE_UTILS_H_
#define XYZPSI_STATE_SPACE_INCLUDE_XYZPSI_STATE_SPACE_XYZPSI_STATE_SPACE_UTILS_H_

#include "core_planning_state_space/state_spaces/xyzpsi_state_space.h"
#include "core_planning_utils/paths/path_waypoint.h"
#include "ompl/base/ScopedState.h"
#include "tf/tf.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerData.h"
#include "visualization_msgs/MarkerArray.h"

ompl::base::SpaceInformationPtr GetStandardXYZPsiSpacePtr();

tf::Transform GetTF(const ompl::base::State *state);

void MakeHeadingTangent(planning_common::PathWaypoint &path);

visualization_msgs::Marker GetMarker(boost::function< const ompl::base::State* (unsigned int)> GetStateFn,
                                     boost::function< std::size_t (void)> GetNumState,
                                     const ompl::base::SpaceInformationPtr& si,
                                     double scale=1, double r=0, double g=1, double b=0, double a=1);

visualization_msgs::Marker GetMarker(const ompl::geometric::PathGeometric& path, double scale=1, double r=0, double g=1, double b=0, double a=1);





#endif /* XYZPSI_STATE_SPACE_INCLUDE_XYZPSI_STATE_SPACE_XYZPSI_STATE_SPACE_UTILS_H_ */
