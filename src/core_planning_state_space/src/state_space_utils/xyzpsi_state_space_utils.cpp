/**
 * Copyright (c) 2015 Carnegie Mellon University, Sanjiban Choudhury <sanjiban@cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

#include "core_planning_state_space/state_space_utils/xyzpsi_state_space_utils.h"


namespace ob = ompl::base;
namespace pc = planning_common;


ompl::base::SpaceInformationPtr GetStandardXYZPsiSpacePtr() {
  ob::StateSpacePtr space(new XYZPsiStateSpace());
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-0.5*std::numeric_limits<double>::max() + std::numeric_limits<double>::epsilon());
  bounds.setHigh(0.5*std::numeric_limits<double>::max());
  space->as<XYZPsiStateSpace>()->setBounds(bounds);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ob::AllValidStateValidityChecker(si)));
  si->setup();
  return si;
}


tf::Transform GetTF(const ompl::base::State *state) {
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(state->as<XYZPsiStateSpace::StateType>()->getX(), state->as<XYZPsiStateSpace::StateType>()->getY(), state->as<XYZPsiStateSpace::StateType>()->getZ()) );
  tf::Quaternion q;
  q.setRPY(0, 0, state->as<XYZPsiStateSpace::StateType>()->getPsi());
  transform.setRotation(q);
  return transform;
}


visualization_msgs::Marker GetMarker(boost::function< const ompl::base::State* (unsigned int)> GetStateFn,
                                     boost::function< std::size_t (void)> GetNumState,
                                     const ompl::base::SpaceInformationPtr& si,
                                     double scale, double r, double g, double b, double a) {

    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "path";
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = scale;
    m.type = visualization_msgs::Marker::LINE_STRIP; //correct marker type
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;

    // Fill up msg and return it
    geometry_msgs::Point ps;

    for (std::size_t i = 0 ; i < GetNumState(); i++) {

        auto *s = static_cast<const XYZPsiStateSpace::StateType*>(GetStateFn(i));

        Eigen::Vector3d vec = s->getXYZ();
        ps.x = vec.x();
        ps.y = vec.y();
        ps.z = vec.rows() == 3 ? vec.z() : 0;
        m.points.push_back(ps);
    }
    return m;

}

visualization_msgs::Marker GetMarker(const ompl::geometric::PathGeometric& path, double scale, double r, double g, double b, double a) {
    return GetMarker( boost::bind( static_cast< const ompl::base::State* (ompl::geometric::PathGeometric::*)( unsigned int ) const > (&ompl::geometric::PathGeometric::getState), &path, _1),
                      boost::bind(&ompl::geometric::PathGeometric::getStateCount, &path),
                      path.getSpaceInformation(),
                      scale, r, g, b, a);
}


void MakeHeadingTangent(pc::PathWaypoint &path) {
  if (path.Size() > 1) {
    for (size_t i = 0; i < path.GetWaypoints().size() - 1; i++) {
      Eigen::Vector3d vec1 = path.GetWaypoints()[i]->as<XYZPsiStateSpace::StateType>()->getXYZ();
      Eigen::Vector3d vec2 = path.GetWaypoints()[i+1]->as<XYZPsiStateSpace::StateType>()->getXYZ();
      double psi = atan2(vec2.y() - vec1.y(), vec2.x() - vec1.x());
      path.GetWaypoints()[i]->as<XYZPsiStateSpace::StateType>()->setPsi(psi);
      if (i == path.GetWaypoints().size() - 2) {
        path.GetWaypoints()[i+1]->as<XYZPsiStateSpace::StateType>()->setPsi(psi);
      }
    }
  }
}





