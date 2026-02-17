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
 * path_waypoint.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: Sanjiban Choudhury
 */

#include "core_planning_utils/paths/path_waypoint.h"


namespace ob = ompl::base;

namespace planning_common{

PathWaypoint::PathWaypoint(const PathWaypoint &path)
: PathParametric(path.si_) {
  CopyFrom(path);
}

PathWaypoint::PathWaypoint(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state1, const ompl::base::State *state2)
: PathParametric(si) {
  waypoints_.resize(2);
  waypoints_[0] = si_->cloneState(state1);
  waypoints_[1] = si_->cloneState(state2);
}

double PathWaypoint::length(void) const {
  return WorkspaceLength();/*
  double L = 0.0;
  for (unsigned int i = 1 ; i < waypoints_.size() ; ++i)
    L += si_->distance(waypoints_[i-1], waypoints_[i]);
  return L;*/
}

double PathWaypoint::WorkspaceLength(void) const {
  Eigen::MatrixXd param = GetParameter();
  double L = 0.0;
  for (std::size_t i = 0; i < param.rows() - 1; i++)
      L += (param.row(i+1) - param.row(i)).norm();
  return L;
}

bool PathWaypoint::check(void) const {
  for (unsigned int i = 0 ; i < waypoints_.size() ; ++i)
    if(!si_->satisfiesBounds(waypoints_[i]) || !si_->isValid(waypoints_[i]))
      return false;
  return true;
}

void PathWaypoint::print(std::ostream &out) const {
  out << "Waypoint path with " << waypoints_.size() << " states" << std::endl;
  for (unsigned int i = 0 ; i < waypoints_.size() ; ++i)
    si_->printState(waypoints_[i], out);
  out << std::endl;
}

void PathWaypoint::PrintAsMatrix(std::ostream &out) const {
  const ompl::base::StateSpace* space(si_->getStateSpace().get());
  std::vector<double> reals;
  for (unsigned int i = 0 ; i < waypoints_.size() ; ++i) {
    space->copyToReals(reals, waypoints_[i]);
    std::copy(reals.begin(), reals.end(), std::ostream_iterator<double>(out, " "));
    out << std::endl;
  }
  out << std::endl;
}

bool PathWaypoint::Interpolate(unsigned int num_waypoints) {
  if (num_waypoints < waypoints_.size() || waypoints_.size() < 2)
    return false;
  unsigned int count = num_waypoints;

  // the remaining length of the path we need to add states along
  double remainingLength = length();

  // the new array of states this path will have
  std::vector<ompl::base::State*> newStates;
  const int n1 = waypoints_.size() - 1;

  for (int i = 0 ; i < n1 ; ++i)
  {
    ompl::base::State *s1 = waypoints_[i];
    ompl::base::State *s2 = waypoints_[i + 1];

    newStates.push_back(s1);

    // the maximum number of states that can be added on the current motion (without its endpoints)
    // such that we can at least fit the remaining states
    int maxNStates = count + i - waypoints_.size();

    if (maxNStates > 0)
    {
      // compute an approximate number of states the following segment needs to contain; this includes endpoints
      double segmentLength = si_->distance(s1, s2);
      int ns = i + 1 == n1 ? maxNStates + 2 : (int)floor(0.5 + (double)count * segmentLength / remainingLength) + 1;

      // if more than endpoints are needed
      if (ns > 2)
      {
        ns -= 2; // subtract endpoints

        // make sure we don't add too many states
        if (ns > maxNStates)
          ns = maxNStates;

        // compute intermediate states
        std::vector<ompl::base::State*> block;
        unsigned int ans = si_->getMotionStates(s1, s2, block, ns, false, true);
        // sanity checks
        if ((int)ans != ns || block.size() != ans) {
          ROS_ERROR_STREAM("Internal error in path interpolation. Incorrect number of intermediate states. Please contact the developers.");
          return false;
        }

        newStates.insert(newStates.end(), block.begin(), block.end());
      }
      else
        ns = 0;

      // update what remains to be done
      count -= (ns + 1);
      remainingLength -= segmentLength;
    }
    else
      count--;
  }

  // add the last state
  newStates.push_back(waypoints_[n1]);
  waypoints_.swap(newStates);
  if (num_waypoints != waypoints_.size()) {
    ROS_ERROR_STREAM("Internal error in path interpolation. This should never happen. Please contact the developers.");
    return false;
  }
  return true;
}

bool PathWaypoint::InterpolateExclusive (unsigned int num_waypoints) {
  if (num_waypoints < waypoints_.size() || waypoints_.size() < 2)
    return false;
  unsigned int count = num_waypoints;

  // the remaining length of the path we need to add states along
  double remainingLength = length();

  // the new array of states this path will have
  std::vector<ompl::base::State*> newStates;
  const int n1 = waypoints_.size() - 1;

  for (int i = 0 ; i < n1 ; ++i)
  {
    ompl::base::State *s1 = waypoints_[i];
    ompl::base::State *s2 = waypoints_[i + 1];

    // the maximum number of states that can be added on the current motion (without its endpoints)
    // such that we can at least fit the remaining states
    int maxNStates = count + i - waypoints_.size() + 2;

    if (maxNStates > 0)
    {
      // compute an approximate number of states the following segment needs to contain; this includes endpoints
      double segmentLength = si_->distance(s1, s2);
      int ns = i + 1 == n1 ? maxNStates + 2 : (int)floor(0.5 + (double)count * segmentLength / remainingLength) + 1;

      // if more than endpoints are needed
      if (ns > 2)
      {
        // make sure we don't add too many states
        if (ns > maxNStates)
          ns = maxNStates;

        // compute intermediate states
        std::vector<ompl::base::State*> block;
        unsigned int ans = si_->getMotionStates(s1, s2, block, ns, false, true);
        // sanity checks
        if ((int)ans != ns || block.size() != ans) {
          ROS_ERROR_STREAM("ayyayInternal error in path interpolation. Incorrect number of intermediate states. Please contact the developers.");
          return false;
        }

        newStates.insert(newStates.end(), block.begin(), block.end());
      }
      else
        ns = 0;

      // update what remains to be done
      count -= (ns + 1);
      remainingLength -= segmentLength;
    }
    else
      count--;
  }

  waypoints_.swap(newStates);
  if (num_waypoints != waypoints_.size()) {
    ROS_ERROR_STREAM(num_waypoints<<" "<<waypoints_.size()<<"Internal error in path interpolation. This should never happen. Please contact the developers.");
    return false;
  }
  return true;
}

bool PathWaypoint::InterpWithDownSample (unsigned int num_waypoints) {
  if (waypoints_.size() < 2)
    return false;
  unsigned int count = num_waypoints;

  // the remaining length of the path we need to add states along
  double remainingLength = length();

  // the new array of states this path will have
  std::vector<ompl::base::State*> newStates;

  // push the first point
  newStates.push_back(waypoints_.front());

  double desired_length = remainingLength / ((double)num_waypoints - 1.0);
  double running_length = 0;

  const int n1 = waypoints_.size() - 1;

  for (int i = 0 ; i < n1 ; ++i)
  {
    ompl::base::State *s1 = waypoints_[i];
    ompl::base::State *s2 = waypoints_[i + 1];

    double segment_length = si_->distance(s1, s2);
    double remaining_segment_length = segment_length;
    while (running_length + remaining_segment_length > desired_length) {
      ob::State* to_add = si_->allocState();
      double alpha = std::min(1.0, std::max(0.0, 1 - (remaining_segment_length - (desired_length - running_length)) / segment_length));
      si_->getStateSpace()->interpolate(s1, s2, alpha, to_add);
      newStates.push_back(to_add);
      remaining_segment_length = segment_length - (desired_length - running_length);
      running_length = 0;
    }

    running_length += remaining_segment_length;
  }

  // add the last state

  if (newStates.size() < num_waypoints)
    newStates.push_back(waypoints_[n1]);
  waypoints_.swap(newStates);
  if (num_waypoints != waypoints_.size()) {
    ROS_ERROR_STREAM("Internal error in path interpolation with down sample. This should never happen. Please contact the developers.");
    return false;
  }
  return true;
}

bool PathWaypoint::EnforceDynamics (const ompl::base::State *start_constraints, const ompl::base::State *end_constraints, double tolerance) {/*
  if (boost::shared_ptr<ob::DubinsStateSpace> dubins_space = boost::dynamic_pointer_cast<ob::DubinsStateSpace>(si_->getStateSpace())) {
    // Assumptions: That waypoints are closely spaced. Hence the check really the finite difference version of xdot = f(x, u)
    // x,y wont be altered - theta will and then u=theta_dot will be checked for bound
    ob::ScopedState<ob::DubinsStateSpace> constrained_state(si_->getStateSpace(), start_constraints);
    for (std::size_t i = 0; i < waypoints_.size(); i++) {
      ob::ScopedState<ob::DubinsStateSpace> candidate_state(si_->getStateSpace(), waypoints_[i]);
      Eigen::Vector2d motion(candidate_state->getX() - constrained_state->getX(), candidate_state->getY() - constrained_state->getY());
      double dist = std::max(motion.norm(), std::numeric_limits<double>::epsilon());
      motion /= dist;
      Eigen::Vector2d constrained_dir(cos(constrained_state->getYaw()), sin(constrained_state->getYaw()));
      if (constrained_dir.dot(motion) - cos(dist/dubins_space->rho()) > -tolerance ) {
        candidate_state->setYaw( atan2(motion[1], motion[0]));
        si_->getStateSpace()->copyState(waypoints_[i], static_cast<const ob::State*>(candidate_state.get()));
        constrained_state = candidate_state;
      } else {
        return false;
      }
    }

    ob::ScopedState<ob::DubinsStateSpace> candidate_state(si_->getStateSpace(), end_constraints);
    Eigen::Vector2d motion(candidate_state->getX() - constrained_state->getX(), candidate_state->getY() - constrained_state->getY());
    double dist = std::max(motion.norm(), std::numeric_limits<double>::epsilon());
    motion /= dist;
    Eigen::Vector2d constrained_dir(cos(constrained_state->getYaw()), sin(constrained_state->getYaw()));
    if (constrained_dir.dot(motion) - cos(dist/dubins_space->rho()) > -tolerance ) {
      return true;
    } else {
      return false;
    }

  } else {
    return true;
  }*/ return true;
}





void PathWaypoint::Clear(void) {
  for (size_t i = 0 ; i < waypoints_.size() ; ++i)
    si_->freeState(waypoints_[i]);
  waypoints_.clear();
}


void PathWaypoint::FreeMemory(void) {
  for (size_t i = 0 ; i < waypoints_.size() ; ++i)
    si_->freeState(waypoints_[i]);
}

void PathWaypoint::CopyFrom(const PathWaypoint &other) {
  Clear();
  waypoints_.resize(other.waypoints_.size());
  for (size_t i = 0 ; i < waypoints_.size() ; ++i)
    waypoints_[i] = si_->cloneState(other.waypoints_[i]);
}

bool PathWaypoint::IsValid() {
  for (unsigned int i = 0 ; i < waypoints_.size() ; ++i)
    if(!si_->satisfiesBounds(waypoints_[i]) || !si_->isValid(waypoints_[i]))
      return false;
  return true;
}

double PathWaypoint::FractionInValid() {
  unsigned int num_invalid = 0;
  for (unsigned int i = 0 ; i < waypoints_.size() ; ++i)
    if(!si_->satisfiesBounds(waypoints_[i]) || !si_->isValid(waypoints_[i]))
      num_invalid++;
  return (double)num_invalid/(double)waypoints_.size();
}


PathWaypoint::waypoints::const_iterator PathWaypoint::CBegin(void) const { 
    return waypoints_.cbegin(); 

}

PathWaypoint::waypoints::const_iterator PathWaypoint::CEnd(void) const { 
    return waypoints_.cend();   
}


//PathWaypoint::waypoints::iterator PathWaypoint::Erase(waypoints::const_iterator first, waypoints::const_iterator last) { 
//    return waypoints_.erase(first, last);
//}

}  // namespace optimization

