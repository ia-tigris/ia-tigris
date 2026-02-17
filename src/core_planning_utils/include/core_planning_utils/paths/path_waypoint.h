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
 * path_waypoint.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Sanjiban Choudhury
 */

#ifndef PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_PATH_WAYPOINT_H_
#define PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_PATH_WAYPOINT_H_

#include "core_planning_utils/paths/path_parametric.h"
#include "visualization_msgs/MarkerArray.h"
#include "ompl/base/ScopedState.h"
#include <ros/ros.h>

namespace planning_common{

/** \brief A waypoint representation of a path. This implies a vector of states
 * which denote a trajectory achieving these states at different time splices.
 */
class PathWaypoint : public PathParametric {
 public:
  /** \brief Shared pointer to ca::planning_common::PathWaypoint*/
  typedef boost::shared_ptr<PathWaypoint> Ptr;

  /** \brief Construct a path instance for a given space information */
  PathWaypoint(const ompl::base::SpaceInformationPtr &si) : PathParametric(si){};

  /** \brief Copy constructor */
  PathWaypoint(const PathWaypoint &path);

  /** \brief Construct a path instance from two states (thus making a segment)
   * @param si State information
   * @param state1 Start state
   * @param state2 End state
   */
  PathWaypoint(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state1, const ompl::base::State *state2);

  virtual ~PathWaypoint(){FreeMemory();}


  /** \brief Return the length of a path */
  virtual double length(void) const;

  virtual double WorkspaceLength(void) const;

  /** \brief Check if the path is valid */
  virtual bool check(void) const;

  /** \brief Print the path to a stream */
  virtual void print(std::ostream &out) const;

  /** \brief Print the path to a stream */
  virtual void PrintAsMatrix(std::ostream &out) const;

  /** \brief Get parameter set i.e. the waypoints in matrix form
   * @return Matrix of dim waypoints X statedim*/
  virtual Eigen::MatrixXd GetParameter() const;

  /** \brief Set parameter set*/
  virtual void SetParameter (const Eigen::MatrixXd &new_parameter);

  virtual Eigen::MatrixXd JacobianWorkspace() const;


  /** \brief Create a path by interpolating to create certian num of waypoints */
  bool Interpolate (unsigned int num_waypoints);

  /** \brief Create a path by interpolating. Throw out endpoints.
   * TODO @Sanjiban: Fix this copy pasting of code  */
  bool InterpolateExclusive (unsigned int num_waypoints);

  bool InterpWithDownSample (unsigned int num_waypoints);

  /** \brief Enforces dynamic constraints (if any). This function is a bit difficult to generalize
   * so only Dubins state space supported at the moment (nothing is done for other state spaces)
   * @param start_constraints Start state that cannot be perturbed
   * @param end_constraints End state that cannot be perturbed
   * @return true if it was successfull, false if not
   */
  virtual bool EnforceDynamics (const ompl::base::State *start_constraints, const ompl::base::State *end_constraints, double tolerance);

  /** \brief Get total number of waypoints */
  size_t NumberOfWaypoints() const{ return waypoints_.size(); }

  /** \brief Get the states that make up the path (as a reference, so it can be modified, hence the function is not const) */
  std::vector<ompl::base::State*>& GetWaypoints() { return waypoints_; }

  /** \brief Get the state located at \e index along the path */
  ompl::base::State* GetWaypoint(unsigned int index) {
      return waypoints_[index];
  }

  /** \brief Get the state located at \e index along the path */
  const ompl::base::State* GetWaypoint(unsigned int index) const {
      return waypoints_[index];
  }
  
  /** \brief Append a state by calling cloneState() */
  void AppendWaypoint(const ompl::base::State *state) {
	waypoints_.push_back(si_->cloneState(state)); 
  }

  void AppendPath(const PathWaypoint &path){
    for (waypoints::const_iterator it = path.CBegin(); it != path.CEnd(); ++it)
      waypoints_.push_back(si_->cloneState(*it));
  }

  void RemoveLastWayPoint() {
    si_->freeState(waypoints_.back());
    waypoints_.pop_back();
  }

  void RemoveFirstWayPoint() {
    si_->freeState(waypoints_.front());
    waypoints_.erase(waypoints_.begin());
  }

  void AppendWaypointToStart(const ompl::base::State *state) {
    waypoints_.insert(waypoints_.begin(),si_->cloneState(state));
  }


  /** \brief Clear */
  void Clear(void);

  /** \brief Copy data to this path from another path instance */
  void CopyFrom(const PathWaypoint& other);
 
  /** \brief Allow for stl compatible iterator interface */
  typedef std::vector<ompl::base::State*> waypoints;
  // src moved to src to prevent c++11 flags
  waypoints::const_iterator CBegin(void) const;// { return waypoints_.cbegin(); }
  waypoints::const_iterator CEnd(void) const;// { return waypoints_.cend();   }
  waypoints::iterator Begin(void) { return waypoints_.begin(); }
  waypoints::iterator End(void)  { return waypoints_.end();   }
  waypoints::size_type Size(void) const { return waypoints_.size();  }
  waypoints::const_reference Front(void) const { return waypoints_.front(); }
  waypoints::const_reference Back(void) const { return waypoints_.back(); }
  //TODO not so easy, have to free memory of each poiter seperately...
  //waypoints::iterator Erase(waypoints::const_iterator first, waypoints::const_iterator last); // { return waypoints_.erase(first, last);}
  bool IsValid();

  double FractionInValid();
 protected:

  /** \brief Free the memory corresponding to the states on this path */
  void FreeMemory(void);



  /** \brief The list of states that make up the path */
  std::vector<ompl::base::State*> waypoints_;
};

}  // namespace planning_common


#endif  // PLANNING_COMMON_INCLUDE_PLANNING_COMMON_PATHS_PATH_WAYPOINT_H_ 
