/**
 * Copyright (c) 2015 Carnegie Mellon University, Sanjiban Choudhury <sanjiban@cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

#ifndef XYZPSI_STATE_SPACE_INCLUDE_XYZPSI_STATE_SPACE_XYZPSI_STATE_SPACE_H_
#define XYZPSI_STATE_SPACE_INCLUDE_XYZPSI_STATE_SPACE_XYZPSI_STATE_SPACE_H_


#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "core_planning_state_space/complex_state_space_types.h"
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>


class XYZPsiStateSpace : public ompl::base::CompoundStateSpace {
 public:
  class StateType : public ompl::base::CompoundStateSpace::StateType {
   public:
    StateType(void) : ompl::base::CompoundStateSpace::StateType() {}

    /** \brief Get the X component of the state */
    double getX(void) const {
        return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
    }

    /** \brief Get the Y component of the state */
    double getY(void) const {
        return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
    }

    /** \brief Get the Z component of the state */
    double getZ(void) const {
        return as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2];
    }

    Eigen::Vector3d getXYZ() const {
      return Eigen::Vector3d(getX(), getY(), getZ());
    }

    /** \brief Get the yaw component of the state. This is
        the rotation in plane, with respect to the Z
        axis. */
    double getPsi(void) const
    {
        return as<ompl::base::SO2StateSpace::StateType>(1)->value;
    }

    const ompl::base::RealVectorStateSpace::StateType& GetTranslation() const {
      return *as<ompl::base::RealVectorStateSpace::StateType>(0);
    }

    const ompl::base::SO2StateSpace::StateType& GetRotation() const {
      return *as<ompl::base::SO2StateSpace::StateType>(1);
    }

    /** \brief Set the X component of the state */
    void setX(double x)
    {
        as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = x;
    }

    /** \brief Set the Y component of the state */
    void setY(double y)
    {
        as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = y;
    }

    /** \brief Set the Y component of the state */
    void setZ(double z)
    {
        as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = z;
    }

    void setXYZ(const Eigen::Vector3d &pos)  {
      setX(pos.x());
      setY(pos.y());
      setZ(pos.z());
    }

    /** \brief Set the yaw component of the state. This is
        the rotation in plane, with respect to the Z
        axis. */
    void setPsi(double psi) {
        as<ompl::base::SO2StateSpace::StateType>(1)->value = psi;
        double v = fmod(as<ompl::base::SO2StateSpace::StateType>(1)->value, 2.0 * boost::math::constants::pi<double>());
        if (v <= -boost::math::constants::pi<double>())
            v += 2.0 * boost::math::constants::pi<double>();
        else
            if (v > boost::math::constants::pi<double>())
                v -= 2.0 * boost::math::constants::pi<double>();
        as<ompl::base::SO2StateSpace::StateType>(1)->value = v;
    }

    ompl::base::RealVectorStateSpace::StateType& GetTranslation() {
      return *as<ompl::base::RealVectorStateSpace::StateType>(0);
    }

    ompl::base::SO2StateSpace::StateType& GetRotation() {
      return *as<ompl::base::SO2StateSpace::StateType>(1);
    }

  };


  XYZPsiStateSpace()
  : CompoundStateSpace(){
    setName("XYZPsiStateSpace" + getName());
    type_ = XYZPSI_STATE_SPACE;

    ompl::base::StateSpacePtr space;

    space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
    addSubspace(space, 1.0);

    space = std::make_shared<ompl::base::SO2StateSpace>();
    addSubspace(space, 1.0);

    lock();
  }

  void setBounds(const ompl::base::RealVectorBounds &bounds) {
    as<ompl::base::RealVectorStateSpace>(0)->setBounds(bounds);
  }

  const ompl::base::RealVectorBounds& getBounds(void) const
  {
      return as<ompl::base::RealVectorStateSpace>(0)->getBounds();
  }

  virtual ~XYZPsiStateSpace(void) {}


  virtual void registerProjections(void);

};





#endif /* XYZPSI_STATE_SPACE_INCLUDE_XYZPSI_STATE_SPACE_XYZPSI_STATE_SPACE_H_ */
