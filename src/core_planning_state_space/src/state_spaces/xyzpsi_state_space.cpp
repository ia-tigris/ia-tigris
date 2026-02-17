/**
 * Copyright (c) 2015 Carnegie Mellon University, Sanjiban Choudhury <sanjiban@cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */


#include "core_planning_state_space/state_spaces/xyzpsi_state_space.h"

#include "ompl/tools/config/MagicConstants.h"

namespace ob = ompl::base;

void XYZPsiStateSpace::registerProjections(void) {
  class XYZPsiDefaultProjection : public ob::ProjectionEvaluator
  {
   public:

    XYZPsiDefaultProjection(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
    {
    }

    virtual unsigned int getDimension(void) const
    {
      return 3;
    }

    virtual void defaultCellSizes(void)
    {
      cellSizes_.resize(3);
      bounds_ = space_->as<XYZPsiStateSpace>()->getBounds();
      cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
      cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
      cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
    }

    virtual void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
    {
        projection = Eigen::Map<const Eigen::VectorXd>(state->as<XYZPsiStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 3);
//      memcpy(&projection(0), state->as<XYZPsiStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 3 * sizeof(double));
    }
  };

  registerDefaultProjection(std::make_shared<XYZPsiDefaultProjection>(this));
}



