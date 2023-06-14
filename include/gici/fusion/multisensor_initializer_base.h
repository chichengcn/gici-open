/**
* @Function: Base class for multisensor initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <memory>

#include "gici/estimate/estimator_base.h"

namespace gici {

// Initiliazer
class MultisensorInitializerBase : public virtual EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MultisensorInitializerBase(
    const EstimatorBaseOptions base_options) : 
    EstimatorBase(base_options) {}
  ~MultisensorInitializerBase() {}

  // (Re)set a graph pointer 
  inline void setGraph(const std::shared_ptr<Graph> graph) {
    graph_ = graph;
  }

  // Check if finished
  inline bool finished() const { return finished_; }

  // Arrange the initialization results to estimator
  // This function should be realized by initiliazers with specific input variables
  virtual bool arrangeToEstimator() { return false; }

protected:
  // flag
  bool finished_ = false;

  // Sub-estimator that gets solutions from raw data.
  // We apply loosely coupled initialization for all type fusion estimators. For tightly 
  // fusion estimator, we need this to compute the solutions from raw data.
  std::shared_ptr<EstimatorBase> sub_gnss_estimator_;
};

}