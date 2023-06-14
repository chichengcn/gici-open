/**
* @Function: Handle stream data input, log, and output
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>

#include "gici/stream/streaming.h"
#include "gici/estimate/estimating.h"
#include "gici/stream/data_integration.h"

namespace gici {

class NodeHandle {
public:
  NodeHandle(const NodeOptionHandlePtr& nodes);
  ~NodeHandle();

  // Get estimating handles
  // Used for dataset stream input
  const std::vector<std::shared_ptr<EstimatingBase>>& 
  getEstimatings() const { return estimatings_; }

protected:
  // Bind streamer->formator->estimator pipelines
  void bindStreamerToFormatorToEstimator(const NodeOptionHandlePtr& nodes);

  // Bind estimator->formator->streamer pipelines
  void bindEstimatorToFormatorToStreamer(const NodeOptionHandlePtr& nodes);

  // Bind estimator->estimator pipelines
  void bindEstimatorToEstimator(const NodeOptionHandlePtr& nodes);

  // Get streamer from given formator tag
  inline std::shared_ptr<Streaming> getStreamFromFormatorTag(std::string tag) {
    for (size_t i = 0; i < streamings_.size(); i++) {
      if (streamings_[i]->hasFormatorTag(tag)) return streamings_[i];
    }
    return nullptr;
  }

protected:
  // Streaming threads, handles streamer and formators
  std::vector<std::shared_ptr<Streaming>> streamings_;

  // Estimating threads, handles estimators
  std::vector<std::shared_ptr<EstimatingBase>> estimatings_;

  // Data integration handles that pack data according to its roles and send them to estimators
  // The outter vector aligns to estimatings_, which describes the data destinations.
  // The inner vector aligns to the input nodes of corresponding estimator.
  std::vector<std::vector<std::shared_ptr<DataIntegrationBase>>> data_integrations_;
};

}
