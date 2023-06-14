/**
* @Function: Handle ROS streams
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <functional>
#include <glog/logging.h>

#include "gici/stream/node_handle.h"
#include "gici/ros_interface/ros_stream.h"

namespace gici {

class RosNodeHandle : public NodeHandle {
public:
  RosNodeHandle(ros::NodeHandle& nh, const NodeOptionHandlePtr& nodes);
  ~RosNodeHandle();

protected:
  // Bind streamer->formator->ROS-streamer pipelines
  void bindStreamerToFormatorToRosStreamer(const NodeOptionHandlePtr& nodes);

  // Bind ROS-streamer->formator->streamer pipelines
  void bindRosStreamerToFormatorToStreamer(const NodeOptionHandlePtr& nodes);

  // Bind estimator->ROS-streamer pipelines
  void bindEstimatorToRosStreamer(const NodeOptionHandlePtr& nodes);

  // Clear streamer->formator->estimator pipeline and rebind them together with
  // ROS-streamer->estimator pipelines
  void rebindAllStreamerToEstimator(const NodeOptionHandlePtr& nodes);

  // Get ROS stream from tag
  inline std::shared_ptr<RosStream> getRosStreamFromTag(std::string tag) {
    for (auto ros_stream : ros_streams_) {
      if (ros_stream->getTag() == tag) return ros_stream;
    }
    return nullptr;
  }

protected:
  std::vector<std::shared_ptr<RosStream>> ros_streams_;
};

}
