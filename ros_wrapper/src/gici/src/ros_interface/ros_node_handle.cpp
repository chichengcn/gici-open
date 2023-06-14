/**
* @Function: Handle stream data input, log, and output
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/ros_interface/ros_node_handle.h"

namespace gici {

RosNodeHandle::RosNodeHandle(ros::NodeHandle& nh, const NodeOptionHandlePtr& nodes) :
  NodeHandle(nodes)
{
  // Initialize ROS streamers
  for (size_t i = 0; i < nodes->streamers.size(); i++) {
    std::string type_str = nodes->streamers[i]->type;
    StreamerType type;
    option_tools::convert(type_str, type);
    if (type != StreamerType::Ros) continue;

    // check inputs and outputs
    int n_io = nodes->streamers[i]->input_tags.size() + 
               nodes->streamers[i]->output_tags.size();
    if (n_io == 0) {
      LOG(ERROR) << nodes->streamers[i]->tag << ": "
        << "At least one I/O port should be specified for a ROS stream!";
      continue;
    }
    if (n_io > 1) {
      LOG(ERROR) << nodes->streamers[i]->tag << ": "
        << "A ROS stream should has only one I/O port.";
      continue;
    }

    auto stream = std::make_shared<RosStream>(nh, nodes, i);
    if (!stream->valid()) continue;
    ros_streams_.push_back(stream);
  }

  // Check if we have ROS streams
  if (ros_streams_.size() == 0) return;

  // Clear streamer->formator->estimator pipeline and rebind them together with
  // ROS-streamer->estimator pipelines
  rebindAllStreamerToEstimator(nodes);

  // Bind ROS-streamer->ROS-streamer pipelines
  RosStream::bindLogWithInput();

  // Bind streamer->formator->ROS-streamer pipelines
  bindStreamerToFormatorToRosStreamer(nodes);

  // Bind ROS-streamer->formator->streamer pipelines
  bindRosStreamerToFormatorToStreamer(nodes);

  // Bind estimator->ROS-streamer pipelines
  bindEstimatorToRosStreamer(nodes);
}

RosNodeHandle::~RosNodeHandle()
{}

// Bind streamer->formator->ROS-streamer pipelines
void RosNodeHandle::bindStreamerToFormatorToRosStreamer(const NodeOptionHandlePtr& nodes)
{
  for (auto ros_stream : ros_streams_) {
    if (ros_stream->getIoType() != StreamIOType::Output) continue;

    std::string ros_stream_tag = ros_stream->getTag();
    NodeOptionHandle::StreamerNodeBasePtr ros_stream_node = 
      std::static_pointer_cast<NodeOptionHandle::StreamerNodeBase>(
        nodes->tag_to_node.at(ros_stream_tag));
    const auto& input_tags = ros_stream_node->input_tags;
    CHECK(input_tags.size() == 1);
    const std::string& input_tag = input_tags[0];

    // only handles input formator
    NodeOptionHandle::FormatorNodeBasePtr input_node = 
      std::static_pointer_cast<NodeOptionHandle::FormatorNodeBase>(
        nodes->tag_to_node.at(input_tag));
    if (input_node->io != "input") continue;

    // bind RosStream->outputDataCallback to streaming
    const auto& input_streaming = getStreamFromFormatorTag(input_tag);
    Streaming::DataCallback callback = std::bind(
      &RosStream::outputDataCallback, 
      ros_stream, std::placeholders::_1, std::placeholders::_2);
    input_streaming->setDataCallback(callback);
  }
}

// Bind ROS-streamer->formator->streamer pipelines
void RosNodeHandle::bindRosStreamerToFormatorToStreamer(const NodeOptionHandlePtr& nodes)
{
  for (auto ros_stream : ros_streams_) {
    if (ros_stream->getIoType() != StreamIOType::Input) continue;

    std::string ros_stream_tag = ros_stream->getTag();
    NodeOptionHandle::StreamerNodeBasePtr ros_stream_node = 
      std::static_pointer_cast<NodeOptionHandle::StreamerNodeBase>(
        nodes->tag_to_node.at(ros_stream_tag));

    // get output formators
    const auto& output_tags = ros_stream_node->output_tags;
    std::vector<std::string> output_formator_tags;
    for (const auto& output_tag : output_tags) {
      NodeOptionHandle::FormatorNodeBasePtr output_formator = 
        std::static_pointer_cast<NodeOptionHandle::FormatorNodeBase>(
          nodes->tag_to_node.at(output_tag));
      if (output_formator->io != "log") continue;
      output_formator_tags.push_back(output_tag);
    }

    // bind output_streaming->pipelineConvertCallback to ROS stream
    for (auto& output_tag : output_formator_tags) {
      const auto& output_streaming = getStreamFromFormatorTag(output_tag);
      Streaming::PipelineConvert pipeline = std::bind(
        &Streaming::pipelineConvertCallback, output_streaming.get(), 
        std::placeholders::_1, std::placeholders::_2);
      ros_stream->setDataCallback(pipeline);
    }
  }
}

// Bind estimator->ROS-streamer pipelines
void RosNodeHandle::bindEstimatorToRosStreamer(const NodeOptionHandlePtr& nodes)
{
  for (auto estimating : estimatings_) {
    std::string estimator_tag = estimating->getTag();
    NodeOptionHandle::EstimatorNodeBasePtr estimator_node = 
      std::static_pointer_cast<NodeOptionHandle::EstimatorNodeBase>(
        nodes->tag_to_node.at(estimator_tag));
    const auto& output_tags = estimator_node->output_tags;

    for (size_t i = 0; i < output_tags.size(); i++) {
      const std::string& output_tag = output_tags[i];

      // only handle streamer output here
      if (output_tag.substr(0, 4) != "str_") continue;

      std::shared_ptr<RosStream> stream = getRosStreamFromTag(output_tag);
      if (stream == nullptr) continue;
      EstimatingBase::OutputDataCallback out_callback = std::bind(
        &RosStream::outputDataCallback, stream.get(), 
        std::placeholders::_1, std::placeholders::_2);
      estimating->setOutputDataCallback(out_callback);
    }
  }
}

// Clear streamer->formator->estimator pipeline and rebind them together with
// ROS-streamer->estimator pipelines
void RosNodeHandle::rebindAllStreamerToEstimator(const NodeOptionHandlePtr& nodes)
{
  // Clear old pipelines. Because we should handle the data integrations together
  for (size_t i = 0; i < data_integrations_.size(); i++) {
    for (auto it = data_integrations_[i].begin(); it != data_integrations_[i].end();) {
      std::shared_ptr<SolutionDataIntegration> solution_data_integration = 
        std::dynamic_pointer_cast<SolutionDataIntegration>((*it));
      if (solution_data_integration && solution_data_integration->isFromEstimator()) {
        it++; 
      } 
      else {
        it = data_integrations_[i].erase(it);
      }
    }
  }
  for (auto& streaming : streamings_) streaming->clearDataCallbacks();

  // Rebind pipelines
  for (size_t i = 0; i < estimatings_.size(); i++) {
    auto& estimating = estimatings_[i];
    std::string estimator_tag = estimating->getTag();
    NodeOptionHandle::EstimatorNodeBasePtr estimator_node = 
      std::static_pointer_cast<NodeOptionHandle::EstimatorNodeBase>(
        nodes->tag_to_node.at(estimator_tag));
    const auto& input_tags = estimator_node->input_tags;
    const auto& input_tag_roles = estimator_node->input_tag_roles;
    CHECK(input_tags.size() == input_tag_roles.size());

    // distinguish sensors
    std::vector<std::string> gnss_tags, imu_tags, image_tags, solution_tags;
    std::vector<std::vector<std::string>> gnss_roles, imu_roles, image_roles, solution_roles;
    std::vector<std::shared_ptr<Streaming>> 
      gnss_streamings, imu_streamings, image_streamings, solution_streamings;
    for (size_t i = 0; i < input_tags.size(); i++) {
      const std::string& input_tag = input_tags[i];
      const std::vector<std::string>& roles = input_tag_roles[i];

      // only handle formator input here
      if (input_tag.substr(0, 4) != "str_" && input_tag.substr(0, 4) != "fmt_") continue;
      bool is_ros = (input_tag.substr(0, 4) == "str_");

      if (option_tools::sensorType(roles[0]) == SensorType::GNSS) {
        gnss_tags.push_back(input_tag);
        gnss_roles.push_back(roles);
        std::shared_ptr<Streaming> stream = is_ros ? 
          getRosStreamFromTag(input_tag) : getStreamFromFormatorTag(input_tag);
        if (!is_ros) {  // we do not need to check ROS because they are injective
          bool found = false;
          for (auto streaming : gnss_streamings) {
            if (stream->getTag() == streaming->getTag()) {
              found = true; break;
            }
          }
          if (!found) gnss_streamings.push_back(stream);
        }
        else gnss_streamings.push_back(stream);
      }
      else if (option_tools::sensorType(roles[0]) == SensorType::IMU) {
        imu_tags.push_back(input_tag);
        imu_roles.push_back(roles);
        std::shared_ptr<Streaming> stream = is_ros ? 
          getRosStreamFromTag(input_tag) : getStreamFromFormatorTag(input_tag);
        if (!is_ros) {  
          bool found = false;
          for (auto streaming : imu_streamings) {
            if (stream->getTag() == streaming->getTag()) {
              found = true; break;
            }
          }
          if (!found) imu_streamings.push_back(stream);
        }
        else imu_streamings.push_back(stream);
      }
      else if (option_tools::sensorType(roles[0]) == SensorType::Camera) {
        image_tags.push_back(input_tag);
        image_roles.push_back(roles);
        std::shared_ptr<Streaming> stream = is_ros ? 
          getRosStreamFromTag(input_tag) : getStreamFromFormatorTag(input_tag);
        if (!is_ros) {  
          bool found = false;
          for (auto streaming : image_streamings) {
            if (stream->getTag() == streaming->getTag()) {
              found = true; break;
            }
          }
          if (!found) image_streamings.push_back(stream);
        }
        else image_streamings.push_back(stream);
      }
      else if (option_tools::sensorType(roles[0]) == SensorType::GeneralSolution) {
        solution_tags.push_back(input_tag);
        solution_roles.push_back(roles);
        std::shared_ptr<Streaming> stream = is_ros ? 
          getRosStreamFromTag(input_tag) : getStreamFromFormatorTag(input_tag);
        if (!is_ros) {  
          bool found = false;
          for (auto streaming : solution_streamings) {
            if (stream->getTag() == streaming->getTag()) {
              found = true; break;
            }
          }
          if (!found) solution_streamings.push_back(stream);
        }
        else solution_streamings.push_back(stream);
      }
    }

    // initialize data integration handles
    std::vector<std::shared_ptr<DataIntegrationBase>> data_integrations;
    data_integrations.push_back(std::make_shared<GnssDataIntegration>(
      estimating, gnss_streamings, gnss_tags, gnss_roles));
    data_integrations.push_back(std::make_shared<ImuDataIntegration>(
      estimating, imu_streamings, imu_tags, imu_roles));
    data_integrations.push_back(std::make_shared<ImageDataIntegration>(
      estimating, image_streamings, image_tags, image_roles));
    data_integrations.push_back(std::make_shared<SolutionDataIntegration>(
      estimating, solution_streamings, solution_tags, solution_roles));
    pushBatchBack(data_integrations_[i], data_integrations);
  }
}

}