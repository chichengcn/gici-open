/**
* @Function: Handle stream data input, log, and output
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/node_handle.h"

#include "gici/fusion/multisensor_estimating.h"

namespace gici {

NodeHandle::NodeHandle(const NodeOptionHandlePtr& nodes)
{
  // Initialize streaming threads (formators are initialized together)
  for (size_t i = 0; i < nodes->streamers.size(); i++) {
    // check if ROS streamer
    std::string type_str = nodes->streamers[i]->type;
    StreamerType type;
    option_tools::convert(type_str, type);
    if (type == StreamerType::Ros) continue;

    auto streaming = std::make_shared<Streaming>(nodes, i);
    if (!streaming->valid()) continue;
    streamings_.push_back(streaming);
  }

  // Initialize estimator threads
  for (size_t i = 0; i < nodes->estimators.size(); i++) {
    std::string type_str = nodes->estimators[i]->type;
    EstimatorType type;
    option_tools::convert(type_str, type);

    if (type != EstimatorType::None) {
      auto estimating = std::make_shared<MultiSensorEstimating>(nodes, i);
      estimatings_.push_back(estimating);
    }
  }

  // Bind streamer->streamer, streamer->formator->formator->streamer pipelines
  Streaming::bindLogWithInput();

  // Bind streamer->formator->estimator pipelines
  bindStreamerToFormatorToEstimator(nodes);

  // Bind estimator->formator->streamer pipelines
  bindEstimatorToFormatorToStreamer(nodes);

  // Bind estimator->estimator pipelines
  bindEstimatorToEstimator(nodes);

  // Get replay option and enable replay
  bool enable_replay = false;
  StreamerReplayOptions replay_options;
  if (!nodes->replay_options.IsDefined() || 
      !option_tools::safeGet(nodes->replay_options, "enable", &enable_replay)) {
    LOG(INFO) << "Unable to load replay options. Disable replay!";
  }
  if (enable_replay) {
    if (!option_tools::safeGet(nodes->replay_options, "speed", &replay_options.speed)) {
      LOG(INFO) << "Unable to load replay speed! Using default instead";
      replay_options.speed = 1.0;
    }
    if (!option_tools::safeGet(nodes->replay_options, "start_offset", 
        &replay_options.start_offset)) {
      LOG(INFO) << "Unable to load replay start offset! Using default instead";
      replay_options.start_offset = 0.0;
    }
    Streaming::enableReplay(replay_options);
  }

  // Start streamings
  for (size_t i = 0; i < streamings_.size(); i++) {
    streamings_[i]->start();
  }

  // Start estimators
  for (size_t i = 0; i < estimatings_.size(); i++) {
    estimatings_[i]->start();
  }
}

NodeHandle::~NodeHandle()
{
  // Stop streamings
  for (size_t i = 0; i < streamings_.size(); i++) {
    streamings_[i]->stop();
  }

  // Stop estimators
  for (size_t i = 0; i < estimatings_.size(); i++) {
    estimatings_[i]->stop();
  }
}

// Bind streamer->formator->estimator pipelines
void NodeHandle::bindStreamerToFormatorToEstimator(const NodeOptionHandlePtr& nodes)
{
  for (auto estimating : estimatings_) {
    data_integrations_.push_back(std::vector<std::shared_ptr<DataIntegrationBase>>());
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
      if (input_tag.substr(0, 4) != "fmt_") continue;

      if (option_tools::sensorType(roles[0]) == SensorType::GNSS) {
        gnss_tags.push_back(input_tag);
        gnss_roles.push_back(roles);
        auto streaming_from_formator = getStreamFromFormatorTag(input_tag);
        bool found = false;
        for (auto streaming : gnss_streamings) {
          if (streaming_from_formator->getTag() == streaming->getTag()) {
            found = true; break;
          }
        }
        if (!found) gnss_streamings.push_back(streaming_from_formator);
      }
      else if (option_tools::sensorType(roles[0]) == SensorType::IMU) {
        imu_tags.push_back(input_tag);
        imu_roles.push_back(roles);
        auto streaming_from_formator = getStreamFromFormatorTag(input_tag);
        bool found = false;
        for (auto streaming : imu_streamings) {
          if (streaming_from_formator->getTag() == streaming->getTag()) {
            found = true; break;
          }
        }
        if (!found) imu_streamings.push_back(streaming_from_formator);
      }
      else if (option_tools::sensorType(roles[0]) == SensorType::Camera) {
        image_tags.push_back(input_tag);
        image_roles.push_back(roles);
        auto streaming_from_formator = getStreamFromFormatorTag(input_tag);
        bool found = false;
        for (auto streaming : image_streamings) {
          if (streaming_from_formator->getTag() == streaming->getTag()) {
            found = true; break;
          }
        }
        if (!found) image_streamings.push_back(streaming_from_formator);
      }
      else if (option_tools::sensorType(roles[0]) == SensorType::GeneralSolution) {
        solution_tags.push_back(input_tag);
        solution_roles.push_back(roles);
        auto streaming_from_formator = getStreamFromFormatorTag(input_tag);
        bool found = false;
        for (auto streaming : solution_streamings) {
          if (streaming_from_formator->getTag() == streaming->getTag()) {
            found = true; break;
          }
        }
        if (!found) solution_streamings.push_back(streaming_from_formator);
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
    data_integrations_.back() = data_integrations;
  }
}

// Bind estimator->formator->streamer pipelines
void NodeHandle::bindEstimatorToFormatorToStreamer(const NodeOptionHandlePtr& nodes)
{
  for (auto estimating : estimatings_) {
    std::string estimator_tag = estimating->getTag();
    NodeOptionHandle::EstimatorNodeBasePtr estimator_node = 
      std::static_pointer_cast<NodeOptionHandle::EstimatorNodeBase>(
        nodes->tag_to_node.at(estimator_tag));
    const auto& output_tags = estimator_node->output_tags;

    for (size_t i = 0; i < output_tags.size(); i++) {
      const std::string& output_tag = output_tags[i];

      // only handle formator output here
      if (output_tag.substr(0, 4) != "fmt_") continue;

      std::shared_ptr<Streaming> stream = getStreamFromFormatorTag(output_tag);
      if (stream == nullptr) continue;
      EstimatingBase::OutputDataCallback out_callback = std::bind(
        &Streaming::outputDataCallback, stream.get(), 
        std::placeholders::_1, std::placeholders::_2);
      estimating->setOutputDataCallback(out_callback);
    }
  }
}

// Bind estimator->estimator pipelines
void NodeHandle::bindEstimatorToEstimator(const NodeOptionHandlePtr& nodes)
{
  for (size_t k = 0; k < estimatings_.size(); k++) {
    auto& estimating = estimatings_[k];
    std::string estimator_tag = estimating->getTag();
    NodeOptionHandle::EstimatorNodeBasePtr estimator_node = 
      std::static_pointer_cast<NodeOptionHandle::EstimatorNodeBase>(
        nodes->tag_to_node.at(estimator_tag));
    const auto& input_tags = estimator_node->input_tags;
    const auto& input_tag_roles = estimator_node->input_tag_roles;
    CHECK(input_tags.size() == input_tag_roles.size());

    for (size_t i = 0; i < input_tags.size(); i++) {
      const std::string& input_tag = input_tags[i];
      const std::vector<std::string>& roles = input_tag_roles[i];

      // only handle estimator input here
      if (input_tag.substr(0, 4) != "est_") continue;

      // Check if roles valid
      for (auto role : roles) {
        SolutionRole solution_role;
        option_tools::convert(role, solution_role);
      }

      // initialize data integration handle
      auto& data_integrations = data_integrations_.at(k);
      for (auto estimating_input : estimatings_) {
        if (estimating_input->getTag() != input_tag) continue;
        data_integrations.push_back(std::make_shared<SolutionDataIntegration>(
          estimating, estimating_input, input_tag, roles));
      }
    }
  }
}

}