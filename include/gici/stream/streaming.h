/**
* @Function: Handle stream thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <functional>
#include <glog/logging.h>

#include "gici/stream/formator.h"
#include "gici/stream/streamer.h"
#include "gici/estimate/estimator_types.h"
#include "gici/utility/node_option_handle.h"

namespace gici {

// I/O type
enum class StreamIOType {
  Input,
  Output,
  Log
};

class Streaming {
public:
  // Types
  using PipelineDirect = std::function<void(const uint8_t *buf, int size)>;
  using PipelineConvert = std::function<void(const std::string&, const std::shared_ptr<DataCluster>&)>;
  using DataClusters = std::vector<std::vector<std::shared_ptr<DataCluster>>>;
  using DataCallback = std::function<void(const std::string&, const std::shared_ptr<DataCluster>&)>;
  struct FormatorCtrl {
    std::string tag; 
    std::string input_tag; // tag of input formator 
    StreamIOType type;
    std::shared_ptr<FormatorBase> formator;
  };


  Streaming(const NodeOptionHandlePtr& nodes, size_t i_streamer);
  Streaming() : valid_(false), opened_(false) { }
  ~Streaming();

  // Get formators
  std::vector<FormatorCtrl>& getFormators() { return formators_; }

  // Set estimator data callback
  virtual void setDataCallback(const DataCallback& callback) {
    data_callbacks_.push_back(callback);
  }

  // Clear data callbacks
  void clearDataCallbacks() { data_callbacks_.clear(); }

  // Start thread
  void start();

  // Stop thread
  void stop();

  // Pipeline sends input data directly to logging stream
  void pipelineDirectCallback(const uint8_t *buf, int size);

  // Pipeline sends decoded data to logging stream
  void pipelineConvertCallback(
    const std::string& tag, const std::shared_ptr<DataCluster>& data);

  // Send solution data to output stream
  virtual void outputDataCallback(
    const std::string tag, const std::shared_ptr<DataCluster>& data);

  // Check if has formator tag
  inline bool hasFormatorTag(std::string tag) {
    for (size_t i = 0; i < formators_.size(); i++) {
      if (formators_[i].tag == tag) return true;
    }
    return false;
  }

  // Check if valid
  inline bool valid() { return valid_; }

  // Set PipelineConvert from ROS input
  void setPipelineConvert(const std::string& input_tag,
    const std::string& log_tag, const PipelineConvert& pipeline) {
    pipelines_convert_[input_tag].insert(std::make_pair(log_tag, pipeline));
  }

  // Get teg
  std::string getTag() { return tag_; }

  // Bind input and logging streams
  static void bindLogWithInput();

  // Enable replay
  // This function should be called after all the streamers are opened
  static void enableReplay(StreamerReplayOptions option);

  // Get instantiated objects
  static std::vector<Streaming *>& getObjects() { return static_this_; }

private:
  // Stream input processing
  void processInput();

  // Stream logging processing
  void processLogging();

  // Stream output processing
  void processOutput();

	// Loop processing
	void run();

private:
	// Thread handles
	std::unique_ptr<std::thread> thread_;
	std::mutex mutex_logging_, mutex_output_;
	bool quit_thread_ = false;
  double loop_duration_;

  // Stream control
  std::string tag_;
  std::string input_tag_;  // tag of input streamer
  bool valid_, opened_;
  std::shared_ptr<StreamerBase> streamer_;
  std::vector<FormatorCtrl> formators_;
  DataClusters data_clusters_;  // store data from each formators
  std::vector<DataCallback> data_callbacks_;  // call external function to send data out
  using PipelinesDirect = std::map<std::string, PipelineDirect>;
  PipelinesDirect pipelines_direct_; // sending data directly to logging streams
  // outer string: send from whom, inner string: who encode the data
  using PipelinesConvert = std::map<std::string, std::map<std::string, PipelineConvert>>;
  PipelinesConvert pipelines_convert_; // sending decoded data to logging streams
  uint8_t *buf_input_, *buf_logging_, *buf_output_;
  int buf_size_input_ = 0, buf_size_logging_ = 0, buf_size_output_ = 0;
  int max_buf_size_;
  bool has_input_ = false;
  bool has_logging_ = false;
  bool has_output_ = false;
  bool need_logging_ = false;
  bool need_output_ = false;

  // Static variables for stream binding
  static std::vector<Streaming *> static_this_;
};

}
