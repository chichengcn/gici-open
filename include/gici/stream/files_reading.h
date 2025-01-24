/**
* @Function: Thread to read all post files
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
#include "gici/stream/file_reader.h"
#include "gici/estimate/estimator_types.h"
#include "gici/utility/node_option_handle.h"

namespace gici {

class FilesReading {
public:
  // Types
  using SequentialDataClusters = std::map<
    double, std::pair<std::string, std::shared_ptr<DataCluster>>>;
  using DataCallback = std::function<void(
    const std::string&, const std::shared_ptr<DataCluster>&)>;


  FilesReading(const NodeOptionHandlePtr& nodes);
  FilesReading() { }
  ~FilesReading();

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

  // Set time duration for each loading
  void setStep(double step) { step_ = step; }

  // Set start offset
  void setStartOffset(double offset) { start_offset_ = offset; }

  // Check if valid
  bool valid() { return tags_.size() > 0; }

  // Check if tag exists
  bool hasTag(const std::string tag) { 
    return std::find(tags_.begin(), tags_.end(), tag) != tags_.end(); 
  }

private:
	// Loop processing
	void run();

private:
	// Thread handles
	std::unique_ptr<std::thread> thread_;
	bool quit_thread_ = false;
  double step_ = 0.1;  // time duration for each loading
  double start_offset_ = 0.0;

  // Stream control
  std::vector<std::string> tags_;
  std::vector<std::shared_ptr<FileReaderBase>> file_readers_;
  std::vector<DataCallback> data_callbacks_;  // call external function to send data out
};

}
